import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.time import Time, Duration

STX = 0x02
ETX = 0x03

class STM32UARTNode(Node):
    def __init__(self, simulate=False):
        super().__init__('stm32_uart_node')
        self.simulate = simulate

        if not self.simulate:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        else:
            self.get_logger().info('Simulation mode enabled — no serial connection used.')

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.moveit_stm32_latency_pub = self.create_publisher(Float32, '/moveit_stm32_time', 10)
        self.stm32_output_latency_pub = self.create_publisher(Float32, '/stm32_output_time', 10)

        self.trajectory_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_group_controller/state',
            self.trajectory_callback,
            10
        )

        if not self.simulate:
            self.timer = self.create_timer(0.1, self.read_from_stm32)

        self.last_sent_time = self.get_clock().now()
        self.waiting_for_ack = False
        self.trajectory_queue = []
        self.current_step = 0

        self.consecutive_parse_errors = 0
        self.max_parse_errors = 5  # Customize this threshold


        self.last_queued = None

        self.send_timer = self.create_timer(0.05, self.send_next_point)
        self.get_logger().info('STM32 UART Node initialized and waiting for MoveIt...')

        self.read_buffer = bytearray()

    def compute_checksum(self, data: bytes):
        return sum(data) & 0xFF

    def send_to_stm32(self, joint_positions):
        now = self.get_clock().now()
        self.last_sent_time = now

        data = b''.join(struct.pack('<h', int(value * 1000)) for value in joint_positions)
        payload = bytes([0x01, 0x10, len(data)]) + data
        checksum = self.compute_checksum(payload)
        message = bytes([STX]) + payload + bytes([checksum, ETX])

        if self.simulate:
            self.get_logger().info(f'[SIM] TX: {message.hex()}')
        else:
            self.serial_port.write(message)
            self.get_logger().info(f'TX: {message.hex()}')

        self.waiting_for_ack = True

    def send_next_point(self):
        if not self.trajectory_queue or self.waiting_for_ack:
            return

        next_point = self.trajectory_queue.pop(0)
        self.send_to_stm32(next_point)

    

    def read_from_stm32(self):
        try:
            while self.serial_port.in_waiting:
                byte = self.serial_port.read()
                self.read_buffer.extend(byte)

                while STX in self.read_buffer and ETX in self.read_buffer:
                    try:
                        start_idx = self.read_buffer.index(STX)
                        end_idx = self.read_buffer.index(ETX, start_idx)
                    except ValueError:
                        # ETX not found after STX
                        break

                    frame = self.read_buffer[start_idx:end_idx + 1]
                    self.read_buffer = self.read_buffer[end_idx + 1:]

                    if len(frame) < 6:
                        self.get_logger().warn('Discarded short frame')
                        self.consecutive_parse_errors += 1
                        self.waiting_for_ack = False
                        break

                    if frame[0] != STX or frame[-1] != ETX:
                        self.get_logger().warn('Invalid start or end byte')
                        self.consecutive_parse_errors += 1
                        self.waiting_for_ack = False
                        break

                    payload = frame[1:-2]
                    checksum = frame[-2]

                    if self.compute_checksum(payload) != checksum:
                        # self.get_logger().warn('Checksum mismatch, discarding frame')
                        self.get_logger().warn(f'Checksum mismatch, discarding frame: {frame.hex()}')
                        self.consecutive_parse_errors += 1
                        self.waiting_for_ack = False
                        break

                    id_, cmd, length = payload[0], payload[1], payload[2]
                    data = payload[3:]

                    if len(data) != length:
                        self.get_logger().warn('Length mismatch, discarding frame')
                        self.consecutive_parse_errors += 1
                        self.waiting_for_ack = False
                        break

                    # ✅ Frame is valid — parse and publish
                    joint_values = [struct.unpack('<h', data[i:i+2])[0] / 1000.0 for i in range(0, length, 2)]

                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    js.name = [f'joint_{i+1}' for i in range(len(joint_values))]
                    js.position = joint_values
                    self.joint_state_pub.publish(js)

                    now = self.get_clock().now()
                    rtt = (now.nanoseconds - self.last_sent_time.nanoseconds) / 1e6
                    latency_msg = Float32()
                    latency_msg.data = rtt
                    self.stm32_output_latency_pub.publish(latency_msg)

                    self.get_logger().info(f'RX: {joint_values}')
                    self.waiting_for_ack = False
                    self.consecutive_parse_errors = 0  # ✅ Reset on success

            # 🧹 Too many parse errors? Reset buffer
            if self.consecutive_parse_errors >= self.max_parse_errors:
                self.get_logger().warn('Too many parse errors — resetting buffer')
                self.read_buffer.clear()
                self.consecutive_parse_errors = 0

        except Exception as e:
            self.get_logger().error(f'Exception in read_from_stm32: {str(e)}')








    def trajectory_callback(self, msg: JointTrajectoryControllerState):
        if not msg.actual.positions:
            self.get_logger().warn("Received trajectory message without positions")
            self.waiting_for_ack = False
            return

        positions = list(msg.actual.positions[:5])

        if self.last_queued is None:
            self.trajectory_queue.append(positions)
            self.last_queued = positions
            self.get_logger().info(f"[First] Queued trajectory point: {positions}")
        elif self._position_changed(positions, self.last_queued):
            self.trajectory_queue.append(positions)
            self.last_queued = positions

    def _position_changed(self, new_pos, prev_pos, threshold=0.001):
        return any(abs(a - b) > threshold for a, b in zip(new_pos, prev_pos))

def main(args=None):
    rclpy.init(args=args)
    node = STM32UARTNode(simulate=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down STM32 UART node...')
    finally:
        if not node.simulate:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()
