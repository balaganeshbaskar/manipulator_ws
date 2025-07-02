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
            self.get_logger().info('Simulation mode enabled — not using serial port')

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.moveit_stm32_latency_pub = self.create_publisher(Float32, '/moveit_stm32_time', 10)
        self.stm32_output_latency_pub = self.create_publisher(Float32, '/stm32_output_time', 10)

        self.trajectory_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_group_controller/state',
            self.trajectory_callback,
            10
        )

        if not self.simulate:
            self.timer = self.create_timer(0.05, self.read_from_stm32)

        self.last_sent = None
        self.last_sent_time = self.get_clock().now() - Duration(seconds=1)

    def compute_checksum(self, data: bytes):
        return sum(data) & 0xFF

    def send_to_stm32(self, joint_positions):
        now = self.get_clock().now()
        self.last_sent_time = now

        if hasattr(self, 'moveit_input_time'):
            moveit_latency = (now.nanoseconds - self.moveit_input_time.nanoseconds) / 1e6
            latency_msg = Float32()
            latency_msg.data = moveit_latency
            self.moveit_stm32_latency_pub.publish(latency_msg)
            del self.moveit_input_time

        data = b''.join(struct.pack('<h', int(value * 1000)) for value in joint_positions)
        payload = bytes([0x01, 0x10, len(data)]) + data
        checksum = self.compute_checksum(payload)
        message = bytes([STX]) + payload + bytes([checksum, ETX])

        if self.simulate:
            self.get_logger().info(f'Simulated >> TX (no serial): {message.hex()}')
        else:
            self.serial_port.write(message)
            self.get_logger().info(f'>> TX: {message.hex()}')

    def read_from_stm32(self):
        if self.serial_port.in_waiting:
            raw = self.serial_port.read_until(expected=bytes([ETX]))
            if len(raw) < 6 or raw[0] != STX or raw[-1] != ETX:
                return

            payload = raw[1:-2]
            checksum = raw[-2]

            if self.compute_checksum(payload) != checksum:
                self.get_logger().warn('Checksum mismatch')
                return

            id_, cmd, length = payload[0], payload[1], payload[2]
            data = payload[3:]

            if len(data) != length:
                self.get_logger().warn('Length mismatch')
                return

            joint_values = [struct.unpack('<h', data[i:i+2])[0] / 1000.0 for i in range(0, length, 2)]

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [f'joint_{i+1}' for i in range(len(joint_values))]
            js.position = joint_values
            self.joint_state_pub.publish(js)

            if hasattr(self, 'last_sent_time'):
                now = self.get_clock().now()
                rtt_latency = (now.nanoseconds - self.last_sent_time.nanoseconds) / 1e6
                rtt_msg = Float32()
                rtt_msg.data = rtt_latency
                self.stm32_output_latency_pub.publish(rtt_msg)
                del self.last_sent_time

            self.get_logger().info(f'<< RX joints: {joint_values}')
            self.get_logger().info(f'<< RX raw: {raw.hex()}')

    def trajectory_callback(self, msg: JointTrajectoryControllerState):
        if not msg.actual.positions:
            return

        joint_positions = list(msg.actual.positions[:5])
        now = self.get_clock().now()

        if not hasattr(self, 'last_sent') or self._position_changed(joint_positions):
            self.moveit_input_time = now
            self.send_to_stm32(joint_positions)
            self.last_sent = joint_positions
        elif hasattr(self, 'last_sent_time'):
            time_since_last = now - self.last_sent_time
            if time_since_last.nanoseconds > 500_000_000:  # 500ms
                self.send_to_stm32(joint_positions)
                self.last_sent_time = now

    def _position_changed(self, new_pos, threshold=0.01):
        if self.last_sent is None:
            return True
        return any(abs(a - b) > threshold for a, b in zip(self.last_sent, new_pos))

def main(args=None):
    rclpy.init(args=args)
    node = STM32UARTNode(simulate=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down STM32 UART node')
    finally:
        if not node.simulate:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()
