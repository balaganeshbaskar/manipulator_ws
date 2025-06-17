import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import struct
from control_msgs.msg import JointTrajectoryControllerState  # ✅ Import MoveIt controller state

STX = 0x02
ETX = 0x03

class STM32UARTNode(Node):
    def __init__(self, simulate=False):
        super().__init__('stm32_uart_node')
        self.simulate = simulate

        if not self.simulate:
            self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
        else:
            self.get_logger().info('Simulation mode enabled — not using serial port')

        self.publisher_ = self.create_publisher(Float64MultiArray, '/stm32_rx_joints', 10)
        self.latency_pub = self.create_publisher(Float32, '/test_output_latency', 10)
        self.feedback_pub = self.create_publisher(JointState, '/test_output_feedback', 10)
        self.tx_latency_pub = self.create_publisher(Float32, '/test_tx_latency', 10)

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/stm32_tx_joints',
            self.send_to_stm32,
            10
        )

        self.trajectory_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_group_controller/state',
            self.trajectory_callback,
            10
        )

        if not self.simulate:
            self.timer = self.create_timer(0.05, self.read_from_stm32)

    def compute_checksum(self, data: bytes):
        return sum(data) & 0xFF

    def send_to_stm32(self, msg: Float64MultiArray):
        timestamp_sent = self.get_clock().now()

        data = b''
        for value in msg.data:
            data += struct.pack('<h', int(value * 1000))

        payload = bytes([0x01, 0x10, len(data)]) + data
        checksum = self.compute_checksum(payload)
        message = bytes([STX]) + payload + bytes([checksum, ETX])

        if self.simulate:
            self.get_logger().info(f'Simulated  >> TX (no serial): {message.hex()}')
        else:
            self.serial_port.write(message)
            self.get_logger().info(f'>> TX: {message.hex()}')

        # Record sent timestamp for round-trip latency measurement
        self.last_sent_time = timestamp_sent

        # Publish TX latency (from message in to message encoded and sent)
        now = self.get_clock().now()
        tx_latency = (now.nanoseconds - timestamp_sent.nanoseconds) / 1e6  # in ms
        tx_msg = Float32()
        tx_msg.data = tx_latency
        self.tx_latency_pub.publish(tx_msg)

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

            joint_values = []
            for i in range(0, length, 2):
                joint_values.append(struct.unpack('<h', data[i:i+2])[0] / 1000.0)

            msg = Float64MultiArray()
            msg.data = joint_values
            self.publisher_.publish(msg)

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [f'joint_{i+1}' for i in range(len(joint_values))]
            js.position = joint_values
            self.feedback_pub.publish(js)

            if hasattr(self, 'last_sent_time'):
                now = self.get_clock().now()
                latency = (now.nanoseconds - self.last_sent_time.nanoseconds) / 1e6
                latency_msg = Float32()
                latency_msg.data = latency
                self.latency_pub.publish(latency_msg)

            self.get_logger().info(f'<< RX joints: {joint_values}')

    def trajectory_callback(self, msg: JointTrajectoryControllerState):
        if not msg.actual.positions:
            return

        joint_positions = list(msg.actual.positions[:5])

        if not hasattr(self, 'last_sent') or self._position_changed(joint_positions):
            float_msg = Float64MultiArray()
            float_msg.data = joint_positions
            self.send_to_stm32(float_msg)
            self.last_sent = joint_positions

    def _position_changed(self, new_pos, threshold=0.01):
        if not hasattr(self, 'last_sent'):
            return True
        for a, b in zip(self.last_sent, new_pos):
            if abs(a - b) > threshold:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = STM32UARTNode(simulate=True)  # use simulate=False when ready
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        if not node.simulate:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()












# PREV WORKING CODE WITHOUT TESTING INTERFACE

# import rclpy
# from rclpy.node import Node
# import serial
# from std_msgs.msg import Float64MultiArray
# import struct
# from control_msgs.msg import JointTrajectoryControllerState  # ✅ Import MoveIt controller state

# STX = 0x02
# ETX = 0x03

# class STM32UARTNode(Node):
#     def __init__(self, simulate=False):
#         super().__init__('stm32_uart_node')
#         self.simulate = simulate

#         if not self.simulate:
#             self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
#         else:
#             self.get_logger().info('Simulation mode enabled — not using serial port')

#         self.publisher_ = self.create_publisher(Float64MultiArray, '/stm32_rx_joints', 10)
        
#         # ✅ Still subscribe to raw manual input (optional)
#         self.subscription = self.create_subscription(
#             Float64MultiArray,
#             '/stm32_tx_joints',
#             self.send_to_stm32,
#             10
#         )

#         # ✅ NEW: Subscribe to MoveIt controller state topic
#         self.trajectory_state_sub = self.create_subscription(
#             JointTrajectoryControllerState,
#             '/arm_group_controller/state',  # Adjusted topic name
#             self.trajectory_callback,
#             10
#         )

#         if not self.simulate:
#             self.timer = self.create_timer(0.05, self.read_from_stm32)

#     def compute_checksum(self, data: bytes):
#         return sum(data) & 0xFF

#     def send_to_stm32(self, msg: Float64MultiArray):
#         data = b''

#         for value in msg.data:
#             data += struct.pack('<h', int(value * 1000))

#         payload = bytes([0x01, 0x10, len(data)]) + data
#         checksum = self.compute_checksum(payload)

#         message = bytes([STX]) + payload + bytes([checksum, ETX])

#         if self.simulate:   # Output to console
#             self.get_logger().info(f'Simulated  >> TX (no serial): {message.hex()}') 
#         else:    # Writing to STM32 
#             self.serial_port.write(message)      
#             self.get_logger().info(f'>> TX: {message.hex()}')       

#     def read_from_stm32(self):
#         if self.serial_port.in_waiting:
#             raw = self.serial_port.read_until(expected=bytes([ETX]))
#             if len(raw) < 6 or raw[0] != STX or raw[-1] != ETX:
#                 return  # invalid message

#             payload = raw[1:-2]
#             checksum = raw[-2]

#             if self.compute_checksum(payload) != checksum:
#                 self.get_logger().warn('Checksum mismatch')
#                 return

#             id_, cmd, length = payload[0], payload[1], payload[2]
#             data = payload[3:]

#             if len(data) != length:
#                 self.get_logger().warn('Length mismatch')
#                 return

#             joint_values = []
#             for i in range(0, length, 2):
#                 joint_values.append(struct.unpack('<h', data[i:i+2])[0] / 1000.0)

#             msg = Float64MultiArray()
#             msg.data = joint_values
#             self.publisher_.publish(msg)

#             self.get_logger().info(f'<< RX joints: {joint_values}')

#     # ✅ NEW: Callback for MoveIt joint state
#     def trajectory_callback(self, msg: JointTrajectoryControllerState):
#         if not msg.actual.positions:
#             return

#         joint_positions = list(msg.actual.positions[:5])  # Trim to 5 joints

#         # Only send if significantly different from last sent data
#         if not hasattr(self, 'last_sent') or self._position_changed(joint_positions):
#             float_msg = Float64MultiArray()
#             float_msg.data = joint_positions
#             self.send_to_stm32(float_msg)
#             self.last_sent = joint_positions

#     def _position_changed(self, new_pos, threshold=0.01):
#         # Check if new positions differ from last sent
#         if not hasattr(self, 'last_sent'):
#             return True
#         for a, b in zip(self.last_sent, new_pos):
#             if abs(a - b) > threshold:
#                 return True
#         return False


# def main(args=None):
#     rclpy.init(args=args)
#     node = STM32UARTNode(simulate=True)  # use simulate=False when ready
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Shutting down')
#     finally:
#         if not node.simulate:
#             node.serial_port.close()
#         node.destroy_node()
#         rclpy.shutdown()
