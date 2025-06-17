import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Float32
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import time

class TestManagerNode(Node):
    def __init__(self):
        super().__init__('test_manager_node')

        # Publisher to toggle test mode
        self.test_mode_pub = self.create_publisher(Bool, '/test_mode_on', 10)

        # Publisher to mimic input topic
        self.joint_pub = self.create_publisher(JointState, '/joint_test_input', 10)

        # Latency tracking subscribers
        self.input_moveit_sub = self.create_subscription(Float32, '/input_moveit_time', self.input_moveit_callback, 10)
        self.moveit_stm32_sub = self.create_subscription(Float32, '/moveit_stm32_time', self.moveit_stm32_callback, 10)
        self.stm32_output_sub = self.create_subscription(Float32, '/stm32_output_time', self.stm32_output_callback, 10)

        self.latest_sent_time = None
        self.waiting_for_latency = False
        self.latency_received = {
            'input_moveit': False,
            'moveit_stm32': False,
            'stm32_output': False
        }

        self.joint_positions = [
            [0.2, 0.1, -0.1, 0.3, 0.0],
            [0.4, 0.3, -0.2, 0.5, 0.1],
            [0.6, 0.2, -0.3, 0.7, 0.2],
            [1.0, 0.5, -0.5, 1.2, 0.3]
        ]
        self.current_index = 0

        self.toggle_test_mode(True)
        self.send_next_test_joint()

    def toggle_test_mode(self, on=True):
        msg = Bool()
        msg.data = on
        self.test_mode_pub.publish(msg)
        self.get_logger().info(f'Test mode set to: {on}')

    def send_next_test_joint(self):
        if self.current_index >= len(self.joint_positions):
            self.get_logger().info('All test positions sent. Shutting down test. ✅')
            self.toggle_test_mode(False)
            return

        now = self.get_clock().now()
        self.latest_sent_time = now
        self.waiting_for_latency = True
        self.latency_received = {k: False for k in self.latency_received}

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [f'joint_{i+1}' for i in range(5)]
        msg.position = self.joint_positions[self.current_index]

        self.joint_pub.publish(msg)
        self.get_logger().info(f'Published test joint position {self.current_index + 1}')

    def check_all_latencies_received(self):
        if all(self.latency_received.values()):
            self.get_logger().info(f'✅ All latency metrics received for position {self.current_index + 1}\n')
            self.current_index += 1
            self.waiting_for_latency = False
            self.send_next_test_joint()

    def input_moveit_callback(self, msg: Float32):
        if self.waiting_for_latency:
            self.latency_received['input_moveit'] = True
            self.get_logger().info(f'[LATENCY] Input → MoveIt: {msg.data:.3f} ms')
            self.check_all_latencies_received()

    def moveit_stm32_callback(self, msg: Float32):
        if self.waiting_for_latency:
            self.latency_received['moveit_stm32'] = True
            self.get_logger().info(f'[LATENCY] MoveIt → STM32 Send: {msg.data:.3f} ms')
            self.check_all_latencies_received()

    def stm32_output_callback(self, msg: Float32):
        if self.waiting_for_latency:
            self.latency_received['stm32_output'] = True
            self.get_logger().info(f'[LATENCY] STM32 → Feedback: {msg.data:.3f} ms')
            self.check_all_latencies_received()


def main(args=None):
    rclpy.init(args=args)
    node = TestManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.toggle_test_mode(False)
        node.get_logger().info('Shutting down test manager')
    finally:
        node.destroy_node()
        rclpy.shutdown()
