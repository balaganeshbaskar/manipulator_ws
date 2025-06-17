import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import time

class TestManagerNode(Node):
    def __init__(self):
        super().__init__('test_manager_node')

        self.publisher = self.create_publisher(Float64MultiArray, '/stm32_tx_joints', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/stm32_rx_joints',
            self.receive_callback,
            10
        )

        self.declare_parameter('testing_mode', True)
        self.testing_mode = self.get_parameter('testing_mode').value
        self.get_logger().info(f"Testing mode: {'ON' if self.testing_mode else 'OFF'}")

        self.test_index = 0
        self.joint_goals = [
            [0.1, 0.2, 0.3, 0.4, 0.5],
            [0.2, 0.1, -0.1, 0.0, 0.3],
            [0.3, 0.3, 0.3, 0.3, 0.3]
        ]

        self.send_time = None
        self.timer = self.create_timer(2.0, self.send_next_command) if self.testing_mode else None

    def send_next_command(self):
        if self.test_index >= len(self.joint_goals):
            self.get_logger().info('All test commands sent. Exiting...')
            rclpy.shutdown()
            return

        msg = Float64MultiArray()
        msg.data = self.joint_goals[self.test_index]

        self.send_time = self.get_clock().now()
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent test command #{self.test_index + 1}: {msg.data}')

        self.test_index += 1

    def receive_callback(self, msg):
        now = self.get_clock().now()
        if self.send_time:
            latency_ms = (now.nanoseconds - self.send_time.nanoseconds) / 1e6
            self.get_logger().info(f'Received feedback. Round-trip latency: {latency_ms:.2f} ms')
        self.get_logger().info(f'Received joint data: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TestManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test Manager interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
