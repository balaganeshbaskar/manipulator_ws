#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random

class MimicRandomOnce(Node):
    def __init__(self):
        super().__init__('mimic_random_once')
        self.joint_names = [f'joint_{i}' for i in range(1, 6)]
        self.pub = self.create_publisher(JointState, '/mimic_joint_states', 10)

        self.retry_count = 0
        self.max_retries = 20  # e.g., 20 * 0.5s = 10 seconds
        self.retry_timer = self.create_timer(0.5, self.check_and_send)

    def check_and_send(self):
        if self.pub.get_subscription_count() > 0:
            self.send_once()
        elif self.retry_count >= self.max_retries:
            self.get_logger().warn('No subscribers after timeout. Giving up.')
            rclpy.shutdown()
        else:
            self.get_logger().info(f'Waiting for subscriber... ({self.retry_count + 1}/{self.max_retries})')
            self.retry_count += 1

    def send_once(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [random.uniform(-1.0, 1.0) for _ in self.joint_names]

        self.get_logger().info(f'Sending random joint positions: {msg.position}')
        self.pub.publish(msg)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MimicRandomOnce()
    rclpy.spin(node)
    node.destroy_node()








# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import math

# class MimicPublisher(Node):
#     def __init__(self):
#         super().__init__('mimic_publisher')
#         self.joint_names = [f'joint_{i}' for i in range(1, 6)]
#         self.step = 0

#         # Publisher for /mimic_joint_states
#         self.pub = self.create_publisher(JointState, '/mimic_joint_states', 10)

#         # Subscriber to external joint test inputs - INPUT CONTROLLER SHOULD PUBLISH TO THIS
#         self.sub = self.create_subscription(JointState, '/joint_test_input', self.test_input_callback, 10)

#         # Timer for generating sine-based motion
#         self.timer = self.create_timer(0.1, self.publish_sine_wave)

#     def publish_sine_wave(self):
#         msg = JointState()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.name = self.joint_names
#         msg.position = [
#             math.sin(self.step / 10.0),
#             math.sin(self.step / 12.0),
#             math.sin(self.step / 15.0),
#             math.sin(self.step / 8.0),
#             math.sin(self.step / 18.0)
#         ]
#         self.pub.publish(msg)
#         self.step += 1

#     def test_input_callback(self, msg: JointState):
#         # Re-publish with updated timestamp for latency tracking
#         msg.header.stamp = self.get_clock().now().to_msg()
#         self.pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MimicPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("MimicPublisher shutting down...")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()