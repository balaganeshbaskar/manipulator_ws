#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class MimicPublisher(Node):
    def __init__(self):
        super().__init__('mimic_publisher')
        self.pub = self.create_publisher(JointState, '/mimic_joint_states', 10)
        self.joint_names = [f'joint_{i}' for i in range(1,6)]
        self.step = 0
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            math.sin(self.step/10.0),
            math.sin(self.step/12.0),
            math.sin(self.step/15.0),
            math.sin(self.step/8.0),
            math.sin(self.step/18.0)
        ]
        self.pub.publish(msg)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = MimicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
