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




# BELOW CODE TO READ VALUE FROM REPLICA CONTROLLER AND PUBLISH THE JOINT ANGLES

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import serial

# class MimicPublisher(Node):
#     def __init__(self):   
#         super().__init__('mimic_publisher')
#         self.pub = self.create_publisher(JointState, '/mimic_joint_states', 10)
#         self.joint_names = [f'joint_{i}' for i in range(1,6)]

#         # === Open serial port (adjust as needed) ===
#         try:
#             self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
#             self.get_logger().info("Serial port opened successfully")
#         except serial.SerialException as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
#             self.serial = None

#         self.timer = self.create_timer(0.05, self.publish_joint_states)

#     def publish_joint_states(self):
#         if not self.serial:
#             return

#         try:
#             line = self.serial.readline().decode().strip()
#             if not line:
#                 return

#             angles = [float(x) for x in line.split(',')]
#             if len(angles) != 5:
#                 self.get_logger().warn(f"Invalid joint data: {line}")
#                 return

#             msg = JointState()
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.name = self.joint_names
#             msg.position = angles
#             self.pub.publish(msg)

#         except Exception as e:
#             self.get_logger().warn(f"Error parsing serial data: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = MimicPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
