import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.time import Time

class MimicActionClient(Node):
    def __init__(self):
        super().__init__('mimic_action_client')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory'
        )
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        self.subscription = self.create_subscription(
            JointState,
            '/mimic_joint_states',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Compute latency
        received_time = self.get_clock().now().nanoseconds
        sent_time = Time.from_msg(msg.header.stamp).nanoseconds
        latency_ms = (received_time - sent_time) / 1e6
        self.get_logger().info(f'Latency: {latency_ms:.2f} ms')

        # Check if action server is ready
        if not self._action_client.server_is_ready():
            self.get_logger().warn('Action server not ready')
            return

        # Prepare trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.time_from_start.sec = 1  # delay from start of trajectory
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        # Send to MoveIt
        self.get_logger().info(f'Sending goal: {point.positions}')
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MimicActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()









# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import JointState

# class MimicActionClient(Node):
#     def __init__(self):
#         super().__init__('mimic_action_client')
#         self._action_client = ActionClient(self, FollowJointTrajectory, '/arm_group_controller/follow_joint_trajectory')
#         self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
#         self.subscription = self.create_subscription(JointState, '/mimic_joint_states', self.listener_callback, 10)

#     def listener_callback(self, msg):
#         if not self._action_client.server_is_ready():
#             self.get_logger().warn('Action server not ready')
#             return

#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names
#         point = JointTrajectoryPoint()
#         point.positions = msg.position
#         point.time_from_start.sec = 1
#         traj.points.append(point)

#         goal_msg = FollowJointTrajectory.Goal()
#         goal_msg.trajectory = traj

#         self.get_logger().info(f'Sending goal: {point.positions}')
#         self._action_client.send_goal_async(goal_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = MimicActionClient()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()