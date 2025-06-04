
This project is built using several tutorials.

1. Understanding & Creating URDF, visualizing in RVIZ along with launch file build - https://www.youtube.com/watch?v=t67JaKiZY_U&list=PLO89phzZmnHi5GCama8rS0kg3jcEXTq7I&index=19

2. Basic URDF to Xacro additions - https://www.youtube.com/watch?v=BcjHyhV0kIs&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=2

3. Moveit Setup assist - https://www.youtube.com/watch?v=T7-E3bh5LMU
   
As the project progesses, other tutorials and help will be added here...





QUICK CODES:

1. Launch (For launching Gazebo):
[Remember to switch xacro files to gazebo_ros2_control/GazeboSystem]


ros2 launch <package_name> <launch_file_name.py>

ros2 launch manipulator launch_sim.launch.py


2. Launch (For launching Moveit):
[Remember to switch xacro files to fake_components/GenericSystem]

ros2 launch <package_name> <launch_file_name.py>

ros2 launch moveit_config demo.launch.py  


3. Launch - For RPi to STM32 Uart coms
ros2 launch stm32_interface_pkg uart.launch.py 

(change simulation mode true or false from launch file)


2. Sample trajectory command to move robot in gazebo:
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'], points: [{positions: [0.5, 0.5, 0.5, 0.5, 0.5], time_from_start: {sec: 2}}]}}"





