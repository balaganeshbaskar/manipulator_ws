
This project is built using several tutorials.

1. Understanding & Creating URDF, visualizing in RVIZ along with launch file build - https://www.youtube.com/watch?v=t67JaKiZY_U&list=PLO89phzZmnHi5GCama8rS0kg3jcEXTq7I&index=19

2. Basic URDF to Xacro additions - https://www.youtube.com/watch?v=BcjHyhV0kIs&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=2

3. Moveit Setup assist - https://www.youtube.com/watch?v=T7-E3bh5LMU
   
As the project progesses, other tutorials and help will be added here...





QUICK CODES:

1. Launch:
ros2 launch <package_name> <launch_file_name.py>
ros2 launch manipulator launch_sim.launch.py

2. Sample trajectory command to move robot in gazebo:
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "joint_names:
- 'joint_1'
- 'joint_2'
- 'joint_3'
- 'joint_4'
- 'joint_5'
points:
- positions: [0.5, 0.5, 0.0, -0.5, 1.0]
  time_from_start:
    sec: 2
    nanosec: 0"




