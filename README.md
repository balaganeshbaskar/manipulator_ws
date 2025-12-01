ROBOTIC ARM folder is for the Arduino Nano Joint module code backup


Teensy 4.1 folder is for the teensy 4.1 sub brain code.




REST OF THE FOLDERS FOR ROS2...





This project is built using several tutorials.

1. Understanding & Creating URDF, visualizing in RVIZ along with launch file build - https://www.youtube.com/watch?v=t67JaKiZY_U&list=PLO89phzZmnHi5GCama8rS0kg3jcEXTq7I&index=19

2. Basic URDF to Xacro additions - https://www.youtube.com/watch?v=BcjHyhV0kIs&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=2

3. Moveit Setup assist - https://www.youtube.com/watch?v=T7-E3bh5LMU
   
As the project progesses, other tutorials and help will be added here...





QUICK CODES:



BUILD:




colcon build --symlink-install




source install/setup.bash








-----------------------------------------------------------------------------------------------------------------------------------------------------

LATEST (01-12-25): 
introduced manipulator_hardware, refactored a lot for industrial standard and actual robot usage.
Finally put everything as one and works only for simulation as of now.
Teensy connection not established as of (01-12-25).

1. ros2 launch manipulator bringup_real.launch.py simulate:=true


                                OR 
(NOT TESTED)
LAUCH WITH REAL HARDWARE:

1. ros2 launch manipulator bringup_real.launch.py simulate:=false


{
When Moving to Real Hardware, when you connect your physical robot (Teensy),
The serial bridge will:

Connect to /dev/ttyACM0 (your Teensy)
Send position commands: <J1,J2,J3,J4,J5>
Receive position feedback: <P1,P2,P3,P4,P5>


Tighten tolerances in ros2_controllers.yaml:

joint_1:
  trajectory: 0.05  # Reduce from 1.0 for real hardware
  goal: 0.001       # Already tight

}


-----------------------------------------------------------------------------------------------------------------------------------------------------












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





