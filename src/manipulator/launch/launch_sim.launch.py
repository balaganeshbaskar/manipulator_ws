import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():

    package_name = 'manipulator'

    pkg_path = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_path, 'urdf', 'model.urdf.xacro')
    yaml_path = os.path.join(pkg_path, 'config', 'ros2_controllers.yaml')

    # Generate robot_description by running xacro
    robot_description_content = Command(['xacro ', urdf_path])

    # Launch robot_state_publisher to publish /robot_description (with use_sim_time)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch Gazebo simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn robot in Gazebo using the /robot_description topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Launch ros2_control node with only the controllers YAML (robot_description is published by rsp)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=["/home/bgb_6342/manipulator_ws/src/manipulator/config/ros2_controllers.yaml"],
        remappings=[('/robot_description', '/robot_description')],
        output="screen"
    )

    # Spawn controller: joint_state_broadcaster (after a delay to let ros2_control_node startup)
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Spawn controller: joint_trajectory_controller (after joint_state_broadcaster is up)
    joint_trajectory_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        rsp,  # robot_state_publisher
        gazebo,
        spawn_entity,
        control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner
    ])
