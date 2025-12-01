import os
import launch
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.actions
from launch_ros.descriptions import ParameterValue  # 1. Import this

def generate_launch_description():
    pkg_path = launch_ros.substitutions.FindPackageShare(package='manipulator').find('manipulator')
    urdf_path = os.path.join(pkg_path, 'urdf/model.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path, 'config/config.rviz')

    robot_description_content = Command(['xacro ', urdf_path])

    # 2. Wrap the content in ParameterValue
    params = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': True
    }

    gui = LaunchConfiguration('gui', default='true')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(gui)
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])





# import os
# import launch
# from launch.substitutions import LaunchConfiguration, Command
# import launch_ros.actions

# def generate_launch_description():
#     pkg_path = launch_ros.substitutions.FindPackageShare(package='manipulator').find('manipulator')
#     urdf_path = os.path.join(pkg_path, 'urdf/model.urdf.xacro')
#     rviz_config_path = os.path.join(pkg_path, 'config/config.rviz')

#     robot_description_content = Command(['xacro ', urdf_path])

#     params = {
#         'robot_description': robot_description_content,
#         'use_sim_time': True
#     }

#     gui = LaunchConfiguration('gui', default='true')

#     robot_state_publisher_node = launch_ros.actions.Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[params]
#     )

#     joint_state_publisher_gui_node = launch_ros.actions.Node(
#         package='joint_state_publisher_gui',
#         executable='joint_state_publisher_gui',
#         name='joint_state_publisher_gui',
#         condition=launch.conditions.IfCondition(gui)
#     )

#     rviz_node = launch_ros.actions.Node(
#         package='rviz2',
#         executable='rviz2',
#         output='screen',
#         arguments=['-d', rviz_config_path],
#         parameters=[{'use_sim_time': True}]
#     )

#     return launch.LaunchDescription([
#         launch.actions.DeclareLaunchArgument(
#             'gui',
#             default_value='true',
#             description='Flag to enable joint_state_publisher_gui'
#         ),
#         robot_state_publisher_node,
#         # joint_state_publisher_gui_node,
#         # rviz_node
#     ])