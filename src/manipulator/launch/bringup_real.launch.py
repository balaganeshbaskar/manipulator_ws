import os
import launch.conditions  # ADD THIS LINE
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction  # ADD OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

import yaml
from ament_index_python.packages import get_package_share_directory


# ADD THIS ENTIRE FUNCTION
def validate_configuration(context, *args, **kwargs):
    """Validate that simulate and use_custom_executor are not both true"""
    simulate = context.launch_configurations.get('simulate', 'false')
    use_custom = context.launch_configurations.get('use_custom_executor', 'false')
    
    if simulate == 'true' and use_custom == 'true':
        raise RuntimeError(
            "\n❌ INVALID CONFIGURATION ❌\n"
            "Cannot use custom executor with simulation mode.\n"
        )
    return []

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # 1. ARGUMENTS
    simulate_arg = DeclareLaunchArgument(
        "simulate", default_value="false", description="Run in simulation mode (mock hardware)"
    )
    simulate = LaunchConfiguration("simulate")

    use_custom_executor_arg = DeclareLaunchArgument(
        "use_custom_executor", default_value="false", description="Use custom Teensy executor"
    )
    use_custom_executor = LaunchConfiguration("use_custom_executor")

    # 2. ROBOT DESCRIPTION
    # Load the URDF file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("manipulator"), "urdf", "model.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Load the SRDF file (Semantic Description)
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("moveit_config"), "config", "five_dof_robot_manipulator.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # 3. LOAD CONFIG FILES
    # We use PathJoinSubstitution so ROS resolves these at runtime (Standard Practice)
    kinematics_yaml = PathJoinSubstitution(
        [FindPackageShare("moveit_config"), "config", "kinematics.yaml"]
    )

    # Load the actual dictionary content
    joint_limits_full = load_yaml("moveit_config", "config/joint_limits.yaml")

    # Extract the robot_description_planning section
    # Handle both possible structures in your YAML
    if joint_limits_full and "/**:" in joint_limits_full:
        # If YAML has /** wildcard structure
        wildcard_content = joint_limits_full["/**:"]
        if "ros__parameters" in wildcard_content:
            joint_limits_dict = wildcard_content["ros__parameters"].get("robot_description_planning", {})
        elif "robot_description_planning" in wildcard_content:
            joint_limits_dict = wildcard_content["robot_description_planning"]
        else:
            joint_limits_dict = {}
    else:
        # Fallback
        joint_limits_dict = {}
    
    print(f"✅ Loaded joint limits: {joint_limits_dict}")  # Debug output

    # joint_limits_yaml = PathJoinSubstitution(
    #     [FindPackageShare("moveit_config"), "config", "joint_limits.yaml"]
    # )
    moveit_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("moveit_config"), "config", "moveit_controllers.yaml"]
    )
    ompl_planning_yaml = PathJoinSubstitution(
        [FindPackageShare("moveit_config"), "config", "ompl_planning.yaml"]
    )
    chomp_planning_yaml = PathJoinSubstitution(
        [FindPackageShare("moveit_config"), "config", "chomp_planning.yaml"]
    )


    # 4. CONTROLLER MANAGER (ROS2 CONTROL)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([FindPackageShare("moveit_config"), "config", "ros2_controllers.yaml"]),
        ],
        output="screen",
        condition=launch.conditions.UnlessCondition(use_custom_executor)  # ADD THIS LINE
    )

    # 5. ROBOT STATE PUBLISHER
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 6. HARDWARE BRIDGE
    hardware_bridge_node = Node(
        package="manipulator_hardware",
        executable="serial_bridge",
        name="serial_bridge",
        parameters=[{"simulate": simulate}],
        output="screen",
        condition=launch.conditions.UnlessCondition(use_custom_executor)  # ADD THIS LINE
    )

    # ADD THIS ENTIRE SECTION (6B):
    # 6B. CUSTOM TEENSY EXECUTOR
    teensy_executor_node = Node(
        package="manipulator",
        executable="teensy_executor.py",
        name="teensy_executor",
        output="screen",
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'serial_baud': 115200,
            'joint_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'],
            'planning_group': 'arm_group',
            'max_waypoints': 40,
            'min_waypoint_distance': 0.02
        }],
        condition=launch.conditions.IfCondition(use_custom_executor)
    )

    # 7. SPAWNERS
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=launch.conditions.UnlessCondition(use_custom_executor)  # ADD THIS LINE
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        condition=launch.conditions.UnlessCondition(use_custom_executor)  # ADD THIS LINE
    )

    # 8. MOVEIT MOVE GROUP
    # This is the standard way to pass parameters: File paths for configs that support it, 
    # or dictionaries if we loaded them manually. 
    # NOTE: MoveIt node often prefers loaded dictionaries for some params to avoid "value before rosparam" errors.
    # However, since you requested "standard files", we will rely on the node reading the files.
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_controllers_yaml,
            chomp_planning_yaml,
            {"robot_description_kinematics": kinematics_yaml},
            # {"robot_description_planning": joint_limits_yaml},
            {"robot_description_planning": joint_limits_dict},

            # Set CHOMP as the default pipeline
            {"planning_plugin": "chomp_interface/CHOMPPlanner"},
            {"planning_pipelines": ["chomp"]},
            {"default_planning_pipeline": "chomp"},
            {
                "request_adapters": (
                    "default_planner_request_adapters/ResolveConstraintFrames "
                    "default_planner_request_adapters/ValidateWorkspaceBounds "
                    "default_planner_request_adapters/CheckStartStateBounds "
                    "default_planner_request_adapters/AddTimeOptimalParameterization"
                ),
            },
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"moveit_manage_controllers": True},
            # Add trajectory execution monitoring defaults
            {"trajectory_execution.allowed_execution_duration_scaling": 1.2},
            {"trajectory_execution.allowed_goal_duration_margin": 0.5},
            {"trajectory_execution.allowed_start_tolerance": 0.01},
        ],
    )

    # 9. RVIZ
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("moveit_config"), "config", "moveit.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_dict},
            # {"robot_description_planning": joint_limits_yaml},
        ],
    )

    # 10. ORCHESTRATION
    # 10. ORCHESTRATION
    return LaunchDescription([
        simulate_arg,
        use_custom_executor_arg,  # ADD THIS
        
        OpaqueFunction(function=validate_configuration),  # ADD THIS
        
        rsp_node,
        control_node,
        hardware_bridge_node,
        teensy_executor_node,  # ADD THIS
        move_group_node,       # ADD THIS (start directly)
        rviz_node,             # ADD THIS (start directly)
        
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
    ])

