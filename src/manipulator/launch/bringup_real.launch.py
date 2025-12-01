import yaml
import os # Needed for path joining manually if FindPackageShare returns path


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = FindPackageShare(package_name).find(package_name)
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

    # 2. ROBOT DESCRIPTION (URDF)
    # We use xacro to process the file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("manipulator"), "urdf", "model.urdf.xacro"]
            ),
        ]
    )
    
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Load SRDF
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



    # 3. CONTROLLER MANAGER (ROS2 CONTROL)
    # This node manages the hardware interface and controllers
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([FindPackageShare("moveit_config"), "config", "ros2_controllers.yaml"]),
        ],
        output="screen",
    )

    # 4. ROBOT STATE PUBLISHER
    # Publishes TF transforms based on URDF and joint states
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 5. HARDWARE BRIDGE (YOUR NEW NODE)
    # Connects ROS2 Control TopicBasedSystem <-> Serial Port
    hardware_bridge_node = Node(
        package="manipulator_hardware",
        executable="serial_bridge",
        name="serial_bridge",
        parameters=[{"simulate": simulate}],
        output="screen",
    )

    # 6. CONTROLLER SPAWNERS
    # These tell the manager to start specific controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 7. MOVEIT (TRAJECTORY PLANNING)
    # We include the existing MoveIt launch logic
    # Note: We delay this slightly to ensure controllers are ready
    # Load configs
    kinematics_config = load_yaml("moveit_config", "config/kinematics.yaml")
    joint_limits_config = load_yaml("moveit_config", "config/joint_limits.yaml")
    # moveit_controllers_config = load_yaml("moveit_config", "config/moveit_controllers.yaml")

    # --- MANUALLY DEFINE CONTROLLERS (Bypass load_yaml) ---
    moveit_controllers_config = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": True,
        "controller_names": ["arm_controller"],
        "arm_controller": {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"],
        },
    }
    # -------------------------------------------------------


    moveit_launch = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_config},
            {"robot_description_planning": joint_limits_config},
            
            # Load the hardcoded config
            moveit_controllers_config,
            
            {"planning_pipelines": ["ompl"]},
        ],
    )


    # 8. RVIZ (VISUALIZATION)
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
            {"robot_description_kinematics": kinematics_config}, # Dictionary!
        ],
    )


    # 9. COORDINATION (DELAY LOGIC)
    # Ensure controllers start only after the control_node is ready
    # Ensure MoveIt starts only after controllers are active
    
    return LaunchDescription([
        simulate_arg,
        rsp_node,
        control_node,
        hardware_bridge_node,
        
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
        
        # Launch MoveIt & RViz once the arm controller is active
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[moveit_launch, rviz_node],
            )
        ),
    ])

