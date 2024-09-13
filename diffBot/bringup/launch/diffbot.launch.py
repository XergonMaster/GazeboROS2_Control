import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Define constants for better readability
PACKAGE_NAME = "diff_bot"
ROBOT_DESCRIPTION_PARAM = "robot_description"
RVIZ_CONFIG_SUBDIR = "rviz"
URDF_SUBDIR = "urdf"
CONTROLLERS_CONFIG_SUBDIR = "config"
RVIZ_FILE = "diffbot.rviz"
URDF_FILE = "diffbot.urdf.xacro"
CONTROLLERS_FILE = "diffbot_controllers.yaml"
GAZEBO_LAUNCH_FILE = "gazebo.launch.py"

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 automatically with this launch file."),
        DeclareLaunchArgument("use_mock_hardware", default_value="true", description="Start robot with mock hardware mirroring command to its states."),
        DeclareLaunchArgument("spawn_gazebo", default_value="false", description="Whether to spawn the robot in Gazebo."),
    ]

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    spawn_gazebo = LaunchConfiguration("spawn_gazebo")

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), URDF_SUBDIR, URDF_FILE]),
        " ",
        "use_mock_hardware:=", use_mock_hardware,
    ])
    robot_description = {ROBOT_DESCRIPTION_PARAM: robot_description_content}

    robot_controllers = PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), CONTROLLERS_CONFIG_SUBDIR, CONTROLLERS_FILE])

    rviz_config_file = PathJoinSubstitution([FindPackageShare("robots_description"), "diff", RVIZ_CONFIG_SUBDIR, RVIZ_FILE])

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/" + ROBOT_DESCRIPTION_PARAM, "/" + ROBOT_DESCRIPTION_PARAM),
            ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("gazebo_ros").find("gazebo_ros"), "launch", GAZEBO_LAUNCH_FILE)
        ),
        condition=IfCondition(spawn_gazebo),
    )

    spawn_entity = Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-topic", "/" + ROBOT_DESCRIPTION_PARAM, "-entity", "diffbot"],
        output="screen",
        condition=IfCondition(spawn_gazebo),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        gazebo,
        spawn_entity,
    ]

    return LaunchDescription(declared_arguments + nodes)
