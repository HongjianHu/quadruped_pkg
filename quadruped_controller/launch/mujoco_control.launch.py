import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations["pkg_description"]

    description_pkg_path = get_package_share_directory(package_description)
    xacro_file = os.path.join(description_pkg_path, "xacro", "robot.xacro")

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            "GAZEBO": "false",
            "CLASSIC": "false",
            "DEBUG": "false",
            "MUJOCO_VIEWER": context.launch_configurations["use_embedded_mujoco_viewer"],
        },
    ).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            "robot_control.yaml",
        ]
    )

    rviz_config = PathJoinSubstitution(
    [
        FindPackageShare(package_description),
        "config",
        "visualize_urdf.rviz",
    ]
    )

    mujoco_model = os.path.join(
        description_pkg_path,
        "mujoco",
        "go2_unitree",
        "scene.xml",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "publish_frequency": 50.0,
                "use_tf_static": True,
                "ignore_timestamp": True,
            }
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_controllers,
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    quadruped_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "quadruped_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    mujoco_viewer = ExecuteProcess(
        cmd=[
        "/opt/mujoco/bin/simulate",
        mujoco_model,
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_mujoco_viewer")),
    )

    return [
        robot_state_publisher,
        controller_manager,
        rviz,
        mujoco_viewer,
        joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=[quadruped_controller],
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        "pkg_description",
        default_value="go2_description",
        description="Robot description package.",
    )
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz.",
    )

    use_mujoco_viewer = DeclareLaunchArgument(
        "use_mujoco_viewer",
        default_value="false",
        description="Start standalone MuJoCo simulate viewer for model inspection.",
    )

    use_embedded_mujoco_viewer = DeclareLaunchArgument(
        "use_embedded_mujoco_viewer",
        default_value="true",
        description="Start the MuJoCo viewer attached to the ros2_control hardware simulation.",
    )

    return LaunchDescription(
        [
            pkg_description,
            use_rviz,
            use_mujoco_viewer,
            use_embedded_mujoco_viewer,
            OpaqueFunction(function=launch_setup),
        ]
    )
