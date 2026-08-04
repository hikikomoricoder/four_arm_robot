#!/usr/bin/env python3
# Copyright 2026 four_arm_robot

"""Event-driven launch for the full robot simulation (Gazebo + ros2_control + MoveIt).

Replaces the fixed-delay `<timer period="15.5">` approach of
`my_robot_gazebo.launch.xml` with a fully event-driven startup:

Dependency chain (controller_manager lives INSIDE the Gazebo server, hosted by
the gz_ros2_control::GazeboSimROS2ControlPlugin from the URDF):
    gz sim server up -> world_room.sdf loaded -> robot spawned (create)
    -> plugin initializes -> controller_manager service becomes available

How this launch removes the fixed delay:
  1. `ros_gz_sim create` itself blocks until the world's
     /world/<world>/create service exists (event-driven robot spawn).
  2. The controller spawners are chained to the EXIT of the `create` process
     (OnProcessExit), so they only start after the robot has been inserted.
  3. Each spawner additionally blocks with `--controller-manager-timeout 120`
     until the plugin's controller_manager service is actually reachable, then
     loads/configures/activates its controller. A genuinely broken startup now
     fails loudly after 120 s instead of hanging forever (or, with the old
     timer, failing when the 15.5 s guess was too short).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# Upper bound (s) each spawner waits for the controller_manager service before
# giving up with a clear error.
CONTROLLER_MANAGER_TIMEOUT = 120


def _spawner_node(controller_name: str, inactive: bool = False) -> Node:
    """controller_manager spawner for one controller, waiting for readiness."""
    args = [
        controller_name,
        "--controller-manager-timeout",
        str(CONTROLLER_MANAGER_TIMEOUT),
    ]
    if inactive:
        args.append("--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen",
    )


def generate_launch_description():
    # ---------------- arguments ----------------
    log_level = LaunchConfiguration("log_level")
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="warn",
        description="ROS log level for verbose nodes",
    )

    # ---------------- static paths ----------------
    bringup_share = get_package_share_directory("my_robot_bringup")
    description_share = get_package_share_directory("my_robot_description")

    urdf_path = os.path.join(
        description_share, "urdf", "four_arm_robot.xml.xacro"
    )
    rviz_config_path = os.path.join(
        description_share, "rviz", "urdf_config.rviz"
    )
    gazebo_bridge_config = os.path.join(
        bringup_share, "config", "gazebo_bridge.yaml"
    )
    left_camera_config = os.path.join(
        bringup_share, "config", "left_camera.yaml"
    )
    right_camera_config = os.path.join(
        bringup_share, "config", "right_camera.yaml"
    )
    world_path = os.path.join(bringup_share, "worlds", "world_room.sdf")

    # ---------------- robot description ----------------
    robot_description = Command(["xacro ", urdf_path])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
        ],
    )

    # ---------------- Gazebo server + robot spawn ----------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": world_path + " -r"}.items(),
    )

    # Blocks until the world's create service exists, then spawns the robot
    # from the /robot_description topic. Exits once the spawn request returns,
    # which is the trigger for the controller spawners below.
    create_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"],
    )

    # ---------------- controllers (event-driven, no fixed delay) ----------------
    # Start only after the robot has been spawned; each spawner then waits for
    # the plugin-hosted controller_manager before loading/activating.
    controller_spawners = [
        _spawner_node("joint_state_broadcaster"),
        _spawner_node("veer_controller"),
        _spawner_node("wheel_controller"),
        _spawner_node("arm_1_controller", inactive=True),
        _spawner_node("arm_2_controller", inactive=True),
        _spawner_node("arm_3_controller", inactive=True),
        _spawner_node("arm_4_controller", inactive=True),
        _spawner_node("gripper_controller"),
        _spawner_node("all_arms_controller"),
    ]
    spawn_controllers_after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=create_robot, on_exit=controller_spawners
        )
    )

    # ---------------- state management ----------------
    # Provides the /group_state_manager service (get_all / get_group / set_group)
    # for robot joint group state management.
    state_manager = Node(
        package="robot_state_manager",
        executable="state_manager",
    )

    # ---------------- bridges / visualization ----------------
    parameter_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": gazebo_bridge_config}],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": True}],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_moveit_config"),
                "launch",
                "move_group_gazebo.launch.py",
            )
        )
    )

    # ---------------- perception (panorama + stereo) ----------------
    display_four_camera = Node(
        package="panorama_camera",
        executable="display_four_camera",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{"use_sim_time": True}],
    )

    # Gazebo bridges camera_info with Tx=0 (monocular); these corrector nodes
    # relay the messages and replace the calibration matrices with correct
    # stereo values (right camera Tx = -fx * baseline) so stereo_image_proc
    # computes metric disparity.
    left_corrector = Node(
        package="stereo_camera",
        executable="camera_info_corrector",
        name="camera_info_corrector_left",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "calibration_file": left_camera_config,
                "input_topic": "/_gz/camera_5/camera_info",
                "output_topic": "/camera_5/camera_info",
            }
        ],
    )
    right_corrector = Node(
        package="stereo_camera",
        executable="camera_info_corrector",
        name="camera_info_corrector_right",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "calibration_file": right_camera_config,
                "input_topic": "/_gz/camera_6/camera_info",
                "output_topic": "/camera_6/camera_info",
            }
        ],
    )

    disparity = Node(
        package="stereo_image_proc",
        executable="disparity_node",
        name="disparity",
        output="screen",
        parameters=[{"use_sim_time": True, "queue_size": 10}],
        remappings=[
            ("left/image_rect", "/camera_5/image_raw"),
            ("right/image_rect", "/camera_6/image_raw"),
            ("left/camera_info", "/camera_5/camera_info"),
            ("right/camera_info", "/camera_6/camera_info"),
        ],
    )

    stereo_processor = Node(
        package="stereo_camera",
        executable="stereo_camera_processor",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            declare_log_level,
            # core robot / simulation
            robot_state_publisher,
            gz_sim,
            create_robot,
            spawn_controllers_after_robot,
            state_manager,
            # bridges / visualization
            parameter_bridge,
            rviz2,
            move_group,
            # perception
            display_four_camera,
            left_corrector,
            right_corrector,
            disparity,
            stereo_processor,
        ]
    )
