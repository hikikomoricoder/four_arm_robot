"""
move_group launch for Gazebo simulation of the full robot (my_robot).

Key differences from move_group.launch.py (standalone arm):
  - Robot name: "my_robot"  (matches my_robot.urdf.xacro and my_robot.srdf)
  - URDF loaded from my_robot_description (same xacro Gazebo uses)
  - use_sim_time=True so MoveIt clock matches Gazebo sim time
  - No rsp.launch.py included (robot_state_publisher is already running in the Gazebo launch)
"""

import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Build MoveIt config for the full robot.
    # MoveItConfigsBuilder("my_robot", ...) automatically looks for:
    #   arm_moveit_config/config/my_robot.srdf
    #   arm_moveit_config/config/kinematics.yaml
    #   arm_moveit_config/config/moveit_controllers.yaml
    #   etc.
    # We override the URDF to point at the full robot xacro in my_robot_description.
    my_robot_urdf = os.path.join(
        get_package_share_directory("my_robot_description"),
        "urdf", "four_arm_robot.xml.xacro",
    )

    # arm_moveit_config/.setup_assistant hardcodes arm.srdf as the SRDF.
    # We must explicitly override it with my_robot.srdf so the robot name matches
    # the my_robot.urdf.xacro used by the full robot Gazebo launch.
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="robot_moveit_config")
        .robot_description(file_path=my_robot_urdf)
        .robot_description_semantic(file_path="config/my_robot.srdf")
        .to_moveit_configs()
    )

    # Build the parameter dict.
    # publish_robot_description_semantic=True is required: without it move_group
    # does not publish the /robot_description_semantic topic that MoveGroupInterface
    # clients (e.g. test_moveit) subscribe to.
    # use_sim_time=True makes MoveIt use Gazebo sim clock so joint state timestamps match.
    params = moveit_config.to_dict()
    params["publish_robot_description_semantic"] = True
    params["use_sim_time"] = True

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([move_group_node])
