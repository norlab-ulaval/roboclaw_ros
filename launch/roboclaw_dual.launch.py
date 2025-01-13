import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()
    config_folder = os.path.join(
        get_package_share_directory("roboclaw_ros"),
        "config",
    )

    # Roboclaw node
    roboclaw_node = Node(
        package="roboclaw_ros",
        executable="roboclaw_dual",
        name="roboclaw_dual",
        parameters=[
            os.path.join(config_folder, "roboclaw_dual.yaml"),
        ],
        remappings=[
            ("/cmd_vel", "/cmd_vel"),
        ],
    )
    ld.add_action(roboclaw_node)

    return ld
