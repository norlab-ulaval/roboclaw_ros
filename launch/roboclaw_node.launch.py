import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()
    config_folder = os.path.join(
        get_package_share_directory("roboclaw_node_ros"),
        "config",
    )

    print(os.path.join(config_folder, "roboclaw.yaml"))

    # Roboclaw node
    roboclaw_node = Node(
        package="roboclaw_node_ros",
        executable="roboclaw_node",
        name="roboclaw_node",
        parameters=[
            os.path.join(config_folder, "roboclaw.yaml"),
        ],
        remappings=[
            ("/cmd_vel", "/twist_mux/cmd_vel"),
        ],
    )
    ld.add_action(roboclaw_node)

    return ld
