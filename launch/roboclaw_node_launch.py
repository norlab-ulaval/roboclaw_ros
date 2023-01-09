from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboclaw_node_ros',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[
                {'dev': '/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x30A-if00'},
                {'baud': 115200},
                {'address': 128},
                {'max_speed': 2.0},
                #{'~ticks_per_meter': 4342.2},
                {'ticks_per_meter': 3802.4},
                {'ticks_per_rotation': 2780},
                {'base_width': 0.315},
                {'pub_odom': True},
                {'stop_movement': True},
            ],
            remappings=[
                ('/cmd_vel', '/twist_mux/cmd_vel'),
            ]
        )
    ])
