# RoboClaw ROS

> RoboClaw node for `melodic` & `humble`

[![License](https://img.shields.io/github/license/norlab-ulaval/roboclaw_ros?style=for-the-badge)](https://opensource.org/licenses/BSD-3-Clause)
[![Made with Python](https://img.shields.io/badge/Made%20with-Python-red?style=for-the-badge&logo=Python)](https://www.python.org)
[![ROS1](https://img.shields.io/badge/ROS1-melodic-blue?labelColor=blue&style=for-the-badge&logo=ROS)](http://wiki.ros.org/melodic)
[![ROS2](https://img.shields.io/badge/ROS2-humble-blue?labelColor=blue&style=for-the-badge&logo=ROS)](https://docs.ros.org/en/humble)

Thanks to Brad Bazemore ([sonyccd](https://github.com/sonyccd)) for developing the original driver.

This is a ROS1/ROS2 driver for the RoboClaw motor controllers made by [Basicmicro Motion Control](https://www.basicmicro.com). It is actively maintained by the [Northern Robotics Laboratory](http://norlab.ulaval.ca/) of Université Laval.

## Dependencies

This ROS driver is using the python driver developed by [Team Chat Robotique](https://gitlab.com/team-chat-robotique/libraries/team-chat-robotique-roboclaw-python) (Thanks!). Install it with pip :

```sh
pip install tcr-roboclaw
```


## Before you begin

For the ROS1 (`melodic`) version : [melodic](https://github.com/norlab-ulaval/roboclaw_ros/tree/melodic)

Before you use this package, you need to calibrate the velocity PID on the RoboClaw. This will require to use BasicMicro's [Motion Studio](https://www.basicmicro.com/downloads) software (only available for Windows).

From the RoboClaw [user manual](https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf):

> Motion Studio provides the option to auto tune velocity and position control. To use the auto tune option, make sure the encoder and motor are running in the desired direction and the basic PWM control of the motor works as expected. It is recommend to ensure the motor and encoder
combination are functioning properly before using the auto tune feature.

> Go to the PWM Settings screen in Motion Studio.
> Slide the motor slider up to start moving the motor forward. Check the encoder is increasing in value. If it is not either reverse the motor wires or the encoder wires. The recheck.
> To start auto tune click the auto tune button for the motor channel that is will be tuned first. The auto tune function will try to determine the best settings for that motor channel.

## Usage instructions

Just clone the repo into your colcon workspace.  Remember to make sure ROS2 has permissions to use the dev port you give it.

```sh
cd <workspace>/src
git clone https://github.com/norlab-ulaval/roboclaw_ros.git
cd <workspace>
colcon build --symlink-install --packages-select roboclaw_ros
. install/local_setup.bash
ros2 launch roboclaw_ros roboclaw_node.launch.py
```

## Parameters

The launch file can be configured at the command line with arguments, by changing the value in the launch file or through the `ros2 param` server.

| Parameter            | Default        | Definition                                                             |
| -------------------- | -------------- | ---------------------------------------------------------------------- |
| `dev`                | `/dev/ttyACM0` | Path to the RoboClaw device path                                       |
| `baud`               | `115200`       | Baud rate the RoboClaw is configured for                               |
| `address`            | `128`          | The address the RoboClaw is set to, 128 is 0x80                        |
| `max_speed_linear`   | `2.0`          | Max linear speed allowed for motors in meters per second               |
| `max_speed_angular`  | `2.0`          | Max angular speed allowed for motors in meters per second              |
| `stop_if_idle`       | `true`         | Stops movement if no velocity commands are received for `idle_timeout` |
| `idle_timeout`       | `1.0`          | Duration, in seconds, after which motors are stopped when idle         |
| `ticks_per_meter`    | `4000`         | The number of encoder ticks per meter of movement                      |
| `ticks_per_rotation` | `2000`         | The number of encoder ticks per rotation of wheel                      |
| `base_width`         | `0.315`        | Width from one wheel center to another, in meters                      |
| `custom_pid`         | `true`         | Whether the driver should overwrite the PID parameters or not          |
| `p_constant`         | `3.0`          | Proportional gain of PID controller (unused if `custom_pid` is false)  |
| `i_constant`         | `0.42`         | Integral gain of PID controller (unused if `custom_pid` is false)      |
| `d_constant`         | `0.0`          | Derivative gain of PID controller (unused if `custom_pid` is false)    |
| `qpps`               | `6000`         | Maximum speed of motor, in ticks/sec (unused if `custom_pid` is false) |
| `odom_rate`          | `30`           | Rate of polling and publishing odometry data                           |
| `elec_rate`          | `10`           | Rate of polling and publishing electrical data                         |
| `publish_odom`       | `true`         | Publishes Odometry if set to true                                      |
| `publish_encoders`   | `true`         | Publishes EncoderState if set to true                                  |
| `publish_elec`       | `true`         | Publishes MotorState if set to true                                    |
| `publish_tf`         | `false`        | Broadcasts TF from `odom` to `base_link` if set to true                |

## Topics

### Subscribed

`/cmd_vel` [(geometry_msgs/Twist)](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Twist.msg)
Velocity commands for the mobile base.

### Published

`/motors/odometry` [(nav_msgs/Odometry)](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg)
Odometry output from the mobile base.

`/motors/{left/right}/encoder` [(norlab_custom_interfaces/EncoderState)](https://github.com/norlab-ulaval/norlab_custom_interfaces/blob/main/msg/EncoderState.msg)
Encoder state for the left and right motors.

`/motors/{left/right}/electrical` [(norlab_custom_interfaces/MotorState)](https://github.com/norlab-ulaval/norlab_custom_interfaces/blob/main/msg/MotorState.msg)
Electrical data for both motors.

### RoboClaw Node graph

```mermaid
flowchart LR

/roboclaw_node[ /roboclaw_node ]:::main
/twist_mux[ /twist_mux ]:::node
/foxglove_bridge[ /foxglove_bridge ]:::node
/twist_mux/cmd_vel([ /twist_mux/cmd_vel<br>geometry_msgs/msg/Twist ]):::topic
/diagnostics([ /diagnostics<br>diagnostic_msgs/msg/DiagnosticArray ]):::topic
/motors/left/electrical([ /motors/left/electrical<br>norlab_custom_interfaces/msg/MotorState ]):::bugged
/motors/left/encoder([ /motors/left/encoder<br>norlab_custom_interfaces/msg/EncoderState ]):::bugged
/motors/odometry([ /motors/odometry<br>nav_msgs/msg/Odometry ]):::bugged
/motors/right/electrical([ /motors/right/electrical<br>norlab_custom_interfaces/msg/MotorState ]):::bugged
/motors/right/encoder([ /motors/right/encoder<br>norlab_custom_interfaces/msg/EncoderState ]):::bugged


/twist_mux/cmd_vel --> /roboclaw_node
/diagnostics --> /foxglove_bridge
/roboclaw_node --> /diagnostics
/roboclaw_node --> /motors/left/electrical
/roboclaw_node --> /motors/left/encoder
/roboclaw_node --> /motors/odometry
/roboclaw_node --> /motors/right/electrical
/roboclaw_node --> /motors/right/encoder
/twist_mux --> /twist_mux/cmd_vel




subgraph keys[<b>Keys<b/>]
subgraph nodes[<b><b/>]
topicb((No connected)):::bugged
main_node[main]:::main
end
subgraph connection[<b><b/>]
node1[node1]:::node
node2[node2]:::node
node1 o-.-o|to server| service[/Service<br>service/Type\]:::service
service <-.->|to client| node2
node1 -->|publish| topic([Topic<br>topic/Type]):::topic
topic -->|subscribe| node2
node1 o==o|to server| action{{/Action<br>action/Type/}}:::action
action <==>|to client| node2
end
end
classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff
classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff
classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff
classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff
classDef main opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff
classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff
style keys opacity:0.15,fill:#FFF
style nodes opacity:0.15,fill:#FFF
style connection opacity:0.15,fill:#FFF

```

