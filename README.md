# NAOqi Miscellaneous for ROS 2

This ROS 2 package provides a bridge to various miscellaneous functionalities of SoftBank Robotics' Pepper and NAO robots, which run on the NAOqi OS. It exposes modules like `ALAutonomousLife`, `ALBasicAwareness`, `ALAutonomousBlinking`, and `ALBattery` as ROS 2 services and topics.

## Features

*   **Autonomous Life**: Enable or disable the robot's autonomous behaviors.
*   **Basic Awareness**: Toggle the robot's basic awareness (e.g., tracking sounds and movement).
*   **Autonomous Blinking**: Enable or disable the robot's automatic eye blinking.
*   **Battery Monitoring**: Periodically publishes the robot's battery charge percentage.

## Dependencies

*   `rclpy`
*   `std_srvs`
*   `std_msgs`

## How to Run the Node

To start the node, you need to provide the IP address and port of the robot.

```bash
ros2 run naoqi_miscellaneous naoqi_miscellaneous_node --ros-args -p ip:=<robot_ip> -p port:=<robot_port>
```

For example:
```bash
ros2 run naoqi_miscellaneous naoqi_miscellaneous_node --ros-args -p ip:=192.168.1.101 -p port:=9559
```

## ROS 2 API

All services and topics are exposed under the node's namespace (`/naoqi_miscellaneous_node/` by default).

### Services

*   **`~/set_autonomous_state`** ([std_srvs/srv/SetBool](https://docs.ros2.org/foxy/api/std_srvs/srv/SetBool.html))  
    Enables (`true`) or disables (`false`) the robot's Autonomous Life mode. Disabling it also makes the robot go to a standing posture.

*   **`~/toggle_awareness`** ([std_srvs/srv/SetBool](https://docs.ros2.org/foxy/api/std_srvs/srv/SetBool.html))  
    Enables (`true`) or disables (`false`) the robot's Basic Awareness.

*   **`~/toggle_blinking`** ([std_srvs/srv/SetBool](https://docs.ros2.org/foxy/api/std_srvs/srv/SetBool.html))  
    Enables (`true`) or disables (`false`) the robot's autonomous blinking.

### Published Topics

*   **`~/battery_percentage`** ([std_msgs/msg/Float32](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html))  
    Publishes the robot's current battery charge percentage every 5 seconds.

## Usage Example

To disable the robot's autonomous life:

```bash
ros2 service call /naoqi_miscellaneous_node/set_autonomous_state std_srvs/srv/SetBool "{data: false}"
```

To enable autonomous blinking:

```bash
ros2 service call /naoqi_miscellaneous_node/toggle_blinking std_srvs/srv/SetBool "{data: true}"
```

To monitor the battery level:

```bash
ros2 topic echo /naoqi_miscellaneous_node/battery_percentage
```