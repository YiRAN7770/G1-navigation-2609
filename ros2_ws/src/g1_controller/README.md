# G1 Controller - ROS2 cmd_vel Bridge

This package provides a ROS2 node that bridges standard `cmd_vel` messages to the Unitree G1 robot's locomotion client.

## Overview

The `cmd_vel_to_g1` node subscribes to `/cmd_vel` topic and translates `geometry_msgs/TwistStamped` messages into calls to the `g1_loco_client` command. This allows you to control the G1 robot using standard ROS2 navigation interfaces.

## Features

- **Rate Limited**: Configurable minimum interval between commands
- **Configurable**: Network interface and paths can be configured via parameters
- **Single Call**: Each cmd_vel message results in exactly ONE call to g1_loco_client (no continuous calling)
- **Navigation Ready**: Compatible with ROS2 Nav2 stack

## Building

```bash
cd <repo-root-or-external-dependency>
source <repo-root-or-external-dependency>
colcon build --symlink-install --packages-select g1_controller
source install/setup.bash
```

## Usage

### Method 1: Using the launch file (Recommended)

```bash
ros2 launch g1_controller cmd_vel_to_g1.launch.py
```

With custom parameters:

```bash
ros2 launch g1_controller cmd_vel_to_g1.launch.py \
    network_interface:=eno1 \
    g1_loco_client_path:=<repo-root-or-external-dependency> \
    command_timeout:=0.1
```

### Method 2: Running the node directly

```bash
ros2 run g1_controller cmd_vel_to_g1
```

## Parameters

- `network_interface` (string, default: `eno1`): Network interface for G1 robot communication
- `g1_loco_client_path` (string, default: value of `G1_LOCO_CLIENT_BIN`): Path to the client executable
- `unitree_sdk_lib_path` (string, default: value of `G1_LOCO_CLIENT_BIN`): Path to Unitree SDK shared libraries
- `command_timeout` (double, default: `0.1`): Minimum interval between accepted commands in seconds
- `max_x_linear_velocity` (double, default: `0.5`): X velocity limit in m/s
- `max_y_linear_velocity` (double, default: `0.15`): Y velocity limit in m/s
- `max_angular_velocity` (double, default: `0.5`): Yaw velocity limit in rad/s

## Topic Interface

### Subscribed Topics

- `/cmd_vel` (geometry_msgs/TwistStamped): Standard velocity commands
  - `twist.linear.x`: Forward/backward velocity (m/s)
  - `twist.linear.y`: Left/right velocity (m/s)
  - `twist.angular.z`: Rotation velocity (rad/s)

## Testing

Send a test command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

Send a command to move forward with left strafe:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

Send a rotation command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}"
```

## Safety Features

- **Command Debouncing**: The node enforces a minimum 0.1-second interval between accepted commands
- **Single Execution**: Each cmd_vel message triggers exactly one g1_loco_client call (no continuous execution)
- **Logging**: All commands are logged for debugging and monitoring

`command_timeout` is not a loss-of-command watchdog and does not send an automatic stop when `/cmd_vel` becomes silent.

## Integration with Navigation

This node is designed to work seamlessly with ROS2 Navigation stack (Nav2). Simply:

1. Launch this bridge node
2. Configure your Nav2 stack to publish to `/cmd_vel`
3. The G1 robot will follow navigation commands automatically

## Troubleshooting

### Command execution failed
- Verify the g1_loco_client path is correct
- Check network interface is connected and correct
- Ensure you have execute permissions on g1_loco_client

### Robot not responding
- Check that the robot is powered on and connected to the network
- Verify the network interface name matches your system
- Try running g1_loco_client manually to test connectivity

### Commands too fast warning
- This is a safety feature. Adjust `command_timeout` parameter if needed
- Default 0.1s timeout should be sufficient for most applications
