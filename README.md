# RR COMMON PLUGINS

A collection of common ROS 2 plugins that provide abstraction layers for network and serial communication interfaces. This package leverages the `transport_drivers` library to handle low-level communication details, allowing developers to focus on application logic.

## Overview

This package provides reusable plugins for common communication patterns in ROS 2:

- **Serial Interface Plugins**: Plugins that communicate over serial ports using `transport_drivers`' serial interface
- **UDP Interface Plugins**: Plugins that handle UDP-based communication using `transport_drivers`' UDP interface

The plugins abstract away the plumbing required for these communication methods, making it easier to integrate hardware devices and network services into ROS 2 systems.

### Current Plugins

- **imu_action_serial_plugin**: Serial communication plugin for IMU action control
- **joy_subscriber_udp_plugin**: UDP-based plugin for joystick/controller data

Additional plugins will be added to this package as common communication patterns are identified.

## Installation

```bash
sudo apt update && sudo apt install -y protobuf-compiler libprotobuf-dev
```

## Debugging

### Debugging imu_action_serial_plugin

The package includes a dedicated debug node (`imu_action_serial_debug_node`) for testing and debugging the `imu_action_serial_plugin` without requiring the full ROS 2 system.

#### Building the Debug Node

The debug node is built automatically with the package:

```bash
cd ~/ros2_ws
colcon build --packages-select rr_common_plugins
```

#### Running the Debug Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rr_common_plugins imu_action_serial_debug_node
```

#### Attaching the Debugger (VSCode)

1. **Start the debug node** in a terminal:
   ```bash
   ros2 run rr_common_plugins imu_action_serial_debug_node
   ```

2. **Open VSCode** in the workspace:
   ```bash
   code ~/ros2_ws/src/rr_common_plugins
   ```

3. **Set breakpoints** in the relevant source files:
   - [src/imu_action_serial_plugin.cpp](src/imu_action_serial_plugin.cpp)
   - [src/debug/imu_action_serial_debug_node.cpp](src/debug/imu_action_serial_debug_node.cpp)

4. **Attach the debugger**:
   - Open the Run and Debug panel (Ctrl+Shift+D)
   - Select "(gdb) Attach to imu_action_serial_debug_node" from the dropdown
   - Press F5 or click the green play button
   - When prompted, select the `imu_action_serial_debug_node` process from the list

5. **Debug**: The debugger is now attached. Execution will pause at your breakpoints, allowing you to inspect variables, step through code, and diagnose issues.

#### Alternative: Launch with Debugger Attached

You can also modify [.vscode/launch.json](.vscode/launch.json) to add a launch configuration that starts the node with the debugger already attached, eliminating the need for the attach step.