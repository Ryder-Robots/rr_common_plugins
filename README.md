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
colcon build --packages-select rr_common_plugins  --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

#### Running the Debug Node

```bash
source install/local_setup.sh
ros2 run --prefix 'gdbserver localhost:3000' rr_common_plugins imu_action_serial_debug_node
```

#### VSCode Debug Configuration

The package includes a pre-configured VSCode launch configuration in [.vscode/launch.json](.vscode/launch.json) that connects to the gdbserver. This configuration uses remote debugging, which provides more control over the debugging session and works well with ROS 2 nodes.

**Purpose**: The launch configuration connects VSCode's debugger to a gdbserver instance running the debug node. This allows you to:
- Set and hit breakpoints in the source code
- Step through code execution line by line
- Inspect variable values and call stacks
- Debug timing-sensitive issues without process attachment overhead

**Configuration**:
```json
{
    "name": "(gdb) Attach to imu_action_serial_debug_node",
    "type": "cppdbg",
    "request": "launch",
    "program": "${workspaceFolder}/../../install/rr_common_plugins/lib/rr_common_plugins/imu_action_serial_debug_node",
    "MIMode": "gdb",
    "miDebuggerServerAddress": "localhost:3000",
    "setupCommands": [
        {
            "description": "Enable pretty-printing for gdb",
            "text": "-enable-pretty-printing",
            "ignoreFailures": true
        }
    ]
}
```

**Key parameters**:
- `miDebuggerServerAddress`: Connects to gdbserver on localhost:3000
- `program`: Points to the installed debug node binary for symbol resolution
- `setupCommands`: Enables GDB pretty-printing for better variable visualization

#### Attaching the Debugger (VSCode)

1. **Start the debug node with gdbserver** in a terminal:
   ```bash
   cd ~/ros2_ws
   source install/local_setup.sh
   ros2 run --prefix 'gdbserver localhost:3000' rr_common_plugins imu_action_serial_debug_node
   ```

   You should see output like:
   ```
   Process /path/to/imu_action_serial_debug_node created; pid = XXXXX
   Listening on port 3000
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
   - VSCode will connect to the gdbserver

5. **Debug**: The debugger is now connected. The node will start executing, and execution will pause at your breakpoints, allowing you to inspect variables, step through code, and diagnose issues.