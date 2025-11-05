# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This repository provides ROS2 support for Unitree robots (Go2, B2, B2W, G1, H1, H1-2) by leveraging the fact that both Unitree SDK2 and ROS2 use CycloneDDS as their communication mechanism. This enables direct communication with Unitree robots using ROS2 messages without wrapping SDK interfaces.

**Supported robots**: Go2, B2, B2W, G1, H1, H1-2
**Supported ROS2 distributions**: Foxy (Ubuntu 20.04), Humble (Ubuntu 22.04, recommended)

## Build Commands

### Initial Setup

Before building, ensure you have the required dependencies:
```bash
# For Foxy
sudo apt install ros-foxy-rmw-cyclonedds-cpp ros-foxy-rosidl-generator-dds-idl libyaml-cpp-dev

# For Humble
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl libyaml-cpp-dev
```

### Build Process

The build follows a two-stage process due to dependency ordering:

**Stage 1: Build CycloneDDS (Foxy only, skip for Humble)**
```bash
cd cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
# Important: Do NOT source ROS2 environment before this step
export LD_LIBRARY_PATH=/opt/ros/foxy/lib  # Only if build fails
colcon build --packages-select cyclonedds
```

**Stage 2: Build Unitree ROS2 Packages**
```bash
source /opt/ros/humble/setup.bash  # or /opt/ros/foxy/setup.bash
cd cyclonedds_ws
colcon build --packages-select unitree_go unitree_hg unitree_api
```

**Stage 3: Build Examples**
```bash
cd example
colcon build
```

### Running Examples

Before running any examples, you must source the setup script:
```bash
source ~/unitree_ros2/setup.sh
```

Then run examples from the example directory:
```bash
cd example
./install/unitree_ros2_example/bin/read_motion_state
./install/unitree_ros2_example/bin/read_low_state
./install/unitree_ros2_example/bin/go2_sport_client
# etc.
```

## Network Configuration

**Critical**: The `setup.sh` file contains network interface configuration that must match your system's Ethernet connection to the robot. Edit the `CYCLONEDDS_URI` environment variable to specify the correct network interface name (e.g., `enp3s0`, `enx9c69d31babe2`).

To find your network interface:
```bash
ifconfig
```

For local simulation (not connected to robot):
```bash
source ~/unitree_ros2/setup_local.sh  # Uses loopback interface "lo"
# or
source ~/unitree_ros2/setup_default.sh  # No interface specified
```

## Architecture

### Workspace Structure

The repository contains two separate colcon workspaces:

1. **cyclonedds_ws/**: Core DDS and message definitions
   - `cyclonedds/`: CycloneDDS 0.10.2 implementation (Foxy only)
   - `rmw_cyclonedds/`: ROS2 middleware implementation for CycloneDDS
   - `unitree/unitree_go/`: Message definitions for Go2/B2 robots
   - `unitree/unitree_hg/`: Message definitions for G1/H1/H1-2 robots (humanoid)
   - `unitree/unitree_api/`: Common API message definitions (Request/Response)

2. **example/**: Example implementations demonstrating robot control
   - Organized by robot type: `go2/`, `b2/`, `b2w/`, `g1/`, `h1-2/`
   - `common/`: Shared utilities across all robots

### Communication Architecture

**Topic-based state monitoring**:
- `/sportmodestate` or `/lf/sportmodestate`: High-level sport mode state (position, velocity, gait)
- `/lowstate` or `/lf/lowstate`: Low-level motor and hardware state
- `/wirelesscontroller`: Remote controller inputs

**Request/Response control**:
- Sport mode control uses request/response pattern via `/api/sport/request` topic
- Low-level motor control uses `/lowcmd` topic with `LowCmd` messages

### Robot-Specific Packages

- **unitree_go**: Messages for quadruped robots (Go2, B2, B2W)
  - Core messages: `LowCmd`, `LowState`, `SportModeState`, `MotorCmd`, `MotorState`

- **unitree_hg**: Messages for humanoid robots (G1, H1, H1-2)
  - Similar structure but adapted for humanoid morphology

- **unitree_api**: Request/Response message definitions used across all robots
  - Sport mode API uses numbered API IDs (e.g., 1001=DAMP, 1002=BALANCESTAND)

### Helper Classes

**SportClient** (`example/src/common/ros2_sport_client.cpp`): Provides convenient methods to construct Request messages for sport mode control. Each method sets the appropriate `api_id` and formats parameters as JSON.

**B2SportClient** (`example/src/common/ros2_b2_sport_client.cpp`): Similar to SportClient but specific to B2 robots.

**Motor CRC utilities**:
- `motor_crc.cpp`: CRC calculation for Go2/B2 low-level commands
- `motor_crc_hg.cpp`: CRC calculation for humanoid robots (G1/H1)

### Control Levels

1. **Sport Mode (High-level)**: Use SportClient to send movement commands, gaits, and behaviors
2. **Low-level Motor Control**: Direct joint position/velocity/torque control via LowCmd messages

Low-level control requires setting motor mode to 0x01 (FOC mode) and calculating CRC checksums.

## Common Development Tasks

### Testing Robot Connection

```bash
source ~/unitree_ros2/setup.sh
ros2 topic list  # Should show robot topics
ros2 topic echo /sportmodestate  # View robot state
```

### Adding New Examples

1. Create source file in `example/src/src/<robot_type>/`
2. Add executable to `example/src/CMakeLists.txt`:
   ```cmake
   add_executable(my_example src/<robot>/my_example.cpp)
   target_compile_features(my_example PRIVATE cxx_std_20)
   ament_target_dependencies(my_example ${DEPENDENCY_LIST})
   install(TARGETS my_example)
   ```
3. Rebuild: `cd example && colcon build`

### Message Definitions

All custom messages are defined in `cyclonedds_ws/src/unitree/*/msg/*.msg` files. After modifying messages, rebuild the message packages:

```bash
cd cyclonedds_ws
colcon build --packages-select unitree_go unitree_hg unitree_api
```

## Important Notes

- **CycloneDDS Version**: Unitree robots use CycloneDDS 0.10.2 specifically
- **RMW Implementation**: Always set `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- **Build Isolation**: CycloneDDS must be built WITHOUT ROS2 sourced (Foxy only)
- **C++ Standard**: Most examples use C++20 features
- **Network Interface**: Critical for robot communication; verify setup.sh configuration
- **Optional Dependencies**: `yaml-cpp >= 0.6` required for `g1_dual_arm_example`
