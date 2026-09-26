# lift_slide_driver

English | [中文](./README-CN.md)

---

ros2_control hardware interface and custom controller plugins for the lift-slide mechanism. Communicates with a CANopen servo drive over SocketCAN.

## Features

- CiA402 state machine management (enable, disable, fault reset, quick stop)
- Velocity mode (mode 3) and Position mode (mode 1)
- Homing mode (mode 6) with configurable DI mapping
- Async feedback polling thread
- Position calibration save/load
- Limit switch monitoring (upper/lower/home via digital inputs)
- Manual step and continuous jog with acceleration profiling

## Hardware Interface Plugin

**Plugin name:** `lift_slide_driver/LiftSlideHardwareInterface`
**Type:** `hardware_interface::SystemInterface`

### Command Interfaces (lift_joint)

- `velocity` - Velocity command
- `position` - Position command

### State Interfaces (lift_joint)

- `position`, `velocity` - Standard motion feedback
- `cia402_state`, `statusword`, `is_enabled`, `is_fault`, `is_target_reached`
- `mode_of_operation`, `homing_state`, `homing_detail`, `homing_complete`, `is_homing`
- `upper_limit_switch`, `home_switch`, `lower_limit_switch`, `limit_switch_valid`, `digital_inputs_raw`
- `physical_position`, `profile_speed_mps`, `profile_accel_mps2`, `profile_decel_mps2`
- `drive_enabled`, `quick_stop_active`

### Extended Command Interfaces

`enable_cmd`, `quick_stop_cmd`, `reset_fault_cmd`, `full_reset_cmd`, `homing_cmd`, `set_mode_cmd`, `profile_speed_cmd`, `accel_time_cmd`, `decel_time_cmd`, `hold_position_cmd`

## Controller Plugins

| Plugin Name | Description |
|-------------|-------------|
| `lift_slide_driver/ParamForwardingVelocityController` | Subscribes to velocity commands and forwards to hardware interface |
| `lift_slide_driver/ParamForwardingPositionController` | Subscribes to position commands and forwards to hardware interface |
| `lift_slide_driver/LiftSlideStateController` | Bridges hardware state to ROS topics/services |
| `lift_slide_driver/LiftSlideManualPositionController` | Handles step moves and continuous jog with acceleration profiling |

## Topics Published (LiftSlideStateController)

| Topic | Type | Description |
|-------|------|-------------|
| `/lift_slide_driver/motor_status` | `lift_slide_msgs/MotorStatus` | Consolidated motor status (~20 Hz) |
| `/lift_slide_driver/homing_state` | `std_msgs/String` | Homing state changes |
| `/lift_slide_driver/limit_switch_state` | `std_msgs/String` | Limit switch state changes |

## Topics Subscribed (LiftSlideStateController)

| Topic | Type | Description |
|-------|------|-------------|
| `~/profile_speed_cmd` | `std_msgs/Float64` | Profile speed setting |
| `/lift_state_controller/profile_speed_cmd` | `std_msgs/Float64` | Profile speed (panel compatibility) |

## Topics Subscribed (LiftSlideManualPositionController)

| Topic | Type | Description |
|-------|------|-------------|
| `~/step_command` | `std_msgs/Float64MultiArray` | Step command [direction, step_m, speed_mps] |
| `~/jog_command` | `std_msgs/Float64` | Jog command (speed_mps, 0 to stop, sign for direction) |

## Services (LiftSlideStateController)

| Service | Type | Description |
|---------|------|-------------|
| `/lift_slide_driver/start_homing` | `std_srvs/Trigger` | Start homing sequence |
| `/lift_slide_driver/return_home` | `std_srvs/Trigger` | Return to home position |
| `/lift_slide_driver/enable` | `std_srvs/Trigger` | Enable drive |
| `/lift_slide_driver/quick_stop` | `std_srvs/Trigger` | Emergency quick stop |
| `/lift_slide_driver/hold_position` | `std_srvs/Trigger` | Hold current position |

## Build

```bash
colcon build --packages-select lift_slide_driver
source install/setup.bash
```

## Prerequisites

- ROS 2 (Humble/Iron)
- ros2_control (hardware_interface, controller_interface)
- pluginlib, realtime_tools
- lift_slide_msgs
- Linux with SocketCAN support
- CAN interface configured and up (e.g., `sudo ip link set can3 up type can bitrate 500000`)

## License

This package is licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, please refer to the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenFlex full-body humanoid robot platform ecosystem, developed specifically for research and industrial applications in the humanoid robotics field.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.
**Chengdu Changshu Robotics Co., Ltd.**

| Contact | Information |
|---------|-------------|
| 📧 Email | openarmrobot@gmail.com |
| 📱 Phone/WeChat | +86-17746530375 |
| 🌐 Website | https://openarmx.com/ |
| 🌐 Docs | http://docs.openarmx.com/ |
| 📍 Address | Tianjin Xiqing District · Daochao Robot Experience Base (City of Tomorrow) · Tianjin Humanoid Robot Center |
| 👤 Contact Person | Mr. Wang |
