# lift_slide_driver

[English](./README.md) | 中文

---

升降滑台机构的 ros2_control 硬件接口和自定义控制器插件。通过 SocketCAN 与 CANopen 伺服驱动器通信。

## 功能

- CiA402 状态机管理（使能、失能、故障复位、快速停止）
- 速度模式（模式 3）和位置模式（模式 1）
- 回零模式（模式 6），支持可配置的 DI 映射
- 异步反馈轮询线程
- 位置标定保存/加载
- 限位开关监测（上限位/下限位/原点，通过数字输入）
- 手动步进和连续点动，带加速度规划

## 硬件接口插件

**插件名：** `lift_slide_driver/LiftSlideHardwareInterface`
**类型：** `hardware_interface::SystemInterface`

### 命令接口（lift_joint）

- `velocity` - 速度指令
- `position` - 位置指令

### 状态接口（lift_joint）

- `position`、`velocity` - 标准运动反馈
- `cia402_state`、`statusword`、`is_enabled`、`is_fault`、`is_target_reached`
- `mode_of_operation`、`homing_state`、`homing_detail`、`homing_complete`、`is_homing`
- `upper_limit_switch`、`home_switch`、`lower_limit_switch`、`limit_switch_valid`、`digital_inputs_raw`
- `physical_position`、`profile_speed_mps`、`profile_accel_mps2`、`profile_decel_mps2`
- `drive_enabled`、`quick_stop_active`

### 扩展命令接口

`enable_cmd`、`quick_stop_cmd`、`reset_fault_cmd`、`full_reset_cmd`、`homing_cmd`、`set_mode_cmd`、`profile_speed_cmd`、`accel_time_cmd`、`decel_time_cmd`、`hold_position_cmd`

## 控制器插件

| 插件名 | 说明 |
|--------|------|
| `lift_slide_driver/ParamForwardingVelocityController` | 订阅速度指令并转发至硬件接口 |
| `lift_slide_driver/ParamForwardingPositionController` | 订阅位置指令并转发至硬件接口 |
| `lift_slide_driver/LiftSlideStateController` | 将硬件状态桥接到 ROS 话题/服务 |
| `lift_slide_driver/LiftSlideManualPositionController` | 处理步进移动和连续点动，带加速度规划 |

## 发布的话题（LiftSlideStateController）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/lift_slide_driver/motor_status` | `lift_slide_msgs/MotorStatus` | 综合电机状态（约 20 Hz） |
| `/lift_slide_driver/homing_state` | `std_msgs/String` | 回零状态变化 |
| `/lift_slide_driver/limit_switch_state` | `std_msgs/String` | 限位开关状态变化 |

## 订阅的话题（LiftSlideStateController）

| 话题 | 类型 | 说明 |
|------|------|------|
| `~/profile_speed_cmd` | `std_msgs/Float64` | 运行速度设置 |
| `/lift_state_controller/profile_speed_cmd` | `std_msgs/Float64` | 运行速度（面板兼容） |

## 订阅的话题（LiftSlideManualPositionController）

| 话题 | 类型 | 说明 |
|------|------|------|
| `~/step_command` | `std_msgs/Float64MultiArray` | 步进指令 [方向, 步长m, 速度m/s] |
| `~/jog_command` | `std_msgs/Float64` | 点动指令（速度m/s，0停止，符号表示方向） |

## 服务（LiftSlideStateController）

| 服务 | 类型 | 说明 |
|------|------|------|
| `/lift_slide_driver/start_homing` | `std_srvs/Trigger` | 启动回零序列 |
| `/lift_slide_driver/return_home` | `std_srvs/Trigger` | 返回原点位置 |
| `/lift_slide_driver/enable` | `std_srvs/Trigger` | 使能驱动器 |
| `/lift_slide_driver/quick_stop` | `std_srvs/Trigger` | 紧急快速停止 |
| `/lift_slide_driver/hold_position` | `std_srvs/Trigger` | 保持当前位置 |

## 编译

```bash
colcon build --packages-select lift_slide_driver
source install/setup.bash
```

## 前置依赖

- ROS 2（Humble/Iron）
- ros2_control（hardware_interface、controller_interface）
- pluginlib、realtime_tools
- lift_slide_msgs
- Linux 系统，支持 SocketCAN
- CAN 接口已配置并启动（例如 `sudo ip link set can3 up type can bitrate 500000`）

## 许可证

本包通过 知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenFlex 全身人形机器人平台生态系统的一部分，专为人形机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | https://openarmx.com/ |
| 🌐 文档 | http://docs.openarmx.com/ |
| 📍 地址 | 天津市西青区・稻潮机器人体验基地（明日之城）・天津市人形机器人中心 |
| 👤 联系人 | 王先生 |
