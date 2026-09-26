#include "lift_slide_driver/lift_slide_state_controller.hpp"

#include <cstdio>
#include <iomanip>
#include <sstream>

#include "pluginlib/class_list_macros.hpp"

namespace lift_slide_driver
{

// --- State interface name list (must match hardware_interface export order) ---
static const std::vector<std::string> kStateInterfaceNames = {
  "cia402_state", "statusword", "is_enabled", "is_fault", "is_target_reached",
  "mode_of_operation", "homing_state", "homing_detail", "homing_complete", "is_homing",
  "upper_limit_switch", "home_switch", "lower_limit_switch", "limit_switch_valid",
  "digital_inputs_raw", "physical_position", "profile_speed_mps",
  "profile_accel_mps2", "profile_decel_mps2", "drive_enabled", "quick_stop_active",
  "motion_ready", "reference_valid", "encoder_reference_lost"
};

// --- Command interface name list ---
static const std::vector<std::string> kCommandInterfaceNames = {
  "enable_cmd", "quick_stop_cmd", "reset_fault_cmd", "full_reset_cmd",
  "homing_cmd", "set_mode_cmd", "profile_speed_cmd", "accel_time_cmd", "decel_time_cmd",
  "hold_position_cmd"
};

// ============================================================================
// Lifecycle
// ============================================================================

controller_interface::CallbackReturn LiftSlideStateController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LiftSlideStateController::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Resolve joint name
  try {
    if (get_node()->has_parameter("joints")) {
      auto joints = get_node()->get_parameter("joints").as_string_array();
      if (!joints.empty()) {
        joint_name_ = joints[0];
      }
    }
  } catch (...) {}

  // Create publishers with absolute topic names (matching what panel subscribes to)
  motor_status_pub_ = get_node()->create_publisher<lift_slide_msgs::msg::MotorStatus>(
    "/lift_slide_driver/motor_status", rclcpp::SystemDefaultsQoS());

  homing_state_pub_ = get_node()->create_publisher<std_msgs::msg::String>(
    "/lift_slide_driver/homing_state", rclcpp::SystemDefaultsQoS());

  limit_switch_state_pub_ = get_node()->create_publisher<std_msgs::msg::String>(
    "/lift_slide_driver/limit_switch_state", rclcpp::SystemDefaultsQoS());

  // Create service servers with absolute names
  start_homing_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "/lift_slide_driver/start_homing",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (get_state("quick_stop_active") > 0.5 || get_state("is_fault") > 0.5) {
        response->success = false;
        response->message = "set-zero rejected: quick stop or drive fault is active";
        return;
      }
      if (get_state("is_homing") > 0.5) {
        response->success = false;
        response->message = "set-zero rejected: another zero/home action is active";
        return;
      }
      set_command("homing_cmd", 1.0);
      response->success = true;
      response->message = "set-zero accepted; monitor /lift_slide_driver/homing_state";
    });

  return_home_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "/lift_slide_driver/return_home",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (get_state("quick_stop_active") > 0.5 || get_state("is_fault") > 0.5) {
        response->success = false;
        response->message = "return-home rejected: quick stop or drive fault is active";
        return;
      }
      if (get_state("is_homing") > 0.5) {
        response->success = false;
        response->message = "return-home rejected: another zero/home action is active";
        return;
      }
      if (get_state("homing_complete") <= 0.5) {
        response->success = false;
        response->message = "return-home rejected: run set-zero first";
        return;
      }
      set_command("homing_cmd", 2.0);
      response->success = true;
      response->message = "return-home accepted; monitor /lift_slide_driver/homing_state";
    });

  enable_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "/lift_slide_driver/enable",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      set_command("enable_cmd", 1.0);
      response->success = true;
      response->message = "enable accepted";
    });

  quick_stop_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "/lift_slide_driver/quick_stop",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      set_command("quick_stop_cmd", 1.0);
      response->success = true;
      response->message = "quick stop accepted";
    });

  hold_position_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "/lift_slide_driver/hold_position",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      set_command("hold_position_cmd", 1.0);
      response->success = true;
      response->message = "hold position accepted";
    });

  // 订阅控制器命名空间下的速度命令话题
  profile_speed_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/profile_speed_cmd",
    rclcpp::QoS(1),
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      set_command("profile_speed_cmd", msg->data);
    });

  // RViz 面板当前发布到这个绝对话题；同时订阅以兼容面板和控制器命名空间两种路径。
  profile_speed_panel_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "/lift_state_controller/profile_speed_cmd",
    rclcpp::QoS(1),
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      set_command("profile_speed_cmd", msg->data);
    });

  // Declare parameters for profile acceleration/deceleration
  try {
    get_node()->declare_parameter<double>("profile_acceleration_mps2", 0.025);
  } catch (...) {}
  try {
    get_node()->declare_parameter<double>("profile_deceleration_mps2", 0.025);
  } catch (...) {}

  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters)
      -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & param : parameters) {
        if (param.get_name() == "profile_acceleration_mps2") {
          set_command("accel_time_cmd", param.as_double());
        } else if (param.get_name() == "profile_deceleration_mps2") {
          set_command("decel_time_cmd", param.as_double());
        }
      }
      return result;
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LiftSlideStateController::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Reset change-detection cache
  prev_homing_state_ = -1;
  prev_homing_detail_ = -1;
  prev_upper_limit_ = -1.0;
  prev_home_switch_ = -1.0;
  prev_lower_limit_ = -1.0;
  prev_digital_inputs_raw_ = -1.0;
  update_cycle_count_ = 0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LiftSlideStateController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LiftSlideStateController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  motor_status_pub_.reset();
  homing_state_pub_.reset();
  limit_switch_state_pub_.reset();
  start_homing_srv_.reset();
  return_home_srv_.reset();
  enable_srv_.reset();
  quick_stop_srv_.reset();
  hold_position_srv_.reset();
  profile_speed_sub_.reset();
  profile_speed_panel_sub_.reset();
  param_callback_handle_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Interface configuration
// ============================================================================

controller_interface::InterfaceConfiguration
LiftSlideStateController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : kCommandInterfaceNames) {
    config.names.push_back(joint_name_ + "/" + name);
  }
  return config;
}

controller_interface::InterfaceConfiguration
LiftSlideStateController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Standard interfaces
  config.names.push_back(joint_name_ + "/position");
  config.names.push_back(joint_name_ + "/velocity");

  // Extended interfaces
  for (const auto & name : kStateInterfaceNames) {
    config.names.push_back(joint_name_ + "/" + name);
  }
  return config;
}

// ============================================================================
// Update loop — called at controller_manager rate (100Hz)
// ============================================================================

controller_interface::return_type LiftSlideStateController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  ++update_cycle_count_;

  // --- Publish homing_state (String) on change ---
  {
    const int state = static_cast<int>(get_state("homing_state"));
    const int detail = static_cast<int>(get_state("homing_detail"));
    if (state != prev_homing_state_ || detail != prev_homing_detail_) {
      prev_homing_state_ = state;
      prev_homing_detail_ = detail;

      std_msgs::msg::String msg;
      const std::string state_str = homing_state_to_string(state);
      const std::string detail_str = homing_detail_to_string(detail);
      msg.data = detail_str.empty() ? state_str : (state_str + ":" + detail_str);
      homing_state_pub_->publish(msg);
    }
  }

  // --- Publish limit_switch_state (String) on change ---
  {
    const double upper = get_state("upper_limit_switch");
    const double home = get_state("home_switch");
    const double lower = get_state("lower_limit_switch");
    const double raw = get_state("digital_inputs_raw");
    if (upper != prev_upper_limit_ || home != prev_home_switch_ ||
        lower != prev_lower_limit_ || raw != prev_digital_inputs_raw_)
    {
      prev_upper_limit_ = upper;
      prev_home_switch_ = home;
      prev_lower_limit_ = lower;
      prev_digital_inputs_raw_ = raw;

      const uint32_t raw_u32 = static_cast<uint32_t>(raw);
      std::ostringstream oss;
      oss << "raw=0x" << std::hex << std::setfill('0') << std::setw(8) << raw_u32 << std::dec
          << ",upper=" << (upper > 0.5 ? 1 : 0)
          << ",home=" << (home > 0.5 ? 1 : 0)
          << ",lower=" << (lower > 0.5 ? 1 : 0)
          << ",b0_NOT=" << ((raw_u32 >> 0) & 1U)
          << ",b1_POT=" << ((raw_u32 >> 1) & 1U)
          << ",b2_HOME=" << ((raw_u32 >> 2) & 1U)
          << ",b7_SI4=" << ((raw_u32 >> 7) & 1U)
          << ",b8_SI5=" << ((raw_u32 >> 8) & 1U)
          << ",b9_SI6=" << ((raw_u32 >> 9) & 1U)
          << ",b19_DI4=" << ((raw_u32 >> 19) & 1U)
          << ",b20_DI5=" << ((raw_u32 >> 20) & 1U)
          << ",b21_DI6=" << ((raw_u32 >> 21) & 1U)
          << ",b27_DI4=" << ((raw_u32 >> 27) & 1U)
          << ",b28_DI5=" << ((raw_u32 >> 28) & 1U)
          << ",b29_DI6=" << ((raw_u32 >> 29) & 1U);

      std_msgs::msg::String msg;
      msg.data = oss.str();
      limit_switch_state_pub_->publish(msg);
    }
  }

  // --- Publish motor_status (MotorStatus) at throttled rate ---
  if ((update_cycle_count_ % kMotorStatusPublishDivisor) == 0) {
    lift_slide_msgs::msg::MotorStatus msg;
    msg.stamp = get_node()->get_clock()->now();
    msg.position_m = get_state("position");
    msg.velocity_mps = get_state("velocity");
    msg.physical_position_m = get_state("physical_position");
    msg.statusword = static_cast<uint16_t>(get_state("statusword"));
    msg.cia402_state = cia402_state_to_string(static_cast<int>(get_state("cia402_state")));
    msg.mode_of_operation = mode_to_string(static_cast<int>(get_state("mode_of_operation")));
    msg.is_enabled = get_state("is_enabled") > 0.5;
    msg.is_fault = get_state("is_fault") > 0.5;
    msg.is_target_reached = get_state("is_target_reached") > 0.5;
    msg.is_homing = get_state("is_homing") > 0.5;
    msg.digital_inputs_raw = static_cast<uint32_t>(get_state("digital_inputs_raw"));
    msg.upper_limit_switch = get_state("upper_limit_switch") > 0.5;
    msg.home_switch = get_state("home_switch") > 0.5;
    msg.lower_limit_switch = get_state("lower_limit_switch") > 0.5;
    msg.limit_switch_valid = get_state("limit_switch_valid") > 0.5;
    msg.homing_complete = get_state("homing_complete") > 0.5;
    msg.motion_ready = get_state("motion_ready") > 0.5;
    msg.reference_valid = get_state("reference_valid") > 0.5;
    msg.encoder_reference_lost = get_state("encoder_reference_lost") > 0.5;

    const int homing_state = static_cast<int>(get_state("homing_state"));
    const bool is_homing = get_state("is_homing") > 0.5;
    const bool homing_complete = get_state("homing_complete") > 0.5;
    if (is_homing) {
      msg.homing_state = "IN_PROGRESS";
    } else if (homing_complete) {
      msg.homing_state = "COMPLETED";
    } else if (homing_state == 3) {  // HomingStateCode::ERROR
      msg.homing_state = "ERROR";
    } else {
      msg.homing_state = "IDLE";
    }

    msg.profile_speed_mps = get_state("profile_speed_mps");
    msg.profile_accel_mps2 = get_state("profile_accel_mps2");
    msg.profile_decel_mps2 = get_state("profile_decel_mps2");

    motor_status_pub_->publish(msg);
  }

  return controller_interface::return_type::OK;
}

// ============================================================================
// Helpers
// ============================================================================

double LiftSlideStateController::get_state(const std::string & interface_name) const
{
  const std::string full_name = joint_name_ + "/" + interface_name;
  for (const auto & si : state_interfaces_) {
    if (si.get_prefix_name() == joint_name_ && si.get_interface_name() == interface_name) {
      return si.get_value();
    }
  }
  return 0.0;
}

void LiftSlideStateController::set_command(const std::string & interface_name, double value)
{
  for (auto & ci : command_interfaces_) {
    if (ci.get_prefix_name() == joint_name_ && ci.get_interface_name() == interface_name) {
      ci.set_value(value);
      return;
    }
  }
}

std::string LiftSlideStateController::homing_state_to_string(int state)
{
  switch (state) {
    case 0: return "IDLE";
    case 1: return "IN_PROGRESS";
    case 2: return "COMPLETED";
    case 3: return "ERROR";
    default: return "IDLE";
  }
}

std::string LiftSlideStateController::homing_detail_to_string(int detail)
{
  switch (detail) {
    case 0: return "";           // NONE
    case 1: return "fake_hardware";
    case 2: return "ready";
    case 3: return "service_ready";
    case 4: return "enabled";
    case 5: return "disabled";
    case 10: return "accepted";
    case 11: return "configuring";
    case 12: return "moving_up_to_home";
    case 13: return "moving_up_to_upper";
    case 14: return "moving_down_to_home";
    case 20: return "ok";
    case 21: return "calibration_restored";
    case 30: return "shutdown";
    case 31: return "quick_stop_active";
    case 32: return "enable_failed";
    case 33: return "full_reset_failed";
    case 34: return "failed";
    case 35: return "aborted_by_quick_stop";
    case 36: return "encoder_reference_lost";
    default: return "";
  }
}

std::string LiftSlideStateController::cia402_state_to_string(int state)
{
  switch (state) {
    case 0: return "NOT_READY_TO_SWITCH_ON";
    case 1: return "SWITCH_ON_DISABLED";
    case 2: return "READY_TO_SWITCH_ON";
    case 3: return "SWITCHED_ON";
    case 4: return "OPERATION_ENABLED";
    case 5: return "QUICK_STOP_ACTIVE";
    case 6: return "FAULT_REACTION_ACTIVE";
    case 7: return "FAULT";
    default: return "UNKNOWN";
  }
}

std::string LiftSlideStateController::mode_to_string(int mode)
{
  switch (mode) {
    case 1: return "position";
    case 3: return "velocity";
    case 6: return "homing";
    default: return "unknown";
  }
}

}  // namespace lift_slide_driver

PLUGINLIB_EXPORT_CLASS(
  lift_slide_driver::LiftSlideStateController,
  controller_interface::ControllerInterface)
