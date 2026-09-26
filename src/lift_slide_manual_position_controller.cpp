#include "lift_slide_driver/lift_slide_manual_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace lift_slide_driver
{
namespace
{
constexpr int kDirectionUp = 1;
constexpr int kDirectionDown = -1;
}  // namespace

controller_interface::CallbackReturn LiftSlideManualPositionController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {"lift_joint"});
    auto_declare<double>("min_position_m", -0.750);
    auto_declare<double>("max_position_m", 0.400);
    auto_declare<double>("lower_switch_position_m", 0.000);
    auto_declare<double>("home_switch_position_m", 0.650);
    auto_declare<double>("upper_switch_position_m", 0.950);
    auto_declare<double>("max_velocity_mps", 0.10);
    auto_declare<double>("max_acceleration_mps2", 0.08);
    auto_declare<double>("max_deceleration_mps2", 0.12);
    auto_declare<double>("position_command_min_delta_m", 0.001);
    auto_declare<double>("target_tolerance_m", 0.0005);
    auto_declare<double>("velocity_tolerance_mps", 0.0005);
    auto_declare<double>("jog_target_lookahead_time_s", 0.25);
    auto_declare<double>("jog_min_target_lookahead_m", 0.010);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "manual position controller init failed: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LiftSlideManualPositionController::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!resolve_joint_name()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to resolve lift joint name");
    return controller_interface::CallbackReturn::ERROR;
  }

  min_position_m_ = get_node()->get_parameter("min_position_m").as_double();
  max_position_m_ = get_node()->get_parameter("max_position_m").as_double();
  lower_switch_position_m_ = get_node()->get_parameter("lower_switch_position_m").as_double();
  home_switch_position_m_ = get_node()->get_parameter("home_switch_position_m").as_double();
  upper_switch_position_m_ = get_node()->get_parameter("upper_switch_position_m").as_double();
  max_velocity_mps_ = get_node()->get_parameter("max_velocity_mps").as_double();
  max_acceleration_mps2_ = get_node()->get_parameter("max_acceleration_mps2").as_double();
  max_deceleration_mps2_ = get_node()->get_parameter("max_deceleration_mps2").as_double();
  position_command_min_delta_m_ =
    get_node()->get_parameter("position_command_min_delta_m").as_double();
  target_tolerance_m_ = get_node()->get_parameter("target_tolerance_m").as_double();
  velocity_tolerance_mps_ = get_node()->get_parameter("velocity_tolerance_mps").as_double();
  jog_target_lookahead_time_s_ =
    get_node()->get_parameter("jog_target_lookahead_time_s").as_double();
  jog_min_target_lookahead_m_ =
    get_node()->get_parameter("jog_min_target_lookahead_m").as_double();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "manual lift soft limits=[%.3f, %.3f]; switch positions are sensor/homing metadata only: lower=%.3f home=%.3f upper=%.3f",
    min_position_m_,
    max_position_m_,
    lower_switch_position_m_,
    home_switch_position_m_,
    upper_switch_position_m_);

  step_command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/lift_manual_position_controller/step_command",
    rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      if (msg->data.empty()) {
        RCLCPP_WARN(get_node()->get_logger(), "manual step command ignored: empty data");
        return;
      }
      const double signed_step = msg->data[0];
      if (std::abs(signed_step) < 1e-9) {
        return;
      }
      CommandRequest command;
      command.type = CommandRequest::Type::STEP;
      command.direction = signed_step > 0.0 ? kDirectionUp : kDirectionDown;
      command.step_m = std::abs(signed_step);
      command.speed_mps = msg->data.size() >= 2 ? msg->data[1] : max_velocity_mps_;
      request_buffer_.writeFromNonRT(command);
    });

  // Compatibility endpoint for VLA/absolute position clients. Absolute
  // targets are handled separately from directional manual jog commands.
  position_command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/lift_manual_position_controller/position_command",
    rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      if (msg->data.empty() || !std::isfinite(msg->data[0])) {
        RCLCPP_WARN(get_node()->get_logger(), "manual position command ignored: invalid data");
        return;
      }
      CommandRequest command;
      command.type = CommandRequest::Type::POSITION;
      command.position_m = msg->data[0];
      command.speed_mps = msg->data.size() >= 2 ? msg->data[1] : max_velocity_mps_;
      request_buffer_.writeFromNonRT(command);
    });

  jog_command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "/lift_manual_position_controller/jog_command",
    rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr msg)
    {
      CommandRequest command;
      if (std::abs(msg->data) < 1e-9) {
        command.type = CommandRequest::Type::JOG_STOP;
      } else {
        command.type = CommandRequest::Type::JOG_START;
        command.direction = msg->data > 0.0 ? kDirectionUp : kDirectionDown;
        command.speed_mps = std::abs(msg->data);
      }
      request_buffer_.writeFromNonRT(command);
    });
  profile_speed_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "/lift_state_controller/profile_speed_cmd",
    rclcpp::SystemDefaultsQoS());

  request_buffer_.writeFromNonRT(CommandRequest{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LiftSlideManualPositionController::on_activate(
  const rclcpp_lifecycle::State &)
{
  state_ = State::IDLE;
  active_direction_ = 0;
  reference_position_ = get_state_value("position");
  target_position_ = reference_position_;
  reference_velocity_ = 0.0;
  last_published_reference_ = std::numeric_limits<double>::quiet_NaN();
  last_update_time_ = get_node()->now();
  request_buffer_.writeFromNonRT(CommandRequest{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LiftSlideManualPositionController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  state_ = State::IDLE;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LiftSlideManualPositionController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  step_command_sub_.reset();
  position_command_sub_.reset();
  jog_command_sub_.reset();
  profile_speed_pub_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
LiftSlideManualPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(joint_name_ + "/position");
  config.names.push_back(joint_name_ + "/manual_halt_cmd");
  config.names.push_back(joint_name_ + "/manual_limit_stop_cmd");
  return config;
}

controller_interface::InterfaceConfiguration
LiftSlideManualPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(joint_name_ + "/position");
  config.names.push_back(joint_name_ + "/drive_enabled");
  config.names.push_back(joint_name_ + "/is_homing");
  config.names.push_back(joint_name_ + "/homing_complete");
  config.names.push_back(joint_name_ + "/quick_stop_active");
  config.names.push_back(joint_name_ + "/upper_limit_switch");
  config.names.push_back(joint_name_ + "/lower_limit_switch");
  return config;
}

controller_interface::return_type LiftSlideManualPositionController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const bool is_homing = get_state_value("is_homing") > 0.5;
  if (is_homing) {
    request_buffer_.writeFromNonRT(CommandRequest{});
    active_direction_ = 0;
    inhibit_jog_until_stop_ = true;
    reference_position_ = get_state_value("position");
    reference_velocity_ = 0.0;
    target_position_ = reference_position_;
    last_published_reference_ = std::numeric_limits<double>::quiet_NaN();
    state_ = State::IDLE;
    last_update_time_ = time;
    return controller_interface::return_type::OK;
  }

  // Do not carry the homing-time jog inhibit into the next manual movement.
  if (inhibit_jog_until_stop_ && get_state_value("homing_complete") > 0.5) {
    inhibit_jog_until_stop_ = false;
  }

  const CommandRequest * request = request_buffer_.readFromRT();
  if (request != nullptr && request->type != CommandRequest::Type::NONE) {
    const CommandRequest command = *request;
    request_buffer_.writeFromNonRT(CommandRequest{});
    if (command.type == CommandRequest::Type::STEP) {
      handle_step(command);
    } else if (command.type == CommandRequest::Type::POSITION) {
      handle_position(command);
    } else if (command.type == CommandRequest::Type::JOG_START) {
      handle_jog_start(command, time);
    } else if (command.type == CommandRequest::Type::JOG_STOP ||
      command.type == CommandRequest::Type::STOP)
    {
      handle_jog_stop();
    }
  }

  const double dt = period.seconds() > 0.0 ?
    period.seconds() : std::max(0.0, (time - last_update_time_).seconds());
  last_update_time_ = time;

  if (state_ == State::JOG_ACTIVE && update_reference(dt) && state_ == State::JOG_ACTIVE) {
    const double command_target = jog_position_target();
    if (should_publish_reference(command_target)) {
      publish_reference(command_target, target_speed_mps_);
      last_published_reference_ = command_target;
    }
  } else if (state_ == State::STEP_ACTIVE) {
    monitor_position_motion();
  }

  return controller_interface::return_type::OK;
}

bool LiftSlideManualPositionController::resolve_joint_name()
{
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  if (!joints.empty()) {
    joint_name_ = joints.front();
  }
  return !joint_name_.empty();
}

double LiftSlideManualPositionController::get_state_value(const std::string & interface_name) const
{
  for (const auto & interface : state_interfaces_) {
    if (interface.get_prefix_name() == joint_name_ &&
      interface.get_interface_name() == interface_name)
    {
      return interface.get_value();
    }
  }
  return 0.0;
}

void LiftSlideManualPositionController::set_command_value(
  const std::string & interface_name, double value)
{
  for (auto & interface : command_interfaces_) {
    if (interface.get_prefix_name() == joint_name_ &&
      interface.get_interface_name() == interface_name)
    {
      interface.set_value(value);
      return;
    }
  }
}

double LiftSlideManualPositionController::clamp(double value, double lower, double upper) const
{
  return std::max(lower, std::min(upper, value));
}

double LiftSlideManualPositionController::user_lower_limit() const
{
  return min_position_m_;
}

double LiftSlideManualPositionController::user_upper_limit() const
{
  return max_position_m_;
}

bool LiftSlideManualPositionController::can_move(int direction, std::string & reason) const
{
  if (get_state_value("drive_enabled") <= 0.5) {
    reason = "drive is not enabled";
    return false;
  }
  if (get_state_value("is_homing") > 0.5) {
    reason = "homing is active";
    return false;
  }
  if (get_state_value("quick_stop_active") > 0.5) {
    reason = "quick stop is active";
    return false;
  }
  const double position = get_state_value("position");
  if (direction > 0) {
    if (get_state_value("upper_limit_switch") > 0.5) {
      reason = "upper limit switch active";
      return false;
    }
    if (position >= user_upper_limit() - target_tolerance_m_) {
      reason = "upper soft limit reached";
      return false;
    }
  } else if (direction < 0) {
    if (get_state_value("lower_limit_switch") > 0.5) {
      reason = "lower limit switch active";
      return false;
    }
    if (position <= user_lower_limit() + target_tolerance_m_) {
      reason = "lower soft limit reached";
      return false;
    }
  } else {
    reason = "invalid direction";
    return false;
  }
  return true;
}

void LiftSlideManualPositionController::publish_reference(double position, double speed_mps)
{
  if (profile_speed_pub_ != nullptr) {
    std_msgs::msg::Float64 msg;
    msg.data = clamp(std::abs(speed_mps), 0.001, max_velocity_mps_);
    profile_speed_pub_->publish(msg);
  }
  set_command_value("position", position);
}

void LiftSlideManualPositionController::trigger_manual_halt()
{
  set_command_value("manual_halt_cmd", 1.0);
}

void LiftSlideManualPositionController::trigger_limit_stop()
{
  set_command_value("manual_limit_stop_cmd", 1.0);
}

void LiftSlideManualPositionController::sync_reference_to_actual()
{
  reference_position_ = clamp(get_state_value("position"), user_lower_limit(), user_upper_limit());
  reference_velocity_ = 0.0;
  target_position_ = reference_position_;
  last_published_reference_ = std::numeric_limits<double>::quiet_NaN();
}

double LiftSlideManualPositionController::approach(double current, double target, double max_delta) const
{
  if (current < target) {
    return std::min(target, current + std::max(0.0, max_delta));
  }
  return std::max(target, current - std::max(0.0, max_delta));
}

bool LiftSlideManualPositionController::update_reference(double dt)
{
  if (dt <= 0.0) {
    return false;
  }

  const double lower = user_lower_limit();
  const double upper = user_upper_limit();
  const double position = get_state_value("position");
  if ((get_state_value("upper_limit_switch") > 0.5 && reference_velocity_ > 0.0) ||
    (get_state_value("lower_limit_switch") > 0.5 && reference_velocity_ < 0.0))
  {
    reference_position_ = clamp(position, lower, upper);
    reference_velocity_ = 0.0;
    target_position_ = reference_position_;
    active_direction_ = 0;
    state_ = State::LIMIT_HOLD;
    trigger_manual_halt();
    last_published_reference_ = reference_position_;
    RCLCPP_WARN(get_node()->get_logger(), "manual trajectory quick-stopped by hard limit");
    return false;
  }

  double desired_velocity = 0.0;
  if (state_ == State::JOG_ACTIVE) {
    desired_velocity = static_cast<double>(active_direction_) * target_speed_mps_;
  } else if (state_ == State::STEP_ACTIVE) {
    return false;
  }

  const double accel_limit = std::max(1e-6, std::abs(desired_velocity) > std::abs(reference_velocity_) ?
    max_acceleration_mps2_ : max_deceleration_mps2_);
  reference_velocity_ = approach(reference_velocity_, desired_velocity, accel_limit * dt);
  reference_position_ = clamp(reference_position_ + reference_velocity_ * dt, lower, upper);

  if ((reference_position_ <= lower + target_tolerance_m_ && reference_velocity_ < 0.0) ||
    (reference_position_ >= upper - target_tolerance_m_ && reference_velocity_ > 0.0))
  {
    reference_position_ = reference_velocity_ > 0.0 ? upper : lower;
    target_position_ = reference_position_;
    reference_velocity_ = 0.0;
    active_direction_ = 0;
    state_ = State::LIMIT_HOLD;
    trigger_limit_stop();
    return true;
  }

  if (state_ == State::JOG_ACTIVE &&
    std::abs(reference_velocity_) <= velocity_tolerance_mps_)
  {
    const double limit = active_direction_ > 0 ? upper : lower;
    if (std::abs(reference_position_ - limit) <= target_tolerance_m_) {
      reference_position_ = limit;
      target_position_ = reference_position_;
      reference_velocity_ = 0.0;
      active_direction_ = 0;
      state_ = State::LIMIT_HOLD;
      trigger_limit_stop();
      return true;
    }
  }

  return true;
}

void LiftSlideManualPositionController::monitor_position_motion()
{
  if (active_direction_ == 0) {
    return;
  }

  const double lower = user_lower_limit();
  const double upper = user_upper_limit();
  const double position = get_state_value("position");
  const bool moving_up = active_direction_ > 0;
  const bool hard_limit =
    (moving_up && get_state_value("upper_limit_switch") > 0.5) ||
    (!moving_up && get_state_value("lower_limit_switch") > 0.5);

  if (hard_limit) {
    reference_position_ = clamp(position, lower, upper);
    reference_velocity_ = 0.0;
    target_position_ = reference_position_;
    active_direction_ = 0;
    state_ = State::LIMIT_HOLD;
    trigger_manual_halt();
    RCLCPP_WARN(
      get_node()->get_logger(),
      "manual position target quick-stopped by hard limit at pos=%.4f",
      reference_position_);
    return;
  }

  const double target = clamp(target_position_, lower, upper);
  const bool reached_target = std::abs(position - target) <= target_tolerance_m_;
  const bool crossed_target =
    (moving_up && position >= target - target_tolerance_m_) ||
    (!moving_up && position <= target + target_tolerance_m_);

  if (reached_target || crossed_target) {
    reference_position_ = target;
    reference_velocity_ = 0.0;
    active_direction_ = 0;
    if (std::abs(target - (moving_up ? upper : lower)) <= target_tolerance_m_) {
      state_ = State::LIMIT_HOLD;
    } else {
      state_ = State::IDLE;
    }
    RCLCPP_INFO(
      get_node()->get_logger(),
      "manual position target reached: target=%.4f current=%.4f",
      target,
      position);
  }
}

double LiftSlideManualPositionController::jog_position_target() const
{
  const double lower = user_lower_limit();
  const double upper = user_upper_limit();
  const double lookahead = std::max(
    jog_min_target_lookahead_m_,
    target_speed_mps_ * std::max(0.0, jog_target_lookahead_time_s_));
  return clamp(
    reference_position_ + static_cast<double>(active_direction_) * lookahead,
    lower,
    upper);
}

bool LiftSlideManualPositionController::should_publish_reference(double position) const
{
  if (std::isnan(last_published_reference_)) {
    return true;
  }
  if (state_ == State::IDLE || state_ == State::LIMIT_HOLD) {
    return std::abs(position - last_published_reference_) > 1e-6;
  }
  return std::abs(position - last_published_reference_) >= position_command_min_delta_m_;
}

void LiftSlideManualPositionController::handle_step(const CommandRequest & request)
{
  std::string reason;
  if (!can_move(request.direction, reason)) {
    RCLCPP_WARN(get_node()->get_logger(), "manual step rejected: %s", reason.c_str());
    return;
  }
  sync_reference_to_actual();
  const double current = reference_position_;
  const double target = clamp(
    current + static_cast<double>(request.direction) * request.step_m,
    user_lower_limit(),
    user_upper_limit());
  target_position_ = target;
  target_speed_mps_ = clamp(std::abs(request.speed_mps), 0.001, max_velocity_mps_);
  active_direction_ = request.direction;
  reference_velocity_ = 0.0;
  state_ = std::abs(target_position_ - reference_position_) <= target_tolerance_m_ ?
    State::IDLE : State::STEP_ACTIVE;
  if (state_ == State::STEP_ACTIVE) {
    publish_reference(target_position_, target_speed_mps_);
    last_published_reference_ = target_position_;
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "manual step: current=%.4f target=%.4f step=%.4f speed=%.4f",
    current, target, request.step_m, target_speed_mps_);
}

void LiftSlideManualPositionController::handle_position(const CommandRequest & request)
{
  const double current = get_state_value("position");
  const double target = clamp(request.position_m, user_lower_limit(), user_upper_limit());
  if (std::abs(target - request.position_m) > target_tolerance_m_) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "absolute position command rejected by soft limit: requested=%.4f limits=[%.4f, %.4f]",
      request.position_m, user_lower_limit(), user_upper_limit());
    return;
  }

  const int direction = target > current ? kDirectionUp :
    (target < current ? kDirectionDown : 0);
  if (direction == 0) {
    sync_reference_to_actual();
    state_ = State::IDLE;
    return;
  }

  std::string reason;
  if (!can_move(direction, reason)) {
    RCLCPP_WARN(
      get_node()->get_logger(), "absolute position command rejected: %s", reason.c_str());
    return;
  }

  sync_reference_to_actual();
  target_position_ = target;
  target_speed_mps_ = clamp(std::abs(request.speed_mps), 0.001, max_velocity_mps_);
  active_direction_ = direction;
  reference_velocity_ = 0.0;
  state_ = State::STEP_ACTIVE;
  publish_reference(target_position_, target_speed_mps_);
  last_published_reference_ = target_position_;
  RCLCPP_INFO(
    get_node()->get_logger(),
    "manual absolute position: current=%.4f target=%.4f speed=%.4f",
    current, target_position_, target_speed_mps_);
}

void LiftSlideManualPositionController::handle_jog_start(
  const CommandRequest & request, const rclcpp::Time & time)
{
  if (inhibit_jog_until_stop_) {
    active_direction_ = 0;
    reference_velocity_ = 0.0;
    state_ = State::IDLE;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "manual jog ignored after homing until neutral command is received");
    return;
  }

  if (state_ == State::JOG_ACTIVE && active_direction_ == request.direction) {
    target_speed_mps_ = clamp(std::abs(request.speed_mps), 0.001, max_velocity_mps_);
    if (profile_speed_pub_ != nullptr) {
      std_msgs::msg::Float64 msg;
      msg.data = target_speed_mps_;
      profile_speed_pub_->publish(msg);
    }
    RCLCPP_DEBUG(
      get_node()->get_logger(),
      "manual jog command refreshed: direction=%d speed=%.4f",
      active_direction_, target_speed_mps_);
    return;
  }

  std::string reason;
  if (!can_move(request.direction, reason)) {
    RCLCPP_WARN(get_node()->get_logger(), "manual jog rejected: %s", reason.c_str());
    return;
  }
  if (state_ == State::IDLE || state_ == State::LIMIT_HOLD) {
    sync_reference_to_actual();
  }
  active_direction_ = request.direction;
  target_speed_mps_ = clamp(std::abs(request.speed_mps), 0.001, max_velocity_mps_);
  target_position_ = reference_position_;
  reference_velocity_ = 0.0;
  last_update_time_ = time;
  state_ = State::JOG_ACTIVE;
  const double command_target = jog_position_target();
  publish_reference(command_target, target_speed_mps_);
  last_published_reference_ = command_target;
  RCLCPP_INFO(
    get_node()->get_logger(),
    "manual jog started: direction=%d start=%.4f first_target=%.4f speed=%.4f",
    active_direction_, reference_position_, command_target, target_speed_mps_);
}

void LiftSlideManualPositionController::handle_jog_stop()
{
  inhibit_jog_until_stop_ = false;
  if (state_ == State::IDLE || state_ == State::LIMIT_HOLD) {
    active_direction_ = 0;
    return;
  }
  sync_reference_to_actual();
  target_position_ = reference_position_;
  active_direction_ = 0;
  reference_velocity_ = 0.0;
  trigger_manual_halt();
  state_ = State::IDLE;
  RCLCPP_INFO(
    get_node()->get_logger(),
    "manual jog stop: halt at pos=%.4f vel=%.4f",
    reference_position_, reference_velocity_);
}

}  // namespace lift_slide_driver

PLUGINLIB_EXPORT_CLASS(
  lift_slide_driver::LiftSlideManualPositionController,
  controller_interface::ControllerInterface)
