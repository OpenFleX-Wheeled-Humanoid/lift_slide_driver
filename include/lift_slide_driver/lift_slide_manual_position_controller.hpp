#pragma once

#include <memory>
#include <limits>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace lift_slide_driver
{

class LiftSlideManualPositionController : public controller_interface::ControllerInterface
{
public:
  LiftSlideManualPositionController() = default;
  ~LiftSlideManualPositionController() override = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class State
  {
    IDLE,
    STEP_ACTIVE,
    JOG_ACTIVE,
    LIMIT_HOLD,
  };

  struct CommandRequest
  {
    enum class Type { NONE, STEP, POSITION, JOG_START, JOG_STOP, STOP };

    Type type{Type::NONE};
    int direction{0};
    double step_m{0.0};
    double position_m{0.0};
    double speed_mps{0.0};
  };

  bool resolve_joint_name();
  double get_state_value(const std::string & interface_name) const;
  void set_command_value(const std::string & interface_name, double value);
  double clamp(double value, double lower, double upper) const;
  double user_lower_limit() const;
  double user_upper_limit() const;
  bool can_move(int direction, std::string & reason) const;
  void publish_reference(double position, double speed_mps);
  void trigger_manual_halt();
  void trigger_limit_stop();
  void sync_reference_to_actual();
  double approach(double current, double target, double max_delta) const;
  bool update_reference(double dt);
  void monitor_position_motion();
  double jog_position_target() const;
  bool should_publish_reference(double position) const;
  void handle_step(const CommandRequest & request);
  void handle_position(const CommandRequest & request);
  void handle_jog_start(const CommandRequest & request, const rclcpp::Time & time);
  void handle_jog_stop();

  std::string joint_name_{"lift_joint"};

  State state_{State::IDLE};
  int active_direction_{0};
  bool inhibit_jog_until_stop_{false};
  double reference_position_{0.0};
  double reference_velocity_{0.0};
  double target_position_{0.0};
  double target_speed_mps_{0.01};
  double last_published_reference_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Time last_update_time_;

  double min_position_m_{-0.750};
  double max_position_m_{0.400};
  double lower_switch_position_m_{0.000};
  double home_switch_position_m_{0.650};
  double upper_switch_position_m_{0.950};
  double max_velocity_mps_{0.10};
  double max_acceleration_mps2_{0.08};
  double max_deceleration_mps2_{0.12};
  double position_command_min_delta_m_{0.001};
  double target_tolerance_m_{0.0005};
  double velocity_tolerance_mps_{0.0005};
  double jog_target_lookahead_time_s_{0.25};
  double jog_min_target_lookahead_m_{0.010};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr step_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr jog_command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr profile_speed_pub_;

  realtime_tools::RealtimeBuffer<CommandRequest> request_buffer_;
};

}  // namespace lift_slide_driver
