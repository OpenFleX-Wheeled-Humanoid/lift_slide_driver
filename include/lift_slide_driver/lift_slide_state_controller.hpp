#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "lift_slide_msgs/msg/motor_status.hpp"

namespace lift_slide_driver
{

class LiftSlideStateController : public controller_interface::ControllerInterface
{
public:
  LiftSlideStateController() = default;
  ~LiftSlideStateController() override = default;

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
  // Helper to read a state interface by name, returns 0.0 if not found
  double get_state(const std::string & interface_name) const;

  // Helper to set a command interface by name
  void set_command(const std::string & interface_name, double value);

  // Convert enum integers to string representations for panel compatibility
  static std::string homing_state_to_string(int state);
  static std::string homing_detail_to_string(int detail);
  static std::string cia402_state_to_string(int state);
  static std::string mode_to_string(int mode);

  std::string joint_name_{"lift_joint"};

  // Publishers
  rclcpp::Publisher<lift_slide_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr homing_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr limit_switch_state_pub_;

  // Service servers
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_homing_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr quick_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_position_srv_;

  // Subscribers for command inputs
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr profile_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr profile_speed_panel_sub_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Cached previous values for change detection
  int prev_homing_state_{-1};
  int prev_homing_detail_{-1};
  double prev_upper_limit_{-1.0};
  double prev_home_switch_{-1.0};
  double prev_lower_limit_{-1.0};
  double prev_digital_inputs_raw_{-1.0};

  // Publish throttle: motor_status at ~20Hz instead of 100Hz
  int update_cycle_count_{0};
  static constexpr int kMotorStatusPublishDivisor = 5;  // 100Hz / 5 = 20Hz
};

}  // namespace lift_slide_driver
