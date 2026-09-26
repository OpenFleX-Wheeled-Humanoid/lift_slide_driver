#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace lift_slide_driver
{

class ParamForwardingControllerBase : public controller_interface::ControllerInterface
{
public:
  ParamForwardingControllerBase() = default;
  ~ParamForwardingControllerBase() override = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  virtual std::string interface_type() const = 0;
  virtual std::string controller_name_hint() const = 0;
  virtual controller_interface::CallbackReturn on_deactivate_impl(
    const rclcpp_lifecycle::State & previous_state);

  bool resolve_joint_names();

  std::string controller_name_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> fallback_joints_{"lift_joint"};
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    command_interfaces_ref_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>>
    command_buffer_;
};

}  // namespace lift_slide_driver
