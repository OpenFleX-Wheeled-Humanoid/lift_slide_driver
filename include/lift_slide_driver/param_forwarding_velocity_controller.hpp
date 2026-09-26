#pragma once

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lift_slide_driver/param_forwarding_controller_base.hpp"

namespace lift_slide_driver
{

class ParamForwardingVelocityController : public ParamForwardingControllerBase
{
public:
  ParamForwardingVelocityController() = default;
  ~ParamForwardingVelocityController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::string interface_type() const override
  {
    return hardware_interface::HW_IF_VELOCITY;
  }

  std::string controller_name_hint() const override
  {
    return "lift_velocity_controller";
  }

  controller_interface::CallbackReturn on_deactivate_impl(
    const rclcpp_lifecycle::State & previous_state) override;
};

}  // namespace lift_slide_driver
