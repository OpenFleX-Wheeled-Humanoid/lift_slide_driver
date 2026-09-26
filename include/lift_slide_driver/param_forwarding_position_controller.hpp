#pragma once

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lift_slide_driver/param_forwarding_controller_base.hpp"

namespace lift_slide_driver
{

class ParamForwardingPositionController : public ParamForwardingControllerBase
{
public:
  ParamForwardingPositionController() = default;
  ~ParamForwardingPositionController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::string interface_type() const override
  {
    return hardware_interface::HW_IF_POSITION;
  }

  std::string controller_name_hint() const override
  {
    return "lift_position_controller";
  }
};

}  // namespace lift_slide_driver
