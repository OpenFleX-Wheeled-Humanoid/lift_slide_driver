#include "lift_slide_driver/param_forwarding_position_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace lift_slide_driver
{

controller_interface::InterfaceConfiguration
ParamForwardingPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  return configuration;
}

controller_interface::InterfaceConfiguration
ParamForwardingPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::NONE;
  return configuration;
}

}  // namespace lift_slide_driver

PLUGINLIB_EXPORT_CLASS(
  lift_slide_driver::ParamForwardingPositionController,
  controller_interface::ControllerInterface)
