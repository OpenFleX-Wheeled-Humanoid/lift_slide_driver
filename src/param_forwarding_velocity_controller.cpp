#include "lift_slide_driver/param_forwarding_velocity_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace lift_slide_driver
{

controller_interface::InterfaceConfiguration
ParamForwardingVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return configuration;
}

controller_interface::InterfaceConfiguration
ParamForwardingVelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::NONE;
  return configuration;
}

controller_interface::CallbackReturn ParamForwardingVelocityController::on_deactivate_impl(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & command_interface : command_interfaces_ref_) {
    command_interface.get().set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace lift_slide_driver

PLUGINLIB_EXPORT_CLASS(
  lift_slide_driver::ParamForwardingVelocityController,
  controller_interface::ControllerInterface)
