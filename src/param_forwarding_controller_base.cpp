#include "lift_slide_driver/param_forwarding_controller_base.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/parameter_client.hpp"

namespace lift_slide_driver
{

controller_interface::CallbackReturn ParamForwardingControllerBase::on_init()
{
  try {
    controller_name_ = controller_name_hint();
    if (controller_name_.empty()) {
      controller_name_ = get_node() ? get_node()->get_name() : std::string{};
    }
    if (controller_name_.empty() || controller_name_ == "controller_manager") {
      controller_name_ = controller_name_hint();
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool ParamForwardingControllerBase::resolve_joint_names()
{
  joint_names_.clear();

  try {
    if (get_node()->has_parameter("joints")) {
      joint_names_ = get_node()->get_parameter("joints").as_string_array();
      if (!joint_names_.empty()) {
        RCLCPP_INFO(
          get_node()->get_logger(), "Resolved joints from controller namespace: %zu", joint_names_.size());
        return true;
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Controller namespace joints lookup failed: %s", exception.what());
  }

  try {
    const std::string parameter_name = controller_name_ + ".joints";
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      get_node(), "/controller_manager");
    if (parameters_client->wait_for_service(std::chrono::seconds(2))) {
      if (parameters_client->has_parameter(parameter_name)) {
        const auto parameters = parameters_client->get_parameters({parameter_name});
        if (!parameters.empty() &&
          parameters[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
          joint_names_ = parameters[0].as_string_array();
          if (!joint_names_.empty()) {
            RCLCPP_INFO(
              get_node()->get_logger(), "Resolved joints from /controller_manager: %zu", joint_names_.size());
            return true;
          }
        }
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_DEBUG(get_node()->get_logger(), "controller_manager joints lookup failed: %s", exception.what());
  }

  joint_names_ = fallback_joints_;
  RCLCPP_WARN(
    get_node()->get_logger(),
    "Using fallback joints (%zu): %s",
    joint_names_.size(),
    joint_names_.empty() ? "" : joint_names_.front().c_str());
  return !joint_names_.empty();
}

controller_interface::CallbackReturn ParamForwardingControllerBase::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  controller_name_ = controller_name_hint();
  if (controller_name_.empty()) {
    controller_name_ = get_node() ? get_node()->get_name() : std::string{};
  }
  if (controller_name_.empty() || controller_name_ == "controller_manager") {
    controller_name_ = controller_name_hint();
  }

  if (!resolve_joint_names() || joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to resolve joints for %s", controller_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!get_node()->has_parameter("command_topic")) {
    get_node()->declare_parameter<std::string>("command_topic", "");
  }

  std::string topic_name = get_node()->get_parameter("command_topic").as_string();
  if (topic_name.empty()) {
    topic_name = "/" + controller_name_ + "/commands";
  }

  command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic_name,
    rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      command_buffer_.writeFromNonRT(msg);
    });
  RCLCPP_INFO(get_node()->get_logger(), "Subscribed command topic: %s", topic_name.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ParamForwardingControllerBase::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interfaces_ref_.clear();
  const auto expected_interface = interface_type();

  for (const auto & joint_name : joint_names_) {
    auto found = std::find_if(
      command_interfaces_.begin(),
      command_interfaces_.end(),
      [&](const auto & interface) {
        return interface.get_prefix_name() == joint_name &&
               interface.get_interface_name() == expected_interface;
      });

    if (found == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Missing command interface for %s/%s",
        joint_name.c_str(),
        expected_interface.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    command_interfaces_ref_.emplace_back(*found);
  }

  command_buffer_.writeFromNonRT(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ParamForwardingControllerBase::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto result = on_deactivate_impl(previous_state);
  command_interfaces_ref_.clear();
  return result;
}

controller_interface::CallbackReturn ParamForwardingControllerBase::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interfaces_ref_.clear();
  joint_names_.clear();
  command_sub_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ParamForwardingControllerBase::on_deactivate_impl(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ParamForwardingControllerBase::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto command_msg = command_buffer_.readFromRT();
  if (!command_msg || !(*command_msg)) {
    return controller_interface::return_type::OK;
  }

  const auto & commands = (*command_msg)->data;
  if (commands.size() != command_interfaces_ref_.size()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Command size (%zu) != expected interfaces (%zu)",
      commands.size(),
      command_interfaces_ref_.size());
    return controller_interface::return_type::OK;
  }

  for (size_t index = 0; index < commands.size(); ++index) {
    command_interfaces_ref_[index].get().set_value(commands[index]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace lift_slide_driver
