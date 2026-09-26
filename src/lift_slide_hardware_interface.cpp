#include "lift_slide_driver/lift_slide_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

// Shorthand for homing state/detail enums
using HSC = lift_slide_driver::HomingStateCode;
using HD = lift_slide_driver::HomingDetail;

namespace
{
using Ld2Object = openflex_can::Ld2CanopenDrive::Object;
constexpr uint16_t OBJ_CONTROLWORD = openflex_can::Ld2CanopenDrive::index(Ld2Object::CONTROLWORD);
constexpr uint16_t OBJ_STATUSWORD = openflex_can::Ld2CanopenDrive::index(Ld2Object::STATUSWORD);
constexpr uint16_t OBJ_MODE_OF_OPERATION = openflex_can::Ld2CanopenDrive::index(Ld2Object::MODE_OF_OPERATION);
constexpr uint16_t OBJ_MODE_OF_OPERATION_DISPLAY = openflex_can::Ld2CanopenDrive::index(Ld2Object::MODE_DISPLAY);
constexpr uint16_t OBJ_POSITION_ACTUAL = openflex_can::Ld2CanopenDrive::index(Ld2Object::POSITION_ACTUAL);
constexpr uint16_t OBJ_VELOCITY_ACTUAL = openflex_can::Ld2CanopenDrive::index(Ld2Object::VELOCITY_ACTUAL);
constexpr uint16_t OBJ_TARGET_POSITION = openflex_can::Ld2CanopenDrive::index(Ld2Object::TARGET_POSITION);
constexpr uint16_t OBJ_TARGET_VELOCITY = openflex_can::Ld2CanopenDrive::index(Ld2Object::TARGET_VELOCITY);
constexpr uint16_t OBJ_PROFILE_VELOCITY = openflex_can::Ld2CanopenDrive::index(Ld2Object::PROFILE_VELOCITY);
constexpr uint16_t OBJ_DIGITAL_INPUTS = openflex_can::Ld2CanopenDrive::index(Ld2Object::DIGITAL_INPUTS);
constexpr uint16_t OBJ_PROFILE_ACCELERATION = openflex_can::Ld2CanopenDrive::index(Ld2Object::PROFILE_ACCELERATION);
constexpr uint16_t OBJ_PROFILE_DECELERATION = openflex_can::Ld2CanopenDrive::index(Ld2Object::PROFILE_DECELERATION);
constexpr uint16_t OBJ_HOMING_METHOD = openflex_can::Ld2CanopenDrive::index(Ld2Object::HOMING_METHOD);
constexpr uint16_t OBJ_HOMING_SPEEDS = openflex_can::Ld2CanopenDrive::index(Ld2Object::HOMING_SPEEDS);
constexpr uint16_t OBJ_HOMING_ACCEL = openflex_can::Ld2CanopenDrive::index(Ld2Object::HOMING_ACCELERATION);
constexpr uint16_t OBJ_ABSOLUTE_ENCODER = openflex_can::Ld2CanopenDrive::index(Ld2Object::ABSOLUTE_ENCODER);
constexpr uint16_t OBJ_CLEAR_POSITION = openflex_can::Ld2CanopenDrive::index(Ld2Object::CLEAR_POSITION);
constexpr uint16_t OBJ_ACCEL_TIME = openflex_can::Ld2CanopenDrive::index(Ld2Object::ACCELERATION_TIME);
constexpr uint16_t OBJ_DECEL_TIME = openflex_can::Ld2CanopenDrive::index(Ld2Object::DECELERATION_TIME);
constexpr uint16_t OBJ_ERROR_CODE = openflex_can::Ld2CanopenDrive::index(Ld2Object::ERROR_CODE);

constexpr uint16_t CMD_SHUTDOWN = static_cast<uint16_t>(openflex_can::Ld2CanopenDrive::SHUTDOWN);
constexpr uint16_t CMD_SWITCH_ON = static_cast<uint16_t>(openflex_can::Ld2CanopenDrive::SWITCH_ON);
constexpr uint16_t CMD_ENABLE_OPERATION = static_cast<uint16_t>(openflex_can::Ld2CanopenDrive::ENABLE_OPERATION);
constexpr uint16_t CMD_QUICK_STOP = static_cast<uint16_t>(openflex_can::Ld2CanopenDrive::QUICK_STOP);
constexpr uint16_t CMD_DISABLE_VOLTAGE = static_cast<uint16_t>(openflex_can::Ld2CanopenDrive::DISABLE_VOLTAGE);
constexpr uint16_t CMD_FAULT_RESET = static_cast<uint16_t>(openflex_can::Ld2CanopenDrive::FAULT_RESET);

constexpr int8_t MODE_NO_MODE = static_cast<int8_t>(openflex_can::Ld2CanopenDrive::Mode::NO_MODE);
constexpr int8_t MODE_POSITION = static_cast<int8_t>(openflex_can::Ld2CanopenDrive::Mode::POSITION);
constexpr int8_t MODE_VELOCITY = static_cast<int8_t>(openflex_can::Ld2CanopenDrive::Mode::VELOCITY);
constexpr int8_t MODE_HOMING = static_cast<int8_t>(openflex_can::Ld2CanopenDrive::Mode::HOMING);

constexpr uint16_t CW_NEW_SETPOINT = static_cast<uint16_t>(openflex_can::Ld2CanopenDrive::NEW_SETPOINT);
constexpr uint16_t CW_CHANGE_IMMEDIATELY = static_cast<uint16_t>(openflex_can::Ld2CanopenDrive::CHANGE_IMMEDIATELY);
}  // namespace

namespace lift_slide_driver
{

LiftSlideHardwareInterface::~LiftSlideHardwareInterface()
{
  shutdown_requested_.store(true);
  homing_in_progress_.store(false);
  shutdown_detached_threads();
  stop_async_read_thread();
  if (homing_thread_.joinable()) {
    homing_thread_.join();
  }
  if (!use_fake_hardware_ && canopen_master_.is_open()) {
    (void)send_rpdo1(CMD_DISABLE_VOLTAGE, 0, MODE_VELOCITY);
  }
  close_can_socket();
  drive_enabled_.store(false);
}

hardware_interface::CallbackReturn LiftSlideHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get_param = [&](const std::string & key) -> std::string {
    auto iter = info_.hardware_parameters.find(key);
    return iter == info_.hardware_parameters.end() ? "" : iter->second;
  };

  can_interface_ = get_param("can_interface").empty() ? "can3" : get_param("can_interface");
  node_id_ = parse_int(get_param("node_id"), 16);
  canopen_master_.set_node_id(static_cast<uint8_t>(node_id_));
  use_fake_hardware_ = parse_bool(get_param("use_fake_hardware"), false);
  counts_per_meter_ = parse_double(get_param("counts_per_meter"), 2000000.0);
  profile_acceleration_.store(parse_int(get_param("profile_acceleration"), 50000));
  profile_deceleration_.store(parse_int(get_param("profile_deceleration"), 50000));
  invert_command_ = parse_bool(get_param("invert_command"), true);
  invert_feedback_ = parse_bool(get_param("invert_feedback"), true);
  min_position_m_ = parse_double(get_param("min_position_m"), -0.750);
  max_position_m_ = parse_double(get_param("max_position_m"), 0.400);
  max_velocity_mps_ = parse_double(get_param("max_velocity_mps"), 0.10);
  command_timeout_sec_ = parse_double(get_param("command_timeout_sec"), 0.5);
  sdo_timeout_sec_ = parse_double(get_param("sdo_timeout_sec"), 0.2);
  feedback_poll_rate_hz_ = parse_double(get_param("feedback_poll_rate_hz"), 20.0);
  homing_speed_mps_ = parse_double(get_param("homing_speed_mps"), 0.010);
  homing_low_speed_ratio_ = parse_double(get_param("homing_low_speed_ratio"), 0.2);
  homing_method_ = parse_int(get_param("homing_method"), 27);
  homing_acceleration_ = parse_int(get_param("homing_acceleration"), 50000);
  homing_timeout_sec_ = parse_double(get_param("homing_timeout_sec"), 60.0);
  homing_configure_di_ = parse_bool(get_param("homing_configure_di"), false);
  lower_switch_position_m_ = parse_double(get_param("lower_switch_position_m"), 0.000);
  home_switch_position_m_ = parse_double(get_param("home_switch_position_m"), 0.650);
  upper_switch_position_m_ = parse_double(get_param("upper_switch_position_m"), 0.950);
  switch_position_tolerance_m_ = parse_double(get_param("switch_position_tolerance_m"), 0.008);
  di_active_low_ = parse_bool(get_param("di_active_low"), true);
  di6_not_func_ = parse_int(get_param("di6_not_func"),
    parse_int(get_param("di3_not_func"), 0x02));
  di4_homing_func_ = parse_int(get_param("di4_homing_func"), 0x16);
  di5_pot_func_ = parse_int(get_param("di5_pot_func"), 0x01);
  home_di_channel_ = std::clamp(parse_int(get_param("home_di_channel"), 4), 1, 6);
  pot_di_channel_ = std::clamp(parse_int(get_param("pot_di_channel"), 5), 1, 6);
  not_di_channel_ = std::clamp(parse_int(get_param("not_di_channel"), 6), 1, 6);

  if (home_di_channel_ == pot_di_channel_ ||
      home_di_channel_ == not_di_channel_ ||
      pot_di_channel_ == not_di_channel_)
  {
    RCLCPP_ERROR(
      logger_,
      "DI channel mapping is invalid: home=%d pot=%d not=%d (channels must be unique)",
      home_di_channel_,
      pot_di_channel_,
      not_di_channel_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(logger_, "Expected 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & joint = info_.joints[0];
  bool has_velocity_command = false;
  for (const auto & interface : joint.command_interfaces) {
    if (interface.name == hardware_interface::HW_IF_VELOCITY) {
      has_velocity_command = true;
      break;
    }
  }
  if (!has_velocity_command) {
    RCLCPP_ERROR(logger_, "Joint '%s' must expose velocity command interface", joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  bool has_position_state = false;
  bool has_velocity_state = false;
  for (const auto & interface : joint.state_interfaces) {
    has_position_state = has_position_state || interface.name == hardware_interface::HW_IF_POSITION;
    has_velocity_state = has_velocity_state || interface.name == hardware_interface::HW_IF_VELOCITY;
  }
  if (!has_position_state || !has_velocity_state) {
    RCLCPP_ERROR(
      logger_,
      "Joint '%s' must expose position and velocity state interfaces",
      joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_position_ = 0.0;
  hw_velocity_ = 0.0;
  hw_command_velocity_ = 0.0;
  hw_position_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_hold_position_cmd_ = 0.0;
  last_sent_velocity_command_ = 0.0;
  last_sent_position_command_ = std::numeric_limits<double>::quiet_NaN();
  last_target_velocity_counts_per_sec_.store(std::numeric_limits<int32_t>::min());
  last_statusword_.store(0);
  drive_enabled_.store(false);
  is_enabled_.store(false);
  is_fault_.store(false);
  is_target_reached_.store(false);
  cia402_state_.store(static_cast<int>(Cia402State::UNKNOWN));
  current_mode_of_operation_.store(MODE_POSITION);
  active_mode_.store(MODE_POSITION);
  position_move_in_progress_.store(false);
  profile_speed_mps_.store(homing_speed_mps_);
  position_offset_m_.store(0.0);
  alive_flag_ = std::make_shared<std::atomic<bool>>(true);
  active_detached_count_ = std::make_shared<std::atomic<int>>(0);
  cmd_dispatch_in_progress_ = std::make_shared<std::atomic<bool>>(false);
  async_read_running_.store(false);
  homing_in_progress_.store(false);
  homing_complete_.store(false);
  homing_error_.store(false);
  quick_stop_active_.store(false);
  motion_ready_.store(false);
  reference_valid_.store(false);
  encoder_reference_lost_.store(false);
  feedback_poll_paused_.store(false);
  shutdown_requested_.store(false);
  last_digital_inputs_raw_ = 0;
  limit_switch_state_valid_ = false;
  cia_not_bit0_ = false;
  cia_pot_bit1_ = false;
  cia_home_bit2_ = false;
  si1_bit4_ = false;
  si2_bit5_ = false;
  si3_bit6_ = false;
  si4_bit7_ = false;
  si5_bit8_ = false;
  si6_bit9_ = false;
  di1_bit16_ = false;
  di2_bit17_ = false;
  di3_bit18_ = false;
  di4_bit19_ = false;
  di5_bit20_ = false;
  di6_bit21_ = false;
  di1_bit24_ = false;
  di2_bit25_ = false;
  di3_bit26_ = false;
  di4_bit27_ = false;
  di5_bit28_ = false;
  di6_bit29_ = false;
  homing_di6_config_applied_ = false;
  homing_di4_config_applied_ = false;
  homing_di5_config_applied_ = false;
  feedback_cycle_counter_ = 0;

  calibration_file_path_ = get_param("calibration_file");
  if (calibration_file_path_.empty()) {
    RCLCPP_WARN(logger_, "No calibration_file parameter supplied; zero persistence is disabled");
  }

  auto now = std::chrono::steady_clock::now();
  last_feedback_time_ = now;
  last_read_time_ = now;
  last_command_update_time_ = now;

  RCLCPP_INFO(
    logger_,
    "LiftSlideHardware initialized: fake=%s can=%s node_id=%d homing_method=%d homing_speed=%.4f home_di=%d pot_di=%d not_di=%d di_active_low=%s",
    use_fake_hardware_ ? "true" : "false",
    can_interface_.c_str(),
    node_id_,
    homing_method_,
    homing_speed_mps_,
    home_di_channel_,
    pot_di_channel_,
    not_di_channel_,
    di_active_low_ ? "true" : "false");
  RCLCPP_INFO(
    logger_,
    "LiftSlide soft limits=[%.3f, %.3f]; switch positions are sensor/homing metadata only: lower=%.3f home=%.3f upper=%.3f",
    min_position_m_,
    max_position_m_,
    lower_switch_position_m_,
    home_switch_position_m_,
    upper_switch_position_m_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LiftSlideHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  shutdown_requested_.store(false);

  if (use_fake_hardware_) {
    drive_enabled_.store(true);
    quick_stop_active_.store(false);
    set_homing_state(static_cast<int>(HSC::IDLE), static_cast<int>(HD::FAKE_HARDWARE));
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!init_can_socket()) {
    RCLCPP_ERROR(logger_, "Failed to initialize CAN socket on interface %s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!send_nmt(0x01)) {
    RCLCPP_WARN(logger_, "Failed to send NMT start");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  bool startup_ready = true;
  bool encoder_recovery_performed = false;
  uint32_t startup_error_code = 0U;
  if (canopen_master_.read_error_code(
      startup_error_code, static_cast<int>(sdo_timeout_sec_ * 1000.0)) &&
    (startup_error_code & 0xFFFFU) == 0x7325U)
  {
    RCLCPP_WARN(
      logger_,
      "LD2 reports Er153 (603Fh=0x%04X); recovering a temporary encoder reference",
      static_cast<unsigned int>(startup_error_code & 0xFFFFU));
    startup_ready = recover_encoder_reference_loss();
    encoder_recovery_performed = startup_ready;
  } else {
    if (!reset_fault()) {
      RCLCPP_WARN(logger_, "Fault reset failed during configure");
    }
    if (!enable_drive()) {
      RCLCPP_WARN(
        logger_,
        "Enable drive failed during configure; starting in disconnected/disabled state");
      startup_ready = false;
    }

    if (startup_ready && !configure_drive()) {
      RCLCPP_WARN(
        logger_,
        "Drive configure failed during startup; controller will remain available for later enable");
      startup_ready = false;
    }

    // Re-assert EnableOperation after mode configuration.
    if (startup_ready && !sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false)) {
      RCLCPP_WARN(logger_, "Re-enable operation after configure failed");
      startup_ready = false;
    }
  }

  drive_enabled_.store(startup_ready);
  motion_ready_.store(startup_ready);
  quick_stop_active_.store(false);
  if (startup_ready && !read_feedback_once()) {
    RCLCPP_WARN(logger_, "Initial feedback read failed");
  }
  if (startup_ready && homing_configure_di_) {
    if (!configure_homing_di_mapping()) {
      RCLCPP_WARN(logger_, "Startup DI mapping failed; limit status may be unavailable");
    } else {
      // Keep all DI mappings (HOME/POT/NOT) active so CiA bits always
      // reflect the physical switches for real-time lamp updates.
      if (!read_feedback_once()) {
        RCLCPP_WARN(logger_, "Feedback refresh failed after startup DI setup");
      }
    }
  }
  set_homing_state(static_cast<int>(HSC::IDLE),
    startup_ready ? static_cast<int>(HD::READY) : static_cast<int>(HD::SERVICE_READY));

  // Try to restore zero-point calibration from previous session
  if (startup_ready && !encoder_recovery_performed && !homing_complete_.load()) {
    if (load_calibration()) {
      // Re-read feedback with the restored offset applied
      if (read_feedback_once()) {
        hw_position_ = async_position_.load() - position_offset_m_.load();
      }
      set_homing_state(
        reference_valid_.load() ? static_cast<int>(HSC::COMPLETED) : static_cast<int>(HSC::IDLE),
        static_cast<int>(HD::CALIBRATION_RESTORED));
    }
  }
  if (encoder_recovery_performed) {
    set_homing_state(static_cast<int>(HSC::IDLE), static_cast<int>(HD::SERVICE_READY));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LiftSlideHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  shutdown_requested_.store(false);
  // Recreate alive_flag_ so old detached threads (from previous deactivate)
  // see false and exit safely, while new threads get the fresh flag
  alive_flag_ = std::make_shared<std::atomic<bool>>(true);
  hw_command_velocity_ = 0.0;
  hw_position_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_hold_position_cmd_ = 0.0;
  last_sent_velocity_command_ = 0.0;
  last_sent_position_command_ = std::numeric_limits<double>::quiet_NaN();
  last_target_velocity_counts_per_sec_.store(std::numeric_limits<int32_t>::min());
  quick_stop_active_.store(false);
  position_move_in_progress_.store(false);
  cmd_dispatch_in_progress_ = std::make_shared<std::atomic<bool>>(false);
  feedback_poll_paused_.store(false);
  homing_in_progress_.store(false);
  last_command_update_time_ = std::chrono::steady_clock::now();

  if (!use_fake_hardware_) {
    // Do not start the async SDO polling thread until the CiA402 enable sequence
    // finishes. Otherwise activation can contend with background polling on the
    // same CAN/SDO channel and make controller_manager appear stuck.
    const bool enabled = enable_drive();
    start_async_read_thread();
    if (!enabled) {
      RCLCPP_WARN(
        logger_,
        "Enable drive failed on activate; continuing with drive disabled until /lift_slide_driver/enable succeeds");
      drive_enabled_.store(false);
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    drive_enabled_.store(true);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LiftSlideHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  shutdown_requested_.store(true);
  hw_command_velocity_ = 0.0;
  hw_position_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_hold_position_cmd_ = 0.0;
  last_sent_velocity_command_ = 0.0;
  last_sent_position_command_ = std::numeric_limits<double>::quiet_NaN();
  last_target_velocity_counts_per_sec_.store(std::numeric_limits<int32_t>::min());
  homing_in_progress_.store(false);
  shutdown_detached_threads();
  stop_async_read_thread();
  join_homing_thread();

  if (!use_fake_hardware_) {
    motor_power_off_sequence();
    drive_enabled_.store(false);
    quick_stop_active_.store(false);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LiftSlideHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  shutdown_requested_.store(true);
  homing_in_progress_.store(false);
  shutdown_detached_threads();
  stop_async_read_thread();
  join_homing_thread();
  close_can_socket();
  drive_enabled_.store(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LiftSlideHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  shutdown_requested_.store(true);
  homing_in_progress_.store(false);
  shutdown_detached_threads();
  stop_async_read_thread();
  join_homing_thread();
  if (!use_fake_hardware_ && canopen_master_.is_open()) {
    motor_power_off_sequence();
  }
  close_can_socket();
  drive_enabled_.store(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LiftSlideHardwareInterface::on_error(
  const rclcpp_lifecycle::State &)
{
  shutdown_requested_.store(true);
  homing_in_progress_.store(false);
  shutdown_detached_threads();
  stop_async_read_thread();
  join_homing_thread();

  if (!use_fake_hardware_ && canopen_master_.is_open()) {
    (void)send_rpdo1(CMD_QUICK_STOP, 0, MODE_VELOCITY);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    (void)send_rpdo1(CMD_DISABLE_VOLTAGE, 0, MODE_VELOCITY);
  }

  close_can_socket();
  drive_enabled_.store(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
LiftSlideHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  const auto & jn = info_.joints[0].name;
  state_interfaces.emplace_back(jn, hardware_interface::HW_IF_POSITION, &hw_position_);
  state_interfaces.emplace_back(jn, hardware_interface::HW_IF_VELOCITY, &hw_velocity_);
  state_interfaces.emplace_back(jn, "cia402_state", &hw_cia402_state_);
  state_interfaces.emplace_back(jn, "statusword", &hw_statusword_);
  state_interfaces.emplace_back(jn, "is_enabled", &hw_is_enabled_);
  state_interfaces.emplace_back(jn, "is_fault", &hw_is_fault_);
  state_interfaces.emplace_back(jn, "is_target_reached", &hw_is_target_reached_);
  state_interfaces.emplace_back(jn, "mode_of_operation", &hw_mode_of_operation_);
  state_interfaces.emplace_back(jn, "homing_state", &hw_homing_state_);
  state_interfaces.emplace_back(jn, "homing_detail", &hw_homing_detail_);
  state_interfaces.emplace_back(jn, "homing_complete", &hw_homing_complete_);
  state_interfaces.emplace_back(jn, "is_homing", &hw_is_homing_);
  state_interfaces.emplace_back(jn, "upper_limit_switch", &hw_upper_limit_switch_);
  state_interfaces.emplace_back(jn, "home_switch", &hw_home_switch_);
  state_interfaces.emplace_back(jn, "lower_limit_switch", &hw_lower_limit_switch_);
  state_interfaces.emplace_back(jn, "limit_switch_valid", &hw_limit_switch_valid_);
  state_interfaces.emplace_back(jn, "digital_inputs_raw", &hw_digital_inputs_raw_);
  state_interfaces.emplace_back(jn, "physical_position", &hw_physical_position_);
  state_interfaces.emplace_back(jn, "profile_speed_mps", &hw_profile_speed_mps_);
  state_interfaces.emplace_back(jn, "profile_accel_mps2", &hw_profile_accel_mps2_);
  state_interfaces.emplace_back(jn, "profile_decel_mps2", &hw_profile_decel_mps2_);
  state_interfaces.emplace_back(jn, "drive_enabled", &hw_drive_enabled_);
  state_interfaces.emplace_back(jn, "quick_stop_active", &hw_quick_stop_active_);
  state_interfaces.emplace_back(jn, "motion_ready", &hw_motion_ready_);
  state_interfaces.emplace_back(jn, "reference_valid", &hw_reference_valid_);
  state_interfaces.emplace_back(jn, "encoder_reference_lost", &hw_encoder_reference_lost_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LiftSlideHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  const auto & jn = info_.joints[0].name;
  command_interfaces.emplace_back(jn, hardware_interface::HW_IF_VELOCITY, &hw_command_velocity_);
  command_interfaces.emplace_back(jn, hardware_interface::HW_IF_POSITION, &hw_position_command_);
  command_interfaces.emplace_back(jn, "enable_cmd", &hw_enable_cmd_);
  command_interfaces.emplace_back(jn, "quick_stop_cmd", &hw_quick_stop_cmd_);
  command_interfaces.emplace_back(jn, "hold_position_cmd", &hw_hold_position_cmd_);
  command_interfaces.emplace_back(jn, "reset_fault_cmd", &hw_reset_fault_cmd_);
  command_interfaces.emplace_back(jn, "full_reset_cmd", &hw_full_reset_cmd_);
  command_interfaces.emplace_back(jn, "homing_cmd", &hw_homing_cmd_);
  command_interfaces.emplace_back(jn, "set_mode_cmd", &hw_set_mode_cmd_);
  command_interfaces.emplace_back(jn, "profile_speed_cmd", &hw_profile_speed_cmd_);
  command_interfaces.emplace_back(jn, "accel_time_cmd", &hw_accel_time_cmd_);
  command_interfaces.emplace_back(jn, "decel_time_cmd", &hw_decel_time_cmd_);
  command_interfaces.emplace_back(jn, "manual_halt_cmd", &hw_manual_halt_cmd_);
  command_interfaces.emplace_back(jn, "manual_limit_stop_cmd", &hw_manual_limit_stop_cmd_);
  return command_interfaces;
}

hardware_interface::return_type LiftSlideHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  const auto now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(now - last_read_time_).count();
  last_read_time_ = now;

  if (use_fake_hardware_) {
    if (!homing_in_progress_.load()) {
      const double applied_velocity = apply_position_limits(clamp(hw_velocity_, -max_velocity_mps_, max_velocity_mps_));
      hw_velocity_ = applied_velocity;
      hw_position_ = clamp(hw_position_ + applied_velocity * dt, min_position_m_, max_position_m_);
    }

    const bool homed = homing_complete_.load();
    const double physical_position = homed ? (hw_position_ + home_switch_position_m_) : hw_position_;
    bool lower_switch = false;
    bool home_switch = false;
    bool upper_switch = false;
    estimate_switch_states(physical_position, lower_switch, home_switch, upper_switch);
    uint32_t raw = 0U;
    const auto set_fake_physical_di = [this, &raw](int channel, bool active) {
        const bool level_high = di_active_low_ ? !active : active;
        const uint32_t bit = 24U + static_cast<uint32_t>(channel - 1);
        if (level_high) {
          raw |= (1U << bit);
        }
      };
    if (upper_switch) {
      raw |= (1U << 0U);
    }
    if (lower_switch) {
      raw |= (1U << 1U);
    }
    if (home_switch) {
      raw |= (1U << 2U);
    }
    // Bits 7-9 (SI4-SI6) must represent physical electrical level,
    // matching what physical_di_channel_level_high() returns.
    // physical_di_channel_active() applies di_active_low_ on top,
    // so we must encode the level the same way as bits 24-29.
    const auto set_fake_si_bit = [this, &raw](int channel, bool active) {
        const bool level_high = di_active_low_ ? !active : active;
        const uint32_t bit = 4U + static_cast<uint32_t>(channel - 1);
        if (level_high) {
          raw |= (1U << bit);
        }
      };
    set_fake_si_bit(home_di_channel_, home_switch);
    set_fake_si_bit(pot_di_channel_, lower_switch);
    set_fake_si_bit(not_di_channel_, upper_switch);
    set_fake_physical_di(not_di_channel_, upper_switch);
    set_fake_physical_di(pot_di_channel_, lower_switch);
    set_fake_physical_di(home_di_channel_, home_switch);
    update_limit_switch_state(raw);
    update_state_interface_values();
    return hardware_interface::return_type::OK;
  }

  if (feedback_poll_paused_.load() || homing_in_progress_.load()) {
    update_state_interface_values();
    return hardware_interface::return_type::OK;
  }

  // Use async read thread data (non-blocking)
  hw_position_ = async_position_.load() - position_offset_m_.load();
  hw_velocity_ = async_velocity_.load();
  last_statusword_.store(async_statusword_.load());
  const uint32_t di_raw = async_digital_inputs_.load();
  if (di_raw != 0 || limit_switch_state_valid_) {
    update_limit_switch_state(di_raw);
  }

  update_state_interface_values();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LiftSlideHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  process_command_interfaces();

  const auto now = std::chrono::steady_clock::now();

  // === Velocity command handling ===
  const double requested_command_mps =
    clamp(hw_command_velocity_, -max_velocity_mps_, max_velocity_mps_);
  const bool velocity_changed =
    std::abs(requested_command_mps - last_sent_velocity_command_) > 1e-9;

  if (velocity_changed) {
    last_sent_velocity_command_ = requested_command_mps;
  }

  // === Position command handling ===
  double pos_cmd = hw_position_command_;
  if (!std::isnan(pos_cmd)) {
    const double clipped_pos_cmd = clamp(pos_cmd, min_position_m_, max_position_m_);
    if (std::abs(clipped_pos_cmd - pos_cmd) > 1e-9) {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *rclcpp::Clock::make_shared(),
        1000,
        "Position target clipped by soft limit: requested=%.4f clipped=%.4f limits=[%.4f, %.4f]",
        pos_cmd,
        clipped_pos_cmd,
        min_position_m_,
        max_position_m_);
      hw_position_command_ = clipped_pos_cmd;
      pos_cmd = clipped_pos_cmd;
    }
  }
  const bool pos_is_valid = !std::isnan(pos_cmd);
  const bool pos_changed = pos_is_valid &&
    (std::isnan(last_sent_position_command_) ||
     std::abs(pos_cmd - last_sent_position_command_) > 1e-6);
  const double position_error = pos_is_valid ? std::abs(pos_cmd - hw_position_) : 0.0;
  bool pos_retrigger = false;
  if (!pos_is_valid || pos_changed || position_error <= 0.005) {
    retrigger_last_error_ = position_error;
    retrigger_last_check_time_ = now;
  } else {
    const double elapsed =
      std::chrono::duration<double>(now - retrigger_last_check_time_).count();
    const bool no_meaningful_progress =
      !std::isnan(retrigger_last_error_) && position_error >= (retrigger_last_error_ - 0.001);
    if (elapsed >= 0.12) {
      pos_retrigger = no_meaningful_progress;
      retrigger_last_error_ = position_error;
      retrigger_last_check_time_ = now;
    }
  }
  const bool should_dispatch_position = pos_changed || pos_retrigger;

  double command_mps = requested_command_mps;

  if (std::abs(requested_command_mps) > 1e-9 || velocity_changed) {
    last_command_update_time_ = now;
  } else {
    const double timeout =
      std::chrono::duration<double>(now - last_command_update_time_).count();
    if (timeout > command_timeout_sec_) {
      command_mps = 0.0;
    }
  }

  command_mps = apply_position_limits(command_mps);
  if (!drive_enabled_.load()) {
    command_mps = 0.0;
  }
  if (homing_in_progress_.load()) {
    command_mps = 0.0;
  }

  if (use_fake_hardware_) {
    // 检查失能状态
    if (!drive_enabled_.load()) {
      hw_velocity_ = 0.0;
      return hardware_interface::return_type::OK;
    }

    if (!homing_in_progress_.load()) {
      if (pos_is_valid) {
        // Simulate position mode: ramp toward target
        const double dir = (pos_cmd > hw_position_) ? 1.0 : -1.0;
        const double speed = std::max(0.001, profile_speed_mps_.load());
        const double remaining = std::abs(pos_cmd - hw_position_);
        if (remaining < 0.001) {
          hw_velocity_ = 0.0;
          last_sent_position_command_ = pos_cmd;
        } else {
          hw_velocity_ = dir * std::min(speed, remaining * 20.0);
        }
      } else {
        hw_velocity_ = command_mps;
      }
    }
    return hardware_interface::return_type::OK;
  }

  if (!drive_enabled_.load()) {
    hw_velocity_ = 0.0;
    last_target_velocity_counts_per_sec_.store(0);
    return hardware_interface::return_type::OK;
  }

  if (feedback_poll_paused_.load() || homing_in_progress_.load()) {
    return hardware_interface::return_type::OK;
  }

  // === Position mode dispatch (via detached thread) ===
  if (ignore_current_position_command_after_hold_.load()) {
    if (pos_is_valid) {
      if (std::isnan(ignored_position_command_after_hold_)) {
        ignored_position_command_after_hold_ = pos_cmd;
      }
      if (std::abs(pos_cmd - ignored_position_command_after_hold_) <= 1e-6) {
        last_sent_position_command_ = pos_cmd;
        RCLCPP_INFO_THROTTLE(
          logger_,
          *rclcpp::Clock::make_shared(),
          1000,
          "[manual] stale position command ignored after hold: target=%.4f",
          pos_cmd);
        return hardware_interface::return_type::OK;
      }
    }
    ignore_current_position_command_after_hold_.store(false);
    ignored_position_command_after_hold_ = std::numeric_limits<double>::quiet_NaN();
  }

  if (should_dispatch_position && pos_is_valid && !cmd_dispatch_in_progress_->load() && !manual_hold_active_.load()) {
    last_sent_position_command_ = pos_cmd;
    const double speed = std::max(0.001, profile_speed_mps_.load());
    auto disp = cmd_dispatch_in_progress_;
    disp->store(true);
    RCLCPP_DEBUG(
      logger_,
      "[manual] position dispatch start: target=%.4f speed=%.4f active_mode=%d retrigger=%d error=%.4f",
      pos_cmd,
      speed,
      static_cast<int>(active_mode_.load()),
      pos_retrigger ? 1 : 0,
      position_error);
    run_detached([this, pos_cmd, speed]() {
      int32_t mode_display_raw = 0;
      if (!sdo_read(OBJ_MODE_OF_OPERATION_DISPLAY, 0x00, mode_display_raw, true)) {
        RCLCPP_ERROR(
          logger_,
          "Failed to read mode of operation display before position move");
        return;
      }

      const int8_t mode_display = static_cast<int8_t>(mode_display_raw & 0xFF);
      if (mode_display != MODE_POSITION) {
        if (!set_mode_of_operation(MODE_POSITION)) {
          RCLCPP_ERROR(
            logger_,
            "Failed to switch lift slide drive into position mode before position move");
          return;
        }

        if (!sdo_read(OBJ_MODE_OF_OPERATION_DISPLAY, 0x00, mode_display_raw, true)) {
          RCLCPP_ERROR(
            logger_,
            "Failed to read mode of operation display after switching to position mode");
          return;
        }

        const int8_t verified_mode_display = static_cast<int8_t>(mode_display_raw & 0xFF);
        if (verified_mode_display != MODE_POSITION) {
          RCLCPP_ERROR(
            logger_,
            "Drive rejected position mode switch: requested=%d actual=%d",
            static_cast<int>(MODE_POSITION),
            static_cast<int>(verified_mode_display));
          return;
        }

        active_mode_.store(MODE_POSITION);
        current_mode_of_operation_.store(MODE_POSITION);
      }
      if (manual_hold_active_.load()) {
        RCLCPP_INFO(
          logger_,
          "[manual] position dispatch aborted by hold: target=%.4f",
          pos_cmd);
        return;
      }
      start_position_move(pos_cmd, speed);
    }, [disp, this, pos_cmd]() {
      disp->store(false);
      RCLCPP_DEBUG(
        logger_,
        "[manual] position dispatch done: target=%.4f",
        pos_cmd);
    });
    return hardware_interface::return_type::OK;
  }

  // === Velocity mode (RPDO, non-blocking) ===
  if (active_mode_.load() == MODE_POSITION && !pos_is_valid && std::abs(command_mps) > 1e-9) {
    // Switch back to velocity mode
    if (!cmd_dispatch_in_progress_->load()) {
      auto disp = cmd_dispatch_in_progress_;
      disp->store(true);
      run_detached([this]() {
        if (restore_velocity_mode()) {
          active_mode_.store(MODE_VELOCITY);
          current_mode_of_operation_.store(MODE_VELOCITY);
        }
      }, [disp]() { disp->store(false); });
    }
    return hardware_interface::return_type::OK;
  }

  int32_t target_counts_per_sec = velocity_mps_to_counts_per_sec(command_mps);
  if (invert_command_) {
    target_counts_per_sec = -target_counts_per_sec;
  }

  if (target_counts_per_sec == last_target_velocity_counts_per_sec_.load()) {
    return hardware_interface::return_type::OK;
  }

  if (!send_rpdo1(CMD_ENABLE_OPERATION, target_counts_per_sec, MODE_VELOCITY)) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *rclcpp::Clock::make_shared(),
      1000,
      "RPDO send target velocity failed");
    return hardware_interface::return_type::ERROR;
  }

  last_target_velocity_counts_per_sec_.store(target_counts_per_sec);
  return hardware_interface::return_type::OK;
}

void LiftSlideHardwareInterface::set_homing_state(int state, int detail)
{
  homing_state_atomic_.store(state);
  homing_detail_atomic_.store(detail);
}

void LiftSlideHardwareInterface::update_state_interface_values()
{
  hw_cia402_state_ = static_cast<double>(cia402_state_.load());
  hw_statusword_ = static_cast<double>(async_statusword_.load());
  hw_is_enabled_ = is_enabled_.load() ? 1.0 : 0.0;
  hw_is_fault_ = is_fault_.load() ? 1.0 : 0.0;
  hw_is_target_reached_ = is_target_reached_.load() ? 1.0 : 0.0;
  hw_mode_of_operation_ = static_cast<double>(current_mode_of_operation_.load());
  hw_homing_state_ = static_cast<double>(homing_state_atomic_.load());
  hw_homing_detail_ = static_cast<double>(homing_detail_atomic_.load());
  hw_homing_complete_ = homing_complete_.load() ? 1.0 : 0.0;
  hw_is_homing_ = homing_in_progress_.load() ? 1.0 : 0.0;
  hw_upper_limit_switch_ = switch_channel_active(not_di_channel_) ? 1.0 : 0.0;
  hw_home_switch_ = switch_channel_active(home_di_channel_) ? 1.0 : 0.0;
  hw_lower_limit_switch_ = switch_channel_active(pot_di_channel_) ? 1.0 : 0.0;
  hw_limit_switch_valid_ = limit_switch_state_valid_ ? 1.0 : 0.0;
  hw_digital_inputs_raw_ = static_cast<double>(async_digital_inputs_.load());
  hw_physical_position_ = async_position_.load();
  hw_profile_speed_mps_ = profile_speed_mps_.load();
  hw_profile_accel_mps2_ = static_cast<double>(profile_acceleration_.load()) / counts_per_meter_;
  hw_profile_decel_mps2_ = static_cast<double>(profile_deceleration_.load()) / counts_per_meter_;
  hw_drive_enabled_ = drive_enabled_.load() ? 1.0 : 0.0;
  hw_quick_stop_active_ = quick_stop_active_.load() ? 1.0 : 0.0;
  hw_motion_ready_ = motion_ready_.load() ? 1.0 : 0.0;
  hw_reference_valid_ = reference_valid_.load() ? 1.0 : 0.0;
  hw_encoder_reference_lost_ = encoder_reference_lost_.load() ? 1.0 : 0.0;
}

void LiftSlideHardwareInterface::process_command_interfaces()
{
  // --- quick_stop_cmd (fast path — immediate RPDO) ---
  if (hw_quick_stop_cmd_ > 0.5) {
    hw_quick_stop_cmd_ = 0.0;
    const bool was_homing = homing_in_progress_.load();
    RCLCPP_WARN(
      logger_,
      "quick_stop_cmd received: pos=%.4f vel=%.4f statusword=0x%04X upper=%d lower=%d home=%d was_homing=%d",
      hw_position_,
      hw_velocity_,
      last_statusword_.load(),
      switch_channel_active(not_di_channel_) ? 1 : 0,
      switch_channel_active(pot_di_channel_) ? 1 : 0,
      switch_channel_active(home_di_channel_) ? 1 : 0,
      was_homing ? 1 : 0);
    homing_in_progress_.store(false);
    hw_command_velocity_ = 0.0;
    last_sent_velocity_command_ = 0.0;
    quick_stop_active_.store(true);
    if (!use_fake_hardware_) {
      (void)send_rpdo1(CMD_QUICK_STOP, 0, MODE_VELOCITY);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      (void)send_rpdo1(CMD_DISABLE_VOLTAGE, 0, MODE_VELOCITY);
    }
    drive_enabled_.store(false);
    is_enabled_.store(false);
    cia402_state_.store(static_cast<int>(Cia402State::SWITCH_ON_DISABLED));
    hw_velocity_ = 0.0;
    last_target_velocity_counts_per_sec_.store(0);
    if (!use_fake_hardware_) {
      run_detached([this]() {
        (void)sdo_write(OBJ_TARGET_VELOCITY, 0x00, 0, 4, true);
        (void)sdo_write(OBJ_CONTROLWORD, 0x00, CMD_DISABLE_VOLTAGE, 2, false);
      });
    }
    join_homing_thread();
    feedback_poll_paused_.store(false);
    if (was_homing) {
      set_homing_state(static_cast<int>(HSC::ERROR), static_cast<int>(HD::ABORTED_BY_QUICK_STOP));
    } else {
      set_homing_state(static_cast<int>(HSC::IDLE), static_cast<int>(HD::DISABLED));
    }
  }

  // --- manual_limit_stop_cmd: quick stop and disable for manual soft/hard limit protection ---
  if (hw_manual_limit_stop_cmd_ > 0.5) {
    hw_manual_limit_stop_cmd_ = 0.0;
    const bool homing_request_pending = hw_homing_cmd_ > 0.5;
    const bool was_homing = homing_in_progress_.load() || homing_request_pending;
    if (was_homing) {
      RCLCPP_WARN(
        logger_,
        "manual_limit_stop_cmd ignored during homing: pos=%.4f vel=%.4f statusword=0x%04X upper=%d lower=%d home=%d pending_cmd=%.1f",
        hw_position_,
        hw_velocity_,
        last_statusword_.load(),
        switch_channel_active(not_di_channel_) ? 1 : 0,
        switch_channel_active(pot_di_channel_) ? 1 : 0,
        switch_channel_active(home_di_channel_) ? 1 : 0,
        hw_homing_cmd_);
    } else {
      RCLCPP_WARN(
        logger_,
        "manual_limit_stop_cmd received: pos=%.4f vel=%.4f statusword=0x%04X upper=%d lower=%d home=%d was_homing=%d",
        hw_position_,
        hw_velocity_,
        last_statusword_.load(),
        switch_channel_active(not_di_channel_) ? 1 : 0,
        switch_channel_active(pot_di_channel_) ? 1 : 0,
        switch_channel_active(home_di_channel_) ? 1 : 0,
        was_homing ? 1 : 0);
      homing_in_progress_.store(false);
      hw_command_velocity_ = 0.0;
      hw_position_command_ = std::numeric_limits<double>::quiet_NaN();
      last_sent_velocity_command_ = 0.0;
      quick_stop_active_.store(true);
      if (!use_fake_hardware_) {
        (void)send_rpdo1(CMD_QUICK_STOP, 0, MODE_VELOCITY);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        (void)send_rpdo1(CMD_DISABLE_VOLTAGE, 0, MODE_VELOCITY);
      }
      drive_enabled_.store(false);
      is_enabled_.store(false);
      cia402_state_.store(static_cast<int>(Cia402State::SWITCH_ON_DISABLED));
      hw_velocity_ = 0.0;
      last_target_velocity_counts_per_sec_.store(0);
      position_move_in_progress_.store(false);
      manual_hold_active_.store(false);
      ignore_current_position_command_after_hold_.store(false);
      if (!use_fake_hardware_) {
        run_detached([this]() {
          (void)sdo_write(OBJ_TARGET_VELOCITY, 0x00, 0, 4, true);
          (void)sdo_write(OBJ_CONTROLWORD, 0x00, CMD_DISABLE_VOLTAGE, 2, false);
        });
      }
      join_homing_thread();
      feedback_poll_paused_.store(false);
      set_homing_state(static_cast<int>(HSC::IDLE), static_cast<int>(HD::DISABLED));
    }
  }

  // --- hold_position_cmd ---
  if (hw_hold_position_cmd_ > 0.5) {
    hw_hold_position_cmd_ = 0.0;
    hw_command_velocity_ = 0.0;
    last_sent_velocity_command_ = 0.0;
    last_target_velocity_counts_per_sec_.store(0);
    ignored_position_command_after_hold_ = hw_position_command_;
    hw_position_command_ = std::numeric_limits<double>::quiet_NaN();
    last_sent_position_command_ = ignored_position_command_after_hold_;
    position_move_in_progress_.store(false);
    manual_hold_active_.store(true);
    ignore_current_position_command_after_hold_.store(true);
    RCLCPP_INFO(
      logger_,
      "[manual] hold cmd received: drive_enabled=%d homing=%d active_mode=%d dispatch_busy=%d",
      drive_enabled_.load() ? 1 : 0,
      homing_in_progress_.load() ? 1 : 0,
      static_cast<int>(active_mode_.load()),
      cmd_dispatch_in_progress_->load() ? 1 : 0);

    if (use_fake_hardware_) {
      hw_velocity_ = 0.0;
      manual_hold_active_.store(false);
    } else if (drive_enabled_.load() && !homing_in_progress_.load()) {
      run_detached([this]() {
        (void)hold_current_position();
      }, [this]() {
        manual_hold_active_.store(false);
      });
    } else {
      manual_hold_active_.store(false);
    }
  }

  // --- manual_halt_cmd ---
  if (hw_manual_halt_cmd_ > 0.5) {
    hw_manual_halt_cmd_ = 0.0;
    hw_command_velocity_ = 0.0;
    last_sent_velocity_command_ = 0.0;
    last_target_velocity_counts_per_sec_.store(0);
    ignored_position_command_after_hold_ = hw_position_command_;
    hw_position_command_ = std::numeric_limits<double>::quiet_NaN();
    last_sent_position_command_ = ignored_position_command_after_hold_;
    position_move_in_progress_.store(false);
    manual_hold_active_.store(true);
    ignore_current_position_command_after_hold_.store(true);
    RCLCPP_INFO(
      logger_,
      "[manual] halt cmd received: drive_enabled=%d homing=%d active_mode=%d dispatch_busy=%d",
      drive_enabled_.load() ? 1 : 0,
      homing_in_progress_.load() ? 1 : 0,
      static_cast<int>(active_mode_.load()),
      cmd_dispatch_in_progress_->load() ? 1 : 0);

    if (use_fake_hardware_) {
      hw_velocity_ = 0.0;
      manual_hold_active_.store(false);
    } else if (drive_enabled_.load() && !homing_in_progress_.load()) {
      // Stop the profile-position generator in this control cycle.  The
      // detached SDO sequence below then latches the measured position as the
      // new target, but it must not be the first point at which motion stops.
      (void)send_rpdo1(
        CMD_ENABLE_OPERATION | 0x0100,
        0,
        MODE_POSITION);
      run_detached([this]() {
        (void)halt_position_motion();
      }, [this]() {
        manual_hold_active_.store(false);
      });
    } else {
      manual_hold_active_.store(false);
    }
  }

  // --- enable_cmd ---
  if (hw_enable_cmd_ > 0.5) {
    hw_enable_cmd_ = 0.0;
    hw_command_velocity_ = 0.0;
    last_sent_velocity_command_ = 0.0;
    last_target_velocity_counts_per_sec_.store(std::numeric_limits<int32_t>::min());
    if (use_fake_hardware_) {
      drive_enabled_.store(true);
      is_enabled_.store(true);
      cia402_state_.store(static_cast<int>(Cia402State::OPERATION_ENABLED));
      quick_stop_active_.store(false);
      set_homing_state(static_cast<int>(HSC::IDLE), static_cast<int>(HD::ENABLED));
    } else if (!homing_in_progress_.load()) {
      run_detached([this]() {
        feedback_poll_paused_.store(true);
        if (!send_nmt(0x01)) {
          RCLCPP_WARN(logger_, "Failed to send NMT start during enable");
        }
        if (!reset_fault()) {
          RCLCPP_WARN(logger_, "Fault reset was not acknowledged during enable");
        }
        bool ok = configure_drive() && enable_drive();
        if (ok) {
          ok = sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false);
        }
        if (ok) {
          drive_enabled_.store(true);
          quick_stop_active_.store(false);
          (void)read_feedback_once();
          set_homing_state(static_cast<int>(HSC::IDLE), static_cast<int>(HD::ENABLED));
        } else {
          RCLCPP_ERROR(
            logger_,
            "enable command failed: pos=%.4f vel=%.4f statusword=0x%04X upper=%d lower=%d home=%d",
            hw_position_,
            hw_velocity_,
            last_statusword_.load(),
            switch_channel_active(not_di_channel_) ? 1 : 0,
            switch_channel_active(pot_di_channel_) ? 1 : 0,
            switch_channel_active(home_di_channel_) ? 1 : 0);
          (void)disable_drive();
          drive_enabled_.store(false);
          set_homing_state(static_cast<int>(HSC::ERROR), static_cast<int>(HD::ENABLE_FAILED));
        }
        feedback_poll_paused_.store(false);
      });
    }
  } else if (hw_enable_cmd_ < -0.5) {
    // disable
    hw_enable_cmd_ = 0.0;
    RCLCPP_WARN(
      logger_,
      "disable command received: pos=%.4f vel=%.4f statusword=0x%04X upper=%d lower=%d home=%d",
      hw_position_,
      hw_velocity_,
      last_statusword_.load(),
      switch_channel_active(not_di_channel_) ? 1 : 0,
      switch_channel_active(pot_di_channel_) ? 1 : 0,
      switch_channel_active(home_di_channel_) ? 1 : 0);
    hw_command_velocity_ = 0.0;
    hw_position_command_ = std::numeric_limits<double>::quiet_NaN();
    if (use_fake_hardware_) {
      drive_enabled_.store(false);
      is_enabled_.store(false);
      cia402_state_.store(static_cast<int>(Cia402State::SWITCH_ON_DISABLED));
      manual_hold_active_.store(false);
    } else {
      run_detached([this]() {
        motor_power_off_sequence();
        drive_enabled_.store(false);
        is_enabled_.store(false);
        manual_hold_active_.store(false);
        set_homing_state(static_cast<int>(HSC::IDLE), static_cast<int>(HD::DISABLED));
      });
    }
  }

  // --- reset_fault_cmd ---
  if (hw_reset_fault_cmd_ > 0.5) {
    hw_reset_fault_cmd_ = 0.0;
    if (use_fake_hardware_) {
      is_fault_.store(false);
    } else {
      run_detached([this]() {
        feedback_poll_paused_.store(true);
        (void)reset_fault();
        feedback_poll_paused_.store(false);
      });
    }
  }

  // --- full_reset_cmd ---
  if (hw_full_reset_cmd_ > 0.5) {
    hw_full_reset_cmd_ = 0.0;
    homing_in_progress_.store(false);
    hw_command_velocity_ = 0.0;
    last_sent_velocity_command_ = 0.0;
    last_target_velocity_counts_per_sec_.store(std::numeric_limits<int32_t>::min());
    hw_velocity_ = 0.0;
    join_homing_thread();
    if (use_fake_hardware_) {
      drive_enabled_.store(true);
      quick_stop_active_.store(false);
      set_homing_state(static_cast<int>(HSC::IDLE), static_cast<int>(HD::ENABLED));
    } else {
      run_detached([this]() {
        feedback_poll_paused_.store(true);
        (void)send_rpdo1(CMD_QUICK_STOP, 0, MODE_VELOCITY);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        (void)send_rpdo1(CMD_DISABLE_VOLTAGE, 0, MODE_VELOCITY);
        (void)sdo_write(OBJ_CONTROLWORD, 0x00, CMD_DISABLE_VOLTAGE, 2, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (!send_nmt(0x01)) {
          RCLCPP_WARN(logger_, "Failed to send NMT start during full_reset");
        }
        bool ok = reset_fault();
        if (ok) { ok = configure_drive(); }
        if (ok) { ok = enable_drive(); }
        if (ok) { ok = sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false); }
        feedback_poll_paused_.store(false);
        if (ok) {
          drive_enabled_.store(true);
          quick_stop_active_.store(false);
          (void)read_feedback_once();
          set_homing_state(static_cast<int>(HSC::IDLE), static_cast<int>(HD::ENABLED));
        } else {
          (void)disable_drive();
          drive_enabled_.store(false);
          quick_stop_active_.store(false);
          set_homing_state(static_cast<int>(HSC::ERROR), static_cast<int>(HD::FULL_RESET_FAILED));
        }
      });
    }
  }

  // --- homing_cmd ---
  if (hw_homing_cmd_ > 0.5 && hw_homing_cmd_ < 1.5) {
    // start_homing
    hw_homing_cmd_ = 0.0;
    if (!shutdown_requested_.load() && !quick_stop_active_.load()) {
      bool expected = false;
      if (homing_in_progress_.compare_exchange_strong(expected, true)) {
        homing_complete_.store(false);
        homing_error_.store(false);
        set_homing_state(static_cast<int>(HSC::IN_PROGRESS), static_cast<int>(HD::ACCEPTED));
        join_homing_thread();
        homing_thread_ = std::thread([this]() {
          const bool ok = use_fake_hardware_ ? simulate_fake_homing_sequence() : start_homing_sequence();
          if (ok) {
            homing_complete_.store(true);
            homing_error_.store(false);
            set_homing_state(static_cast<int>(HSC::COMPLETED), static_cast<int>(HD::OK));
          } else {
            homing_complete_.store(false);
            homing_error_.store(true);
            if (quick_stop_active_.load()) {
              set_homing_state(static_cast<int>(HSC::ERROR), static_cast<int>(HD::ABORTED_BY_QUICK_STOP));
            } else {
              set_homing_state(static_cast<int>(HSC::ERROR), static_cast<int>(HD::FAILED));
            }
          }
          homing_in_progress_.store(false);
        });
      }
    }
  } else if (hw_homing_cmd_ > 1.5) {
    // return_home
    hw_homing_cmd_ = 0.0;
    if (!shutdown_requested_.load() && !quick_stop_active_.load() && homing_complete_.load()) {
      bool expected = false;
      if (homing_in_progress_.compare_exchange_strong(expected, true)) {
        homing_error_.store(false);
        set_homing_state(static_cast<int>(HSC::IN_PROGRESS), static_cast<int>(HD::ACCEPTED));
        join_homing_thread();
        homing_thread_ = std::thread([this]() {
          const bool ok = use_fake_hardware_ ? simulate_fake_homing_sequence() : return_to_home_sequence();
          if (ok) {
            set_homing_state(static_cast<int>(HSC::COMPLETED), static_cast<int>(HD::OK));
          } else {
            homing_error_.store(true);
            if (quick_stop_active_.load()) {
              set_homing_state(static_cast<int>(HSC::ERROR), static_cast<int>(HD::ABORTED_BY_QUICK_STOP));
            } else {
              set_homing_state(static_cast<int>(HSC::ERROR), static_cast<int>(HD::FAILED));
            }
          }
          homing_in_progress_.store(false);
        });
      }
    }
  }

  // --- set_mode_cmd ---
  if (std::abs(hw_set_mode_cmd_) > 0.5) {
    const int8_t mode = static_cast<int8_t>(hw_set_mode_cmd_);
    hw_set_mode_cmd_ = 0.0;
    if (mode == MODE_VELOCITY) {
      if (homing_in_progress_.load()) {
        homing_in_progress_.store(false);
      }
      if (use_fake_hardware_) {
        active_mode_.store(MODE_VELOCITY);
        current_mode_of_operation_.store(MODE_VELOCITY);
      } else {
        run_detached([this]() {
          if (restore_velocity_mode()) {
            active_mode_.store(MODE_VELOCITY);
            current_mode_of_operation_.store(MODE_VELOCITY);
          }
        });
      }
    } else if (mode == MODE_POSITION) {
      if (homing_in_progress_.load()) {
        homing_in_progress_.store(false);
      }
      if (use_fake_hardware_) {
        active_mode_.store(MODE_POSITION);
        current_mode_of_operation_.store(MODE_POSITION);
      } else {
        run_detached([this]() {
          if (set_mode_of_operation(MODE_POSITION)) {
            active_mode_.store(MODE_POSITION);
            current_mode_of_operation_.store(MODE_POSITION);
          }
        });
      }
    } else if (mode == MODE_HOMING) {
      if (use_fake_hardware_) {
        active_mode_.store(MODE_HOMING);
        current_mode_of_operation_.store(MODE_HOMING);
      } else {
        run_detached([this]() {
          if (set_mode_of_operation(MODE_HOMING)) {
            active_mode_.store(MODE_HOMING);
            current_mode_of_operation_.store(MODE_HOMING);
          }
        });
      }
    }
  }

  // --- profile_speed_cmd ---
  if (!std::isnan(hw_profile_speed_cmd_)) {
    profile_speed_mps_.store(std::max(0.001, hw_profile_speed_cmd_));
    hw_profile_speed_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

  // --- accel_time_cmd ---
  if (!std::isnan(hw_accel_time_cmd_)) {
    const double val = hw_accel_time_cmd_;
    hw_accel_time_cmd_ = std::numeric_limits<double>::quiet_NaN();
    if (val > 0.0 && !use_fake_hardware_ && drive_enabled_.load()) {
      run_detached([this, val]() {
        sdo_write(OBJ_ACCEL_TIME, 0x00, static_cast<int32_t>(val), 4, false);
      });
    }
  }

  // --- decel_time_cmd ---
  if (!std::isnan(hw_decel_time_cmd_)) {
    const double val = hw_decel_time_cmd_;
    hw_decel_time_cmd_ = std::numeric_limits<double>::quiet_NaN();
    if (val > 0.0 && !use_fake_hardware_ && drive_enabled_.load()) {
      run_detached([this, val]() {
        sdo_write(OBJ_DECEL_TIME, 0x00, static_cast<int32_t>(val), 4, false);
      });
    }
  }

  // --- profile accel/decel via mps2 (from parameter interface) ---
  // These are handled through the state controller's parameter callback
  // writing to accel_time_cmd/decel_time_cmd above.
}

void LiftSlideHardwareInterface::join_homing_thread()
{
  if (homing_thread_.joinable()) {
    homing_thread_.join();
  }
}

void LiftSlideHardwareInterface::update_limit_switch_state(uint32_t digital_inputs_raw)
{
  const bool had_previous_sample = limit_switch_state_valid_;
  const uint32_t previous_raw = last_digital_inputs_raw_;

  last_digital_inputs_raw_ = digital_inputs_raw;
  limit_switch_state_valid_ = true;
  cia_not_bit0_ = ((digital_inputs_raw >> 0U) & 0x1U) != 0U;
  cia_pot_bit1_ = ((digital_inputs_raw >> 1U) & 0x1U) != 0U;
  cia_home_bit2_ = ((digital_inputs_raw >> 2U) & 0x1U) != 0U;
  si1_bit4_ = ((digital_inputs_raw >> 4U) & 0x1U) != 0U;
  si2_bit5_ = ((digital_inputs_raw >> 5U) & 0x1U) != 0U;
  si3_bit6_ = ((digital_inputs_raw >> 6U) & 0x1U) != 0U;
  si4_bit7_ = ((digital_inputs_raw >> 7U) & 0x1U) != 0U;
  si5_bit8_ = ((digital_inputs_raw >> 8U) & 0x1U) != 0U;
  si6_bit9_ = ((digital_inputs_raw >> 9U) & 0x1U) != 0U;
  di1_bit16_ = ((digital_inputs_raw >> 16U) & 0x1U) != 0U;
  di2_bit17_ = ((digital_inputs_raw >> 17U) & 0x1U) != 0U;
  di3_bit18_ = ((digital_inputs_raw >> 18U) & 0x1U) != 0U;
  di4_bit19_ = ((digital_inputs_raw >> 19U) & 0x1U) != 0U;
  di5_bit20_ = ((digital_inputs_raw >> 20U) & 0x1U) != 0U;
  di6_bit21_ = ((digital_inputs_raw >> 21U) & 0x1U) != 0U;
  di1_bit24_ = ((digital_inputs_raw >> 24U) & 0x1U) != 0U;
  di2_bit25_ = ((digital_inputs_raw >> 25U) & 0x1U) != 0U;
  di3_bit26_ = ((digital_inputs_raw >> 26U) & 0x1U) != 0U;
  di4_bit27_ = ((digital_inputs_raw >> 27U) & 0x1U) != 0U;
  di5_bit28_ = ((digital_inputs_raw >> 28U) & 0x1U) != 0U;
  di6_bit29_ = ((digital_inputs_raw >> 29U) & 0x1U) != 0U;

  if (!had_previous_sample || previous_raw != digital_inputs_raw) {
    const bool upper_switch = switch_channel_active(not_di_channel_);
    const bool home_switch = switch_channel_active(home_di_channel_);
    const bool lower_switch = switch_channel_active(pot_di_channel_);
    const bool upper_physical = physical_di_channel_active(not_di_channel_);
    const bool home_physical = physical_di_channel_active(home_di_channel_);
    const bool lower_physical = physical_di_channel_active(pot_di_channel_);
    RCLCPP_INFO(
      logger_,
      "DI change: 0x60FD=0x%08X b4-9(SI1..SI6=%d%d%d%d%d%d) b16-21=%d%d%d%d%d%d b24-29=%d%d%d%d%d%d b0-2(NOT%d/POT%d/HOME%d) map(home=DI%d:%d pot=DI%d:%d not=DI%d:%d) phys(home=DI%d:%d pot=DI%d:%d not=DI%d:%d)",
      digital_inputs_raw,
      si1_bit4_ ? 1 : 0,
      si2_bit5_ ? 1 : 0,
      si3_bit6_ ? 1 : 0,
      si4_bit7_ ? 1 : 0,
      si5_bit8_ ? 1 : 0,
      si6_bit9_ ? 1 : 0,
      di1_bit16_ ? 1 : 0,
      di2_bit17_ ? 1 : 0,
      di3_bit18_ ? 1 : 0,
      di4_bit19_ ? 1 : 0,
      di5_bit20_ ? 1 : 0,
      di6_bit21_ ? 1 : 0,
      di1_bit24_ ? 1 : 0,
      di2_bit25_ ? 1 : 0,
      di3_bit26_ ? 1 : 0,
      di4_bit27_ ? 1 : 0,
      di5_bit28_ ? 1 : 0,
      di6_bit29_ ? 1 : 0,
      cia_not_bit0_ ? 1 : 0,
      cia_pot_bit1_ ? 1 : 0,
      cia_home_bit2_ ? 1 : 0,
      home_di_channel_,
      home_switch ? 1 : 0,
      pot_di_channel_,
      lower_switch ? 1 : 0,
      not_di_channel_,
      upper_switch ? 1 : 0,
      home_di_channel_,
      home_physical ? 1 : 0,
      pot_di_channel_,
      lower_physical ? 1 : 0,
      not_di_channel_,
      upper_physical ? 1 : 0);
  }
}

uint16_t LiftSlideHardwareInterface::di_config_index_for_channel(int channel) const
{
  switch (channel) {
    case 1: return openflex_can::Ld2CanopenDrive::index(Ld2Object::DIGITAL_INPUT_1_CONFIG);
    case 2: return openflex_can::Ld2CanopenDrive::index(Ld2Object::DIGITAL_INPUT_2_CONFIG);
    case 3: return openflex_can::Ld2CanopenDrive::index(Ld2Object::DIGITAL_INPUT_3_CONFIG);
    case 4: return openflex_can::Ld2CanopenDrive::index(Ld2Object::DIGITAL_INPUT_4_CONFIG);
    case 5: return openflex_can::Ld2CanopenDrive::index(Ld2Object::DIGITAL_INPUT_5_CONFIG);
    case 6: return openflex_can::Ld2CanopenDrive::index(Ld2Object::DIGITAL_INPUT_6_CONFIG);
    default: return 0;
  }
}

bool LiftSlideHardwareInterface::physical_di_channel_level_high(int channel) const
{
  // 0x60FD bit24~bit29 carry the physical electrical levels for DI1~DI6.
  // The lower SI bits are function-level signals and can be remapped to
  // HOME/POT/NOT, so they must not be used as raw manual limit inputs.
  switch (channel) {
    case 1:
      return di1_bit24_;
    case 2:
      return di2_bit25_;
    case 3:
      return di3_bit26_;
    case 4:
      return di4_bit27_;
    case 5:
      return di5_bit28_;
    case 6:
      return di6_bit29_;
    default:
      return false;
  }
}

bool LiftSlideHardwareInterface::physical_di_channel_active(int channel) const
{
  const bool level_high = physical_di_channel_level_high(channel);
  return di_active_low_ ? !level_high : level_high;
}

bool LiftSlideHardwareInterface::switch_channel_active(int channel) const
{
  if (channel == home_di_channel_ && homing_di4_config_applied_) {
    return cia_home_bit2_;
  }
  if (channel == pot_di_channel_ && homing_di5_config_applied_) {
    return cia_pot_bit1_;
  }
  if (channel == not_di_channel_ && homing_di6_config_applied_) {
    return cia_not_bit0_;
  }
  return physical_di_channel_active(channel);
}

bool LiftSlideHardwareInterface::limit_switch_active(int channel) const
{
  return switch_channel_active(channel);
}

bool LiftSlideHardwareInterface::init_can_socket()
{
  return canopen_master_.open(can_interface_);
}

void LiftSlideHardwareInterface::close_can_socket()
{
  canopen_master_.close();
}

bool LiftSlideHardwareInterface::send_can_frame(
  uint32_t can_id,
  const std::array<uint8_t, 8> & data,
  uint8_t dlc)
{
  return canopen_master_.send_frame(can_id, data, dlc);
}

bool LiftSlideHardwareInterface::recv_can_frame(struct can_frame & frame, int timeout_ms)
{
  return canopen_master_.receive_frame(frame, timeout_ms);
}

void LiftSlideHardwareInterface::drain_rx()
{
  canopen_master_.drain_rx();
}

bool LiftSlideHardwareInterface::send_nmt(uint8_t command)
{
  return canopen_master_.send_nmt(command);
}

bool LiftSlideHardwareInterface::sdo_write(
  uint16_t index,
  uint8_t subindex,
  int32_t value,
  uint8_t size,
  bool signed_value)
{
  const auto result = canopen_master_.sdo_write_ex(
    static_cast<uint8_t>(node_id_),
    index,
    subindex,
    value,
    size,
    signed_value,
    static_cast<int>(sdo_timeout_sec_ * 1000.0));
  if (!result.ok) {
    RCLCPP_WARN(
      logger_,
      "SDO write 0x%04X:%u failed: %s abort=0x%08X timeout=%d",
      index,
      subindex,
      result.error.c_str(),
      result.abort_code,
      result.timeout ? 1 : 0);
  }
  return result.ok;
}

bool LiftSlideHardwareInterface::sdo_read(
  uint16_t index,
  uint8_t subindex,
  int32_t & value,
  bool signed_value)
{
  const auto result = canopen_master_.sdo_read_ex(
    static_cast<uint8_t>(node_id_),
    index,
    subindex,
    value,
    signed_value,
    static_cast<int>(sdo_timeout_sec_ * 1000.0));
  if (!result.ok) {
    RCLCPP_WARN(
      logger_,
      "SDO read 0x%04X:%u failed: %s abort=0x%08X timeout=%d",
      index,
      subindex,
      result.error.c_str(),
      result.abort_code,
      result.timeout ? 1 : 0);
  }
  return result.ok;
}

bool LiftSlideHardwareInterface::send_rpdo1(
  uint16_t controlword,
  int32_t target_velocity_counts_per_sec,
  int8_t mode)
{
  return canopen_master_.send_rpdo1(
    static_cast<uint8_t>(node_id_),
    controlword,
    target_velocity_counts_per_sec,
    mode);
}

bool LiftSlideHardwareInterface::set_mode_of_operation(int8_t mode)
{
  if (!sdo_write(OBJ_CONTROLWORD, 0x00, CMD_SWITCH_ON, 2, false)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  if (!sdo_write(OBJ_MODE_OF_OPERATION, 0x00, mode, 1, true)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  if (!sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  return true;
}

bool LiftSlideHardwareInterface::restore_velocity_mode()
{
  if (!set_mode_of_operation(MODE_VELOCITY)) {
    return false;
  }
  if (!sdo_write(OBJ_TARGET_VELOCITY, 0x00, 0, 4, true)) {
    return false;
  }
  return sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false);
}

bool LiftSlideHardwareInterface::clear_homing_di_configuration()
{
  bool ok = true;
  auto clear_if_applied = [this, &ok](const char * name, uint16_t index, bool & applied_flag) {
      if (!applied_flag) {
        return;
      }
      if (!sdo_write(index, 0x00, 0x00, 4, false)) {
        RCLCPP_WARN(logger_, "%s clear failed", name);
        ok = false;
        return;
      }
      applied_flag = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    };

  const auto home_index = di_config_index_for_channel(home_di_channel_);
  const auto pot_index = di_config_index_for_channel(pot_di_channel_);
  const auto not_index = di_config_index_for_channel(not_di_channel_);
  const std::string home_name = "HOME(DI" + std::to_string(home_di_channel_) + ")";
  const std::string pot_name = "POT(DI" + std::to_string(pot_di_channel_) + ")";
  const std::string not_name = "NOT(DI" + std::to_string(not_di_channel_) + ")";

  clear_if_applied(home_name.c_str(), home_index, homing_di4_config_applied_);
  clear_if_applied(pot_name.c_str(), pot_index, homing_di5_config_applied_);
  clear_if_applied(not_name.c_str(), not_index, homing_di6_config_applied_);
  return ok;
}

bool LiftSlideHardwareInterface::trigger_homing_start()
{
  // CiA402 homing trigger: bit4 must see a 1→0→1 rising edge.
  // Step1: assert bit4=1 first (ensure known state)
  if (!sdo_write(OBJ_CONTROLWORD, 0x00, 0x001F, 2, false)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // Step2: clear bit4=0
  if (!sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // Step3: set bit4=1 again — this rising edge triggers homing
  if (!sdo_write(OBJ_CONTROLWORD, 0x00, 0x001F, 2, false)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return true;
}

bool LiftSlideHardwareInterface::recover_encoder_reference_loss()
{
  // Er153 means the multi-turn reference is no longer trustworthy.  The LD2
  // still needs its multi-turn state cleared while disabled before it can be
  // enabled for temporary operation.  This is deliberately not a mechanical
  // set-zero: start_homing_sequence() remains the only path that validates a
  // real zero against the installed sensors.
  const auto fail = [this](const std::string & reason) {
      RCLCPP_ERROR(logger_, "Encoder-reference recovery failed: %s", reason.c_str());
      (void)disable_drive();
      drive_enabled_.store(false);
      motion_ready_.store(false);
      return false;
    };

  if (!disable_drive()) {
    return fail("cannot disable drive before clearing Er153");
  }

  bool disabled_confirmed = false;
  for (int poll = 0; poll < 20; ++poll) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int32_t statusword_raw = 0;
    if (!sdo_read(OBJ_STATUSWORD, 0x00, statusword_raw, false)) {
      continue;
    }
    const uint16_t statusword = static_cast<uint16_t>(statusword_raw & 0xFFFF);
    last_statusword_.store(statusword);
    update_cia402_state(statusword);
    if (cia402_state_.load() == static_cast<int>(Cia402State::SWITCH_ON_DISABLED)) {
      disabled_confirmed = true;
      break;
    }
  }
  if (!disabled_confirmed) {
    return fail("drive did not reach SWITCH_ON_DISABLED");
  }

  if (!sdo_write(OBJ_MODE_OF_OPERATION, 0x00, MODE_HOMING, 1, true)) {
    return fail("cannot select homing mode before clearing multi-turn state");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  int32_t mode_display = 0;
  if (!sdo_read(OBJ_MODE_OF_OPERATION_DISPLAY, 0x00, mode_display, true) ||
    static_cast<int8_t>(mode_display & 0xFF) != MODE_HOMING)
  {
    return fail("drive did not confirm homing mode");
  }

  if (!canopen_master_.write_absolute_encoder_command(
      9U, static_cast<int>(sdo_timeout_sec_ * 1000.0)))
  {
    return fail("2015h=9 write was rejected");
  }
  bool multi_turn_cleared = false;
  uint32_t absolute_encoder_readback = 9U;
  for (int poll = 0; poll < 60; ++poll) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!canopen_master_.read_absolute_encoder_parameter(
        absolute_encoder_readback, static_cast<int>(sdo_timeout_sec_ * 1000.0)))
    {
      continue;
    }
    if (absolute_encoder_readback == 1U || absolute_encoder_readback == 2U) {
      multi_turn_cleared = true;
      break;
    }
  }
  if (!multi_turn_cleared) {
    return fail("2015h=9 did not complete within three seconds");
  }

  if (!canopen_master_.clear_position(static_cast<int>(sdo_timeout_sec_ * 1000.0))) {
    return fail("2610h=1 write was rejected");
  }
  constexpr int32_t kTemporaryZeroToleranceCounts = 10;
  int32_t position_counts = 0;
  bool position_cleared = false;
  for (int poll = 0; poll < 60; ++poll) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!sdo_read(OBJ_POSITION_ACTUAL, 0x00, position_counts, true)) {
      continue;
    }
    if (std::abs(position_counts) <= kTemporaryZeroToleranceCounts) {
      position_cleared = true;
      break;
    }
  }
  if (!position_cleared) {
    return fail("6064h did not reach zero after 2610h=1");
  }

  uint32_t error_code = 0U;
  if (!canopen_master_.read_error_code(
      error_code, static_cast<int>(sdo_timeout_sec_ * 1000.0)) ||
    (error_code & 0xFFFFU) != 0U)
  {
    return fail("603Fh remains non-zero after multi-turn clear");
  }

  if (!configure_drive() || !enable_drive() ||
    !sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false))
  {
    return fail("cannot re-enable drive after clearing Er153");
  }
  if (!read_feedback_once()) {
    return fail("cannot read feedback after temporary recovery");
  }

  zero_source_ = "encoder_recovery";
  zero_raw_position_ = 0;
  zero_verified_by_drive_ = false;
  position_offset_m_.store(0.0);
  homing_complete_.store(false);
  reference_valid_.store(false);
  encoder_reference_lost_.store(true);
  motion_ready_.store(true);
  drive_enabled_.store(true);
  if (!save_calibration() || !load_calibration()) {
    return fail("cannot persist and reload temporary recovery state");
  }

  RCLCPP_WARN(
    logger_,
    "Er153 recovery completed: manual motion is available, but the mechanical zero is unverified; run set-zero");
  return true;
}

bool LiftSlideHardwareInterface::set_drive_zero_point()
{
  if (use_fake_hardware_) {
    zero_verified_by_drive_ = true;
    zero_raw_position_ = 0;
    position_offset_m_.store(0.0);
    return true;
  }

  int32_t position_before_zero = 0;
  if (!sdo_read(OBJ_POSITION_ACTUAL, 0x00, position_before_zero, true)) {
    RCLCPP_ERROR(logger_, "Cannot read 6064h before writing LD2 zero point");
    return false;
  }

  // The LD2 rejects the absolute-encoder zero command while the motor is
  // still decelerating.  Confirm actual velocity (606Ch), rather than using
  // a fixed delay, before disabling the drive and writing 2015h.
  constexpr int32_t velocity_zero_threshold_counts_per_sec = 100;
  constexpr int velocity_zero_streak_required = 3;
  constexpr int velocity_zero_poll_limit = 40;  // 2 s at 50 ms per sample
  int velocity_zero_streak = 0;
  int32_t actual_velocity_before_zero = 0;
  bool velocity_zero_confirmed = false;
  for (int poll = 0; poll < velocity_zero_poll_limit; ++poll) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!sdo_read(OBJ_VELOCITY_ACTUAL, 0x00, actual_velocity_before_zero, true)) {
      velocity_zero_streak = 0;
      continue;
    }
    if (std::abs(actual_velocity_before_zero) <= velocity_zero_threshold_counts_per_sec) {
      ++velocity_zero_streak;
      if (velocity_zero_streak >= velocity_zero_streak_required) {
        velocity_zero_confirmed = true;
        break;
      }
    } else {
      velocity_zero_streak = 0;
    }
  }
  if (!velocity_zero_confirmed) {
    RCLCPP_ERROR(
      logger_, "Timed out waiting for actual velocity to settle before zeroing (606Ch=%d)",
      actual_velocity_before_zero);
    return false;
  }
  RCLCPP_INFO(
    logger_, "Zero point: actual velocity settled (606Ch=%d)", actual_velocity_before_zero);

  // LD2 manual: 2015h=9 only responds while the drive is disabled. 2610h=1
  // clears 6064h and is the power-cycle-persistent zero operation.
  if (!disable_drive()) {
    RCLCPP_ERROR(logger_, "Cannot disable drive before writing LD2 zero point");
    return false;
  }
  drive_enabled_.store(false);

  // An acknowledged 6040h write only means the command reached the drive.
  // Do not write 2015h until 6041h confirms the required disabled state.
  bool disabled_confirmed = false;
  uint16_t disabled_statusword = 0;
  for (int poll = 0; poll < 20; ++poll) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int32_t statusword_raw = 0;
    if (!sdo_read(OBJ_STATUSWORD, 0x00, statusword_raw, false)) {
      continue;
    }
    disabled_statusword = static_cast<uint16_t>(statusword_raw & 0xFFFF);
    last_statusword_.store(disabled_statusword);
    update_cia402_state(disabled_statusword);
    if (cia402_state_.load() == static_cast<int>(Cia402State::SWITCH_ON_DISABLED)) {
      disabled_confirmed = true;
      break;
    }
    if (is_fault_.load()) {
      RCLCPP_ERROR(
        logger_, "Drive fault while waiting for zero-point disable (6041h=0x%04X)",
        disabled_statusword);
      return false;
    }
  }
  if (!disabled_confirmed) {
    RCLCPP_ERROR(
      logger_, "Timed out waiting for SWITCH_ON_DISABLED before zero-point write (6041h=0x%04X)",
      disabled_statusword);
    return false;
  }
  RCLCPP_INFO(
    logger_, "Zero point: SWITCH_ON_DISABLED confirmed (6041h=0x%04X)",
    disabled_statusword);

  // LD2 Pr0.15/2015h is only applicable in PP/HM modes.  The normal lift
  // runtime uses PV, so select HM while the drive remains disabled and verify
  // the mode display before issuing the absolute-encoder zero command.
  if (!sdo_write(OBJ_MODE_OF_OPERATION, 0x00, MODE_HOMING, 1, true)) {
    RCLCPP_ERROR(logger_, "Failed to select HM mode before LD2 zero-point write");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  int32_t mode_display_raw = 0;
  if (!sdo_read(OBJ_MODE_OF_OPERATION_DISPLAY, 0x00, mode_display_raw, true) ||
      static_cast<int8_t>(mode_display_raw & 0xFF) != MODE_HOMING)
  {
    RCLCPP_ERROR(
      logger_, "Drive rejected HM mode before zero-point write (6061h=%d)",
      static_cast<int>(static_cast<int8_t>(mode_display_raw & 0xFF)));
    return false;
  }
  RCLCPP_INFO(logger_, "Zero point: HM mode confirmed (6061h=6)");

  uint32_t absolute_encoder_parameter = 0;
  if (!canopen_master_.read_absolute_encoder_parameter(
      absolute_encoder_parameter, static_cast<int>(sdo_timeout_sec_ * 1000.0)))
  {
    RCLCPP_ERROR(logger_, "Cannot read LD2 absolute-encoder parameter 2015h before zeroing");
    return false;
  }
  if (absolute_encoder_parameter == 9U) {
    RCLCPP_ERROR(
      logger_, "LD2 absolute-encoder parameter 2015h remained 9 from a previous command");
    return false;
  }
  if (absolute_encoder_parameter == 0U) {
    // Some LD2 firmware ships with absolute-encoder mode disabled. Enable it
    // explicitly, but require a readback before continuing; firmware that
    // applies this parameter only after a power cycle will fail clearly here.
    if (!canopen_master_.write_absolute_encoder_command(
        1U, static_cast<int>(sdo_timeout_sec_ * 1000.0)))
    {
      RCLCPP_ERROR(logger_, "Failed to enable LD2 absolute-encoder mode (2015h=1)");
      return false;
    }
    bool absolute_encoder_enabled = false;
    for (int poll = 0; poll < 20; ++poll) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (!canopen_master_.read_absolute_encoder_parameter(
          absolute_encoder_parameter, static_cast<int>(sdo_timeout_sec_ * 1000.0)))
      {
        continue;
      }
      if (absolute_encoder_parameter == 1U || absolute_encoder_parameter == 2U) {
        absolute_encoder_enabled = true;
        break;
      }
    }
    if (!absolute_encoder_enabled) {
      RCLCPP_ERROR(
        logger_,
        "LD2 absolute-encoder mode did not become active after writing 2015h=1; power cycle may be required (2015h=%u)",
        absolute_encoder_parameter);
      return false;
    }
  }
  if (absolute_encoder_parameter != 1U && absolute_encoder_parameter != 2U) {
    RCLCPP_ERROR(
      logger_, "Unsupported LD2 absolute-encoder parameter 2015h=%u", absolute_encoder_parameter);
    return false;
  }

  if (!canopen_master_.write_absolute_encoder_command(
      9U, static_cast<int>(sdo_timeout_sec_ * 1000.0))) {
    int32_t error_code = 0;
    if (sdo_read(OBJ_ERROR_CODE, 0x00, error_code, false)) {
      RCLCPP_ERROR(
        logger_, "LD2 zero-point write 2015h=9 failed; 603Fh error=0x%04X",
        static_cast<unsigned int>(error_code & 0xFFFF));
    } else {
      RCLCPP_ERROR(logger_, "LD2 zero-point write 2015h=9 failed; 603Fh read failed");
    }
    return false;
  }
  bool absolute_encoder_command_completed = false;
  uint32_t absolute_encoder_readback = 9U;
  bool absolute_encoder_readback_seen = false;
  for (int poll = 0; poll < 60; ++poll) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!canopen_master_.read_absolute_encoder_parameter(
        absolute_encoder_readback, static_cast<int>(sdo_timeout_sec_ * 1000.0)))
    {
      continue;
    }
    absolute_encoder_readback_seen = true;
    if (absolute_encoder_readback == 1U || absolute_encoder_readback == 2U) {
      absolute_encoder_command_completed = true;
      break;
    }
  }
  if (!absolute_encoder_command_completed) {
    if (absolute_encoder_readback_seen && absolute_encoder_readback == 9U) {
      uint32_t error_code = 0;
      if (canopen_master_.read_error_code(
          error_code, static_cast<int>(sdo_timeout_sec_ * 1000.0))) {
        RCLCPP_ERROR(
          logger_,
          "LD2 zero-point command verification failed: 2015h remained 9; 603Fh=0x%04X",
          static_cast<unsigned int>(error_code & 0xFFFF));
      } else {
        RCLCPP_ERROR(
          logger_, "LD2 zero-point command verification failed: 2015h remained 9; 603Fh read failed");
      }
    } else {
      RCLCPP_ERROR(
        logger_, "LD2 zero-point command verification failed: 2015h readback=%u",
        absolute_encoder_readback);
    }
    return false;
  }
  RCLCPP_INFO(
    logger_, "Zero point: 2015h command completed (2015h=%u)", absolute_encoder_readback);
  if (!canopen_master_.clear_position(static_cast<int>(sdo_timeout_sec_ * 1000.0))) {
    RCLCPP_ERROR(logger_, "LD2 position clear write 2610h=1 failed");
    return false;
  }
  RCLCPP_INFO(logger_, "Zero point: 2610h=1 accepted; waiting for 6064h to reach zero");

  constexpr int32_t zero_tolerance_counts = 10;
  int32_t raw_position = 0;
  int zero_confirm_streak = 0;
  bool zero_confirmed = false;
  for (int poll = 0; poll < 60; ++poll) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (!sdo_read(OBJ_POSITION_ACTUAL, 0x00, raw_position, true)) {
      zero_confirm_streak = 0;
      continue;
    }
    if (std::abs(raw_position) <= zero_tolerance_counts) {
      ++zero_confirm_streak;
      if (zero_confirm_streak >= 2) {
        zero_confirmed = true;
        break;
      }
    } else {
      zero_confirm_streak = 0;
    }
  }
  if (!zero_confirmed) {
    RCLCPP_ERROR(
      logger_, "LD2 zero-point verification timed out: final 6064h=%d", raw_position);
    return false;
  }
  RCLCPP_INFO(logger_, "Zero point: 6064h reached zero (6064h=%d)", raw_position);

  // Return to the normal PV mode before the caller runs the complete
  // CiA402 re-enable sequence.
  if (!sdo_write(OBJ_MODE_OF_OPERATION, 0x00, MODE_VELOCITY, 1, true)) {
    RCLCPP_ERROR(logger_, "Failed to restore PV mode after LD2 zero-point write");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  mode_display_raw = 0;
  if (!sdo_read(OBJ_MODE_OF_OPERATION_DISPLAY, 0x00, mode_display_raw, true) ||
      static_cast<int8_t>(mode_display_raw & 0xFF) != MODE_VELOCITY)
  {
    RCLCPP_ERROR(
      logger_, "Drive rejected PV mode restore after zero-point write (6061h=%d)",
      static_cast<int>(static_cast<int8_t>(mode_display_raw & 0xFF)));
    return false;
  }
  RCLCPP_INFO(logger_, "Zero point: PV mode restored (6061h=3)");

  // 2610h has already made the drive's raw position zero. Persist that
  // canonical value instead of the pre-clear position read at the start.
  zero_raw_position_ = 0;
  zero_verified_by_drive_ = true;
  position_offset_m_.store(0.0);
  return true;
}

bool LiftSlideHardwareInterface::configure_homing_di_mapping()
{
  if (!homing_configure_di_) {
    return true;
  }

  auto sdo_write_retry = [this](uint16_t index, uint8_t subindex, int32_t value, const char * name,
                                int max_retries = 3) -> bool {
      for (int attempt = 0; attempt < max_retries; ++attempt) {
        if (sdo_write(index, subindex, value, 4, false)) {
          RCLCPP_INFO(logger_, "%s (0x%04X)=0x%02X OK", name, index, value & 0xFF);
          return true;
        }
        RCLCPP_WARN(
          logger_, "%s (0x%04X) write failed, retry %d/%d", name, index, attempt + 1, max_retries);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      RCLCPP_ERROR(logger_, "%s (0x%04X) write failed after retries", name, index);
      return false;
    };

  auto log_digital_inputs = [this](const char * label) {
      int32_t digital_inputs = 0;
      if (!sdo_read(OBJ_DIGITAL_INPUTS, 0x00, digital_inputs, false)) {
        RCLCPP_WARN(logger_, "%s: failed to read 0x60FD", label);
        return;
      }
      const auto di = static_cast<uint32_t>(digital_inputs);
      update_limit_switch_state(di);
      RCLCPP_INFO(
        logger_,
        "%s: 0x60FD=0x%08X b7-9(SI4=%d/SI5=%d/SI6=%d) b19-21(DI4=%d/DI5=%d/DI6=%d) b27-29(DI4=%d/DI5=%d/DI6=%d) b0-2(NOT=%d/POT=%d/HOME=%d) map(home=DI%d:%d pot=DI%d:%d not=DI%d:%d)",
        label,
        di,
        (di >> 7) & 1U, (di >> 8) & 1U, (di >> 9) & 1U,
        (di >> 19) & 1U, (di >> 20) & 1U, (di >> 21) & 1U,
        (di >> 27) & 1U, (di >> 28) & 1U, (di >> 29) & 1U,
        (di >> 0) & 1U, (di >> 1) & 1U, (di >> 2) & 1U,
        home_di_channel_, physical_di_channel_active(home_di_channel_) ? 1 : 0,
        pot_di_channel_, physical_di_channel_active(pot_di_channel_) ? 1 : 0,
        not_di_channel_, physical_di_channel_active(not_di_channel_) ? 1 : 0);
    };

  auto recover_fault_if_needed = [this]() {
      int32_t sw_check = 0;
      if (sdo_read(OBJ_STATUSWORD, 0x00, sw_check, false) && ((sw_check & 0x0008) != 0)) {
        RCLCPP_WARN(
          logger_,
          "FAULT detected after DI mapping change (sw=0x%04X), resetting and re-enabling...",
          static_cast<uint16_t>(sw_check & 0xFFFF));
        (void)reset_fault();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (enable_drive()) {
          drive_enabled_.store(true);
        } else {
          RCLCPP_WARN(logger_, "Re-enable after DI mapping change failed");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    };

  const std::array<uint16_t, 6> di_indices = {0x2400, 0x2401, 0x2402, 0x2403, 0x2404, 0x2405};
  const std::array<const char *, 6> di_names = {
    "DI1(0x2400)", "DI2(0x2401)", "DI3(0x2402)", "DI4(0x2403)", "DI5(0x2404)", "DI6(0x2405)"};
  std::array<int32_t, 6> di_values{};

  for (size_t i = 0; i < di_indices.size(); ++i) {
    if (sdo_read(di_indices[i], 0x00, di_values[i], false)) {
      RCLCPP_INFO(logger_, "Current %s = 0x%02X", di_names[i], di_values[i] & 0xFF);
    } else {
      RCLCPP_WARN(logger_, "Read %s failed", di_names[i]);
      di_values[i] = 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  log_digital_inputs("Before DI config");

  homing_di4_config_applied_ = false;
  homing_di5_config_applied_ = false;
  homing_di6_config_applied_ = false;

  const std::array<int, 3> mapped_channels = {home_di_channel_, pot_di_channel_, not_di_channel_};
  RCLCPP_INFO(
    logger_,
    "Using DI mapping: HOME=DI%d(0x%04X) POT=DI%d(0x%04X) NOT=DI%d(0x%04X)",
    home_di_channel_, di_config_index_for_channel(home_di_channel_),
    pot_di_channel_, di_config_index_for_channel(pot_di_channel_),
    not_di_channel_, di_config_index_for_channel(not_di_channel_));

  // 清理未参与本次回零映射的残留功能，避免与选中的 HOME/POT/NOT 冲突。
  for (size_t i = 0; i < di_indices.size(); ++i) {
    const int channel = static_cast<int>(i) + 1;
    if (std::find(mapped_channels.begin(), mapped_channels.end(), channel) != mapped_channels.end()) {
      continue;
    }
    if (di_values[i] == 0) {
      continue;
    }
    const std::string clear_name = std::string(di_names[i]) + "=CLEAR";
    RCLCPP_WARN(logger_, "%s has residual config 0x%02X, clearing to 0x00...", di_names[i], di_values[i] & 0xFF);
    if (!sdo_write_retry(di_indices[i], 0x00, 0x00, clear_name.c_str())) {
      RCLCPP_WARN(logger_, "Failed to clear %s, continue with selected DI mapping", di_names[i]);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  recover_fault_if_needed();

  const auto home_index = di_config_index_for_channel(home_di_channel_);
  const auto pot_index = di_config_index_for_channel(pot_di_channel_);
  const auto not_index = di_config_index_for_channel(not_di_channel_);
  const std::string home_name = "DI" + std::to_string(home_di_channel_) + "=HOME";
  const std::string pot_name = "DI" + std::to_string(pot_di_channel_) + "=POT";
  const std::string not_name = "DI" + std::to_string(not_di_channel_) + "=NOT";

  const bool di4_ok = sdo_write_retry(home_index, 0x00, di4_homing_func_, home_name.c_str());
  if (di4_ok) {
    homing_di4_config_applied_ = true;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const bool di5_ok = sdo_write_retry(pot_index, 0x00, di5_pot_func_, pot_name.c_str());
  if (di5_ok) {
    homing_di5_config_applied_ = true;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const bool di6_ok = sdo_write_retry(not_index, 0x00, di6_not_func_, not_name.c_str());
  if (di6_ok) {
    homing_di6_config_applied_ = true;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  recover_fault_if_needed();
  log_digital_inputs("After DI config");

  return di4_ok && di5_ok && di6_ok;
}

bool LiftSlideHardwareInterface::read_homing_feedback(
  LiftSlideHardwareInterface::HomingFeedback & feedback)
{
  int32_t position_counts = 0;
  int32_t velocity_counts_per_sec = 0;
  int32_t statusword_raw = 0;

  if (!sdo_read(OBJ_POSITION_ACTUAL, 0x00, position_counts, true)) {
    return false;
  }
  if (!sdo_read(OBJ_VELOCITY_ACTUAL, 0x00, velocity_counts_per_sec, true)) {
    return false;
  }
  if (!sdo_read(OBJ_STATUSWORD, 0x00, statusword_raw, false)) {
    return false;
  }

  feedback.physical_position_m = position_counts_to_m(position_counts);
  feedback.user_position_m = feedback.physical_position_m - position_offset_m_.load();
  feedback.velocity_mps = velocity_counts_per_sec_to_mps(velocity_counts_per_sec);
  feedback.statusword = static_cast<uint16_t>(statusword_raw & 0xFFFF);
  feedback.fault = (feedback.statusword & 0x0008U) != 0U;

  hw_position_ = feedback.user_position_m;
  hw_velocity_ = feedback.velocity_mps;
  last_statusword_.store(feedback.statusword);

  int32_t digital_inputs = 0;
  if (sdo_read(OBJ_DIGITAL_INPUTS, 0x00, digital_inputs, false)) {
    update_limit_switch_state(static_cast<uint32_t>(digital_inputs));
  }

  feedback.digital_inputs_raw = last_digital_inputs_raw_;
  feedback.switch_feedback_valid = limit_switch_state_valid_;

  const bool lower_switch_actual = switch_channel_active(pot_di_channel_);
  const bool home_switch_actual = switch_channel_active(home_di_channel_);
  const bool upper_switch_actual = switch_channel_active(not_di_channel_);

  if (feedback.switch_feedback_valid) {
    feedback.lower_switch = lower_switch_actual;
    feedback.home_switch = home_switch_actual;
    feedback.upper_switch = upper_switch_actual;
    return true;
  }

  bool lower_switch_estimated = false;
  bool home_switch_estimated = false;
  bool upper_switch_estimated = false;
  estimate_switch_states(
    feedback.physical_position_m,
    lower_switch_estimated,
    home_switch_estimated,
    upper_switch_estimated);

  feedback.lower_switch = lower_switch_estimated;
  feedback.home_switch = home_switch_estimated;
  feedback.upper_switch = upper_switch_estimated;
  return true;
}

bool LiftSlideHardwareInterface::send_velocity_command_direct(double velocity_mps)
{
  const double command = clamp(velocity_mps, -max_velocity_mps_, max_velocity_mps_);
  int32_t target_counts_per_sec = velocity_mps_to_counts_per_sec(command);
  if (invert_command_) {
    target_counts_per_sec = -target_counts_per_sec;
  }

  if (target_counts_per_sec == last_target_velocity_counts_per_sec_.load()) {
    return true;
  }

  if (!sdo_write(OBJ_TARGET_VELOCITY, 0x00, target_counts_per_sec, 4, true)) {
    return false;
  }
  if (!sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false)) {
    return false;
  }

  hw_command_velocity_ = command;
  last_sent_velocity_command_ = command;
  last_target_velocity_counts_per_sec_.store(target_counts_per_sec);
  return true;
}

bool LiftSlideHardwareInterface::stop_velocity_motion_direct()
{
  if (!send_velocity_command_direct(0.0)) {
    return false;
  }
  hw_velocity_ = 0.0;
  hw_command_velocity_ = 0.0;
  last_sent_velocity_command_ = 0.0;
  return true;
}

void LiftSlideHardwareInterface::estimate_switch_states(
  double physical_position_m,
  bool & lower_switch,
  bool & home_switch,
  bool & upper_switch) const
{
  const double tolerance = std::max(0.001, switch_position_tolerance_m_);
  lower_switch = physical_position_m <= (lower_switch_position_m_ + tolerance);
  home_switch = std::abs(physical_position_m - home_switch_position_m_) <= tolerance;
  upper_switch = physical_position_m >= (upper_switch_position_m_ - tolerance);
}

bool LiftSlideHardwareInterface::start_homing_sequence()
{
  auto stop_motion_state = [this]() {
      hw_command_velocity_ = 0.0;
      last_sent_velocity_command_ = 0.0;
      last_target_velocity_counts_per_sec_.store(0);
      hw_velocity_ = 0.0;
    };

  auto fail = [this, &stop_motion_state](const std::string & reason, bool restore_drive) -> bool {
      RCLCPP_ERROR(logger_, "Homing failed: %s", reason.c_str());
      (void)stop_velocity_motion_direct();
      stop_motion_state();
      const bool can_restore =
        restore_drive && !shutdown_requested_.load() && !quick_stop_active_.load();
      if (can_restore && !restore_velocity_mode()) {
        RCLCPP_WARN(logger_, "Failed to restore velocity mode after homing failure");
      }
      // Re-configure all DI mappings (HOME/POT/NOT) so lamps work after failure
      (void)clear_homing_di_configuration();
      if (can_restore && homing_configure_di_) {
        if (!configure_homing_di_mapping()) {
          RCLCPP_WARN(logger_, "Failed to restore runtime DI mapping after homing failure");
        }
      }
      feedback_poll_paused_.store(false);
      drive_enabled_.store(can_restore);
      return false;
    };

  set_homing_state(static_cast<int>(HSC::IN_PROGRESS), static_cast<int>(HD::CONFIGURING));
  feedback_poll_paused_.store(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  int32_t statusword_raw = 0;
  if (sdo_read(OBJ_STATUSWORD, 0x00, statusword_raw, false)) {
    const uint16_t statusword = static_cast<uint16_t>(statusword_raw & 0xFFFF);
    last_statusword_.store(statusword);
    if ((statusword & 0x0008U) != 0U) {
      return fail("motor in fault state before zeroing", false);
    }
    if ((statusword & 0x006FU) != 0x0027U) {
      RCLCPP_WARN(
        logger_,
        "Motor not in OPERATION_ENABLED (sw=0x%04X), attempting re-enable...", statusword);
      if (!enable_drive()) {
        return fail("re-enable drive failed before zeroing", false);
      }
      drive_enabled_.store(true);
    }
  }

  if (!drive_enabled_.load()) {
    if (!enable_drive()) {
      return fail("enable drive failed", false);
    }
    drive_enabled_.store(true);
  }

  if (!configure_homing_di_mapping()) {
    return fail("configure DI mapping failed", true);
  }

  if (!configure_drive()) {
    return fail("set velocity mode failed", true);
  }
  if (!sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false)) {
    return fail("enable operation failed", true);
  }

  // Keep feedback_poll_paused_ = true during homing to avoid CAN bus
  // contention with async_read_loop.  read_homing_feedback() does its own
  // SDO reads, so the async loop is not needed here.

  LiftSlideHardwareInterface::HomingFeedback feedback;
  if (!read_homing_feedback(feedback)) {
    return fail("initial feedback read failed", true);
  }
  if (!feedback.switch_feedback_valid) {
    return fail("digital input feedback unavailable", true);
  }

  const double speed = std::clamp(std::abs(homing_speed_mps_), 0.001, max_velocity_mps_);
  const double configured_travel_m =
    std::abs(upper_switch_position_m_ - lower_switch_position_m_);
  const double phase_timeout_sec = std::max(
    homing_timeout_sec_, configured_travel_m / speed * 1.5 + 5.0);
  auto phase_start_time = std::chrono::steady_clock::now();
  bool midpoint_fallback = false;

  bool search_down_after_upper = feedback.upper_switch;
  bool home_found = false;
  double upper_endpoint_position = feedback.physical_position_m;

  // HOME is the primary zero reference.  Confirm it before moving so a lift
  // that is already at HOME does not travel to an unrelated hard limit.
  if (feedback.home_switch) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!read_homing_feedback(feedback)) {
      return fail("confirm initial HOME feedback failed", true);
    }
    if (feedback.fault || !feedback.switch_feedback_valid) {
      return fail("initial HOME feedback is invalid", true);
    }
    if (feedback.home_switch) {
      home_found = true;
      zero_source_ = "home_sensor";
      RCLCPP_INFO(
        logger_, "HOME already active; setting zero without motion: raw=0x%08X",
        feedback.digital_inputs_raw);
    }
  }

  // If HOME is not currently active, search upward first.  HOME is checked
  // before the upper hard limit, so a lift below HOME stops at HOME instead
  // of unnecessarily travelling to the upper limit.
  if (!home_found && !search_down_after_upper) {
    phase_start_time = std::chrono::steady_clock::now();
    set_homing_state(static_cast<int>(HSC::IN_PROGRESS), static_cast<int>(HD::MOVING_UP_TO_HOME));
    if (!send_velocity_command_direct(speed)) {
      return fail("start upward HOME search failed", true);
    }
    int home_true_streak = 0;

    while (homing_in_progress_.load() && !shutdown_requested_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      if (!read_homing_feedback(feedback)) {
        continue;
      }
      if (feedback.fault) {
        return fail("fault during upward HOME search", true);
      }
      if (!feedback.switch_feedback_valid) {
        return fail("digital input feedback lost", true);
      }

      if (feedback.home_switch) {
        ++home_true_streak;
        if (home_true_streak >= 2) {
          home_found = true;
          zero_source_ = "home_sensor";
          RCLCPP_INFO(
            logger_,
            "HOME detected while moving up: raw=0x%08X home=%d upper=%d lower=%d",
            feedback.digital_inputs_raw,
            feedback.home_switch ? 1 : 0,
            feedback.upper_switch ? 1 : 0,
            feedback.lower_switch ? 1 : 0);
          break;
        }
      } else {
        home_true_streak = 0;
      }

      if (feedback.upper_switch) {
        search_down_after_upper = true;
        upper_endpoint_position = feedback.physical_position_m;
        RCLCPP_INFO(
          logger_,
          "Upper hard limit established; starting downward HOME search: raw=0x%08X",
          feedback.digital_inputs_raw);
        break;
      }

      const double elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - phase_start_time).count();
      if (elapsed_sec > phase_timeout_sec) {
        return fail("timeout during upward HOME search", true);
      }
    }

    if (!homing_in_progress_.load()) {
      return fail("aborted externally", true);
    }
    if (shutdown_requested_.load()) {
      return fail("aborted by shutdown", false);
    }
  }

  if (!home_found && search_down_after_upper) {
      if (!stop_velocity_motion_direct()) {
        return fail("stop before downward HOME search failed", true);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      // After hitting the upper limit (NOT), the drive may have left
      // OPERATION_ENABLED.  Re-read statusword and re-enable if needed.
      {
        int32_t sw_raw = 0;
        if (sdo_read(OBJ_STATUSWORD, 0x00, sw_raw, false)) {
          const uint16_t sw = static_cast<uint16_t>(sw_raw & 0xFFFF);
          update_cia402_state(sw);
          if ((sw & 0x006FU) != 0x0027U) {
            RCLCPP_WARN(
              logger_,
              "Drive not in OPERATION_ENABLED (sw=0x%04X) after upper limit, re-enabling...", sw);
            if (!enable_drive()) {
              return fail("re-enable drive after upper limit failed", true);
            }
            drive_enabled_.store(true);
            // Re-set velocity mode and enable operation
            if (!sdo_write(OBJ_MODE_OF_OPERATION, 0x00, MODE_VELOCITY, 1, true)) {
              return fail("restore velocity mode after upper limit failed", true);
            }
            if (!sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false)) {
              return fail("re-enable operation after upper limit failed", true);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
        }
      }

      // Reset velocity tracking so next command is not filtered as duplicate
      last_target_velocity_counts_per_sec_.store(std::numeric_limits<int32_t>::min());

      set_homing_state(static_cast<int>(HSC::IN_PROGRESS), static_cast<int>(HD::MOVING_DOWN_TO_HOME));
      phase_start_time = std::chrono::steady_clock::now();
      if (!send_velocity_command_direct(-speed)) {
        return fail("start downward motion failed", true);
      }
      bool home_released_after_move = !feedback.home_switch;
      int home_true_streak = 0;

      while (homing_in_progress_.load() && !shutdown_requested_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        if (!read_homing_feedback(feedback)) {
          continue;
        }
        if (feedback.fault) {
          return fail("fault during downward zeroing motion", true);
        }
        if (!feedback.switch_feedback_valid) {
          return fail("digital input feedback lost", true);
        }

        if (feedback.home_switch) {
          if (!home_released_after_move) {
            continue;
          }
          ++home_true_streak;
          if (home_true_streak < 2) {
            continue;
          }
          home_found = true;
          zero_source_ = "home_sensor";
          RCLCPP_INFO(
            logger_,
            "HOME detected while moving down: raw=0x%08X home=%d upper=%d lower=%d",
            feedback.digital_inputs_raw,
            feedback.home_switch ? 1 : 0,
            feedback.upper_switch ? 1 : 0,
            feedback.lower_switch ? 1 : 0);
          break;
        } else {
          home_released_after_move = true;
          home_true_streak = 0;
        }

        if (feedback.lower_switch) {
          const double lower_endpoint_position = feedback.physical_position_m;
          if (std::abs(upper_endpoint_position - lower_endpoint_position) <=
            2.0 * switch_position_tolerance_m_)
          {
            return fail("hard-limit endpoints are too close for midpoint fallback", true);
          }
          const double midpoint =
            0.5 * (upper_endpoint_position + lower_endpoint_position);
          if (!stop_velocity_motion_direct()) {
            return fail("stop at lower limit before midpoint fallback failed", true);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          RCLCPP_ERROR(
            logger_,
            "HOME was not detected between hard limits; moving to midpoint %.4f m as fallback",
            midpoint);

          const double direction = midpoint > feedback.physical_position_m ? speed : -speed;
          if (std::abs(midpoint - feedback.physical_position_m) > switch_position_tolerance_m_ &&
            !send_velocity_command_direct(direction))
          {
            return fail("start midpoint fallback motion failed", true);
          }
          phase_start_time = std::chrono::steady_clock::now();
          while (homing_in_progress_.load() && !shutdown_requested_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (!read_homing_feedback(feedback)) {
              continue;
            }
            if (feedback.fault || !feedback.switch_feedback_valid) {
              return fail("feedback fault during midpoint fallback", true);
            }
            if ((direction > 0.0 && feedback.upper_switch) ||
              (direction < 0.0 && feedback.lower_switch))
            {
              return fail("hard limit reached during midpoint fallback", true);
            }
            if (std::abs(feedback.physical_position_m - midpoint) <= switch_position_tolerance_m_) {
              break;
            }
            const double elapsed_sec =
              std::chrono::duration<double>(std::chrono::steady_clock::now() - phase_start_time).count();
            if (elapsed_sec > phase_timeout_sec) {
              return fail("timeout during midpoint fallback", true);
            }
          }
          if (!stop_velocity_motion_direct()) {
            return fail("stop midpoint fallback motion failed", true);
          }
          midpoint_fallback = true;
          zero_source_ = "hard_limit_midpoint";
          home_found = true;
          break;
        }

        const double elapsed_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - phase_start_time).count();
        if (elapsed_sec > phase_timeout_sec) {
          return fail("timeout during downward HOME search", true);
        }
      }

      if (!homing_in_progress_.load()) {
        return fail("aborted externally", true);
      }
      if (shutdown_requested_.load()) {
        return fail("aborted by shutdown", false);
      }
    }

  if (!home_found) {
    return fail("HOME not detected", true);
  }

  if (!stop_velocity_motion_direct()) {
    return fail("stop motion failed", true);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  LiftSlideHardwareInterface::HomingFeedback final_feedback;
  if (read_homing_feedback(final_feedback)) {
    feedback = final_feedback;
  }

  if (!set_drive_zero_point()) {
    return fail("LD2 hardware zero-point write/verification failed", true);
  }
  position_offset_m_.store(0.0);
  stop_motion_state();

  // Keep DI4 (HOME) mapping active for real-time HOME lamp updates.  The
  // zero-point write disabled the drive, so use the complete CiA402 enable
  // sequence instead of writing 0x000F directly from Switch-on-disabled.
  if (!configure_drive()) {
    return fail("configure velocity mode after zeroing failed", true);
  }
  if (!enable_drive()) {
    return fail("enable drive after zeroing failed", false);
  }
  active_mode_.store(MODE_VELOCITY);
  current_mode_of_operation_.store(MODE_VELOCITY);
  drive_enabled_.store(true);

  // Verify that re-enabling did not invalidate the hardware zero point, then
  // publish the actual encoder reading rather than an assumed zero cache.
  int32_t final_position_counts = 0;
  if (!sdo_read(OBJ_POSITION_ACTUAL, 0x00, final_position_counts, true)) {
    return fail("read 6064h after re-enable failed", true);
  }
  // Re-enabling the LD2 can introduce a small encoder quantisation offset
  // (the observed stable value is about 119 counts = 0.06 mm). Keep the
  // post-enable check strict enough to catch real motion, but do not reject a
  // valid drive zero because of this repeatable readback offset.
  constexpr int32_t zero_reenable_tolerance_counts = 1000;
  if (std::abs(final_position_counts) > zero_reenable_tolerance_counts) {
    return fail(
      std::string("final 6064h verification after re-enable failed: value=") +
      std::to_string(final_position_counts), true);
  }
  async_position_.store(position_counts_to_m(final_position_counts));
  hw_position_ = position_counts_to_m(final_position_counts) - position_offset_m_.load();
  hw_velocity_ = 0.0;
  RCLCPP_INFO(
    logger_, "Zero point: final 6064h verified after re-enable (6064h=%d)",
    final_position_counts);

  last_target_velocity_counts_per_sec_.store(std::numeric_limits<int32_t>::min());
  feedback_poll_paused_.store(false);

  RCLCPP_INFO(
    logger_,
    "Zero set in LD2: source=%s fallback=%d physical_before=%.6f raw=0x%08X",
    zero_source_.c_str(), midpoint_fallback ? 1 : 0,
    feedback.physical_position_m,
    feedback.digital_inputs_raw);
  motion_ready_.store(true);
  reference_valid_.store(true);
  encoder_reference_lost_.store(false);
  zero_verified_by_drive_ = true;
  if (!calibration_file_path_.empty() && !save_calibration()) {
    return fail("zero point saved in drive but calibration file could not be written", true);
  }
  return true;
}

bool LiftSlideHardwareInterface::return_to_home_sequence()
{
  auto stop_motion_state = [this]() {
      hw_command_velocity_ = 0.0;
      last_sent_velocity_command_ = 0.0;
      last_target_velocity_counts_per_sec_.store(0);
      hw_velocity_ = 0.0;
    };

  auto fail = [this, &stop_motion_state](const std::string & reason, bool restore_drive) -> bool {
      RCLCPP_ERROR(logger_, "Return home failed: %s", reason.c_str());
      (void)sdo_write(
        OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION | 0x0100, 2, false);
      stop_motion_state();
      const bool can_restore =
        restore_drive && !shutdown_requested_.load() && !quick_stop_active_.load();
      if (can_restore && !restore_velocity_mode()) {
        RCLCPP_WARN(logger_, "Failed to restore velocity mode after return home failure");
      } else if (can_restore) {
        active_mode_.store(MODE_VELOCITY);
        current_mode_of_operation_.store(MODE_VELOCITY);
      }
      feedback_poll_paused_.store(false);
      drive_enabled_.store(can_restore);
      return false;
    };

  if (!homing_complete_.load() || !zero_verified_by_drive_) {
    RCLCPP_ERROR(
      logger_,
      "Return home rejected: no verified LD2 zero point is available; run set-zero first");
    return false;
  }

  set_homing_state(static_cast<int>(HSC::IN_PROGRESS), static_cast<int>(HD::CONFIGURING));
  feedback_poll_paused_.store(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Check drive state
  int32_t statusword_raw = 0;
  if (sdo_read(OBJ_STATUSWORD, 0x00, statusword_raw, false)) {
    const uint16_t statusword = static_cast<uint16_t>(statusword_raw & 0xFFFF);
    last_statusword_.store(statusword);
    if ((statusword & 0x0008U) != 0U) {
      return fail("motor in fault state", false);
    }
    if ((statusword & 0x006FU) != 0x0027U) {
      RCLCPP_WARN(logger_, "Motor not in OPERATION_ENABLED (sw=0x%04X), re-enabling...", statusword);
      if (!enable_drive()) {
        return fail("re-enable drive failed", false);
      }
      drive_enabled_.store(true);
    }
  }

  if (!drive_enabled_.load()) {
    if (!enable_drive()) {
      return fail("enable drive failed", false);
    }
    drive_enabled_.store(true);
  }

  // Keep the physical hard-limit mappings active while returning to the
  // already-established drive zero.  Return-home never searches for HOME and
  // never changes zero calibration.
  if (!configure_homing_di_mapping()) {
    return fail("configure DI mapping failed", true);
  }

  HomingFeedback feedback;
  if (!read_homing_feedback(feedback)) {
    return fail("initial feedback read failed", true);
  }
  if (!feedback.switch_feedback_valid) {
    return fail("digital input feedback unavailable", true);
  }

  const double speed = std::clamp(std::abs(homing_speed_mps_), 0.001, max_velocity_mps_);
  const double initial_position = feedback.user_position_m;
  const auto start_time = std::chrono::steady_clock::now();

  if (std::abs(initial_position) <= switch_position_tolerance_m_) {
    RCLCPP_INFO(logger_, "Return home: already at drive zero (position=%.4f)", initial_position);
    stop_motion_state();
    if (!restore_velocity_mode()) {
      return fail("restore velocity mode failed when already at zero", true);
    }
    active_mode_.store(MODE_VELOCITY);
    current_mode_of_operation_.store(MODE_VELOCITY);
    drive_enabled_.store(true);
    last_target_velocity_counts_per_sec_.store(std::numeric_limits<int32_t>::min());
    feedback_poll_paused_.store(false);
    return true;
  }

  const double direction = initial_position > 0.0 ? -1.0 : 1.0;
  const int direction_detail = direction > 0.0
    ? static_cast<int>(HD::MOVING_UP_TO_HOME)
    : static_cast<int>(HD::MOVING_DOWN_TO_HOME);
  set_homing_state(static_cast<int>(HSC::IN_PROGRESS), direction_detail);

  if (!set_mode_of_operation(MODE_POSITION)) {
    return fail("set profile-position mode failed", true);
  }
  active_mode_.store(MODE_POSITION);
  current_mode_of_operation_.store(MODE_POSITION);
  if (!start_position_move(0.0, speed)) {
    return fail("start absolute move to drive zero failed", true);
  }

  while (homing_in_progress_.load() && !shutdown_requested_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    if (!read_homing_feedback(feedback)) {
      continue;
    }
    if (feedback.fault) {
      return fail("fault during return home motion", true);
    }
    if (!feedback.switch_feedback_valid) {
      return fail("digital input feedback lost", true);
    }

    if ((direction > 0.0 && feedback.upper_switch) ||
      (direction < 0.0 && feedback.lower_switch))
    {
      return fail("hard limit reached before drive zero", true);
    }

    if (std::abs(feedback.user_position_m) <= switch_position_tolerance_m_) {
      break;
    }

    const double elapsed_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    if (elapsed_sec > homing_timeout_sec_) {
      return fail("timeout during return home", true);
    }
  }

  if (!homing_in_progress_.load()) {
    return fail("aborted externally", true);
  }
  if (shutdown_requested_.load()) {
    return fail("aborted by shutdown", false);
  }

  if (!sdo_write(
      OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION | 0x0100, 2, false))
  {
    return fail("halt at drive zero failed", true);
  }
  stop_motion_state();
  hw_position_ = 0.0;
  hw_velocity_ = 0.0;

  if (!restore_velocity_mode()) {
    return fail("restore velocity mode after return home failed", true);
  }
  active_mode_.store(MODE_VELOCITY);
  current_mode_of_operation_.store(MODE_VELOCITY);
  drive_enabled_.store(true);
  // Reset velocity tracking so the next write() command is not filtered as duplicate
  last_target_velocity_counts_per_sec_.store(std::numeric_limits<int32_t>::min());
  feedback_poll_paused_.store(false);

  RCLCPP_INFO(logger_, "Return home completed at verified LD2 zero");
  return true;
}

bool LiftSlideHardwareInterface::enable_drive()
{
  // Check and clear fault first
  int32_t sw_raw = 0;
  if (sdo_read(OBJ_STATUSWORD, 0x00, sw_raw, false)) {
    if ((sw_raw & 0x0008) != 0) {
      if (!reset_fault()) {
        RCLCPP_WARN(logger_, "Fault reset failed in enable_drive");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  constexpr int max_retries = 5;
  for (int retry = 0; retry < max_retries; ++retry) {
    // Step 0: Disable Voltage (reset state machine)
    (void)sdo_write(OBJ_CONTROLWORD, 0x00, CMD_DISABLE_VOLTAGE, 2, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Step 1: Shutdown -> READY_TO_SWITCH_ON
    if (!sdo_write(OBJ_CONTROLWORD, 0x00, CMD_SHUTDOWN, 2, false)) {
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Step 2: Switch On -> SWITCHED_ON
    if (!sdo_write(OBJ_CONTROLWORD, 0x00, CMD_SWITCH_ON, 2, false)) {
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Step 3: Enable Operation -> OPERATION_ENABLED
    if (!sdo_write(OBJ_CONTROLWORD, 0x00, CMD_ENABLE_OPERATION, 2, false)) {
      continue;
    }

    // Poll statusword until OPERATION_ENABLED.
    // This drive needs ~100ms for power stage initialization after Enable Operation.
    for (int poll = 0; poll < 20; ++poll) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (sdo_read(OBJ_STATUSWORD, 0x00, sw_raw, false)) {
        update_cia402_state(static_cast<uint16_t>(sw_raw & 0xFFFF));
        if (cia402_state_.load() == static_cast<int>(Cia402State::OPERATION_ENABLED)) {
          is_enabled_.store(true);
          return true;
        }
      }
    }
    RCLCPP_WARN(logger_, "enable_drive retry %d: state=%s (sw=0x%04X)",
      retry, cia402_state_name().c_str(), sw_raw & 0xFFFF);
  }
  RCLCPP_ERROR(logger_, "enable_drive failed after %d retries", max_retries);
  return false;
}

bool LiftSlideHardwareInterface::disable_drive()
{
  return sdo_write(OBJ_CONTROLWORD, 0x00, CMD_DISABLE_VOLTAGE, 2, false);
}

bool LiftSlideHardwareInterface::reset_fault()
{
  // 3-step fault reset: clear -> rising edge on bit7 -> clear
  (void)sdo_write(OBJ_CONTROLWORD, 0x00, 0x0000, 2, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!sdo_write(OBJ_CONTROLWORD, 0x00, CMD_FAULT_RESET, 2, false)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  (void)sdo_write(OBJ_CONTROLWORD, 0x00, 0x0000, 2, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Verify fault cleared
  int32_t sw_raw = 0;
  if (sdo_read(OBJ_STATUSWORD, 0x00, sw_raw, false)) {
    update_cia402_state(static_cast<uint16_t>(sw_raw & 0xFFFF));
    return !is_fault_.load();
  }
  return true;
}

bool LiftSlideHardwareInterface::configure_drive()
{
  if (!sdo_write(OBJ_MODE_OF_OPERATION, 0x00, MODE_VELOCITY, 1, true)) {
    return false;
  }
  if (!sdo_write(OBJ_TARGET_VELOCITY, 0x00, 0, 4, true)) {
    return false;
  }
  return apply_profile_acceleration_deceleration(
    profile_acceleration_.load(),
    profile_deceleration_.load());
}

bool LiftSlideHardwareInterface::apply_profile_acceleration_deceleration(
  int acceleration_counts,
  int deceleration_counts)
{
  if (!sdo_write(OBJ_PROFILE_ACCELERATION, 0x00, acceleration_counts, 4, false)) {
    return false;
  }
  if (!sdo_write(OBJ_PROFILE_DECELERATION, 0x00, deceleration_counts, 4, false)) {
    return false;
  }

  return true;
}

bool LiftSlideHardwareInterface::read_feedback_once()
{
  int32_t position_counts = 0;
  int32_t velocity_counts_per_sec = 0;
  int32_t statusword = 0;

  if (!sdo_read(OBJ_POSITION_ACTUAL, 0x00, position_counts, true)) {
    return false;
  }
  if (!sdo_read(OBJ_VELOCITY_ACTUAL, 0x00, velocity_counts_per_sec, true)) {
    return false;
  }
  if (!sdo_read(OBJ_STATUSWORD, 0x00, statusword, false)) {
    return false;
  }

  hw_position_ = position_counts_to_m(position_counts) - position_offset_m_.load();
  hw_velocity_ = velocity_counts_per_sec_to_mps(velocity_counts_per_sec);
  last_statusword_.store(static_cast<uint16_t>(statusword & 0xFFFF));
  update_cia402_state(static_cast<uint16_t>(statusword & 0xFFFF));

  // Update async atomics too
  async_position_.store(position_counts_to_m(position_counts));
  async_velocity_.store(hw_velocity_);
  async_statusword_.store(static_cast<uint16_t>(statusword & 0xFFFF));

  int32_t digital_inputs = 0;
  if (sdo_read(OBJ_DIGITAL_INPUTS, 0x00, digital_inputs, false)) {
    update_limit_switch_state(static_cast<uint32_t>(digital_inputs));
  }
  return true;
}

bool LiftSlideHardwareInterface::simulate_fake_homing_sequence()
{
  auto aborted = [this](const char * reason) -> bool {
    RCLCPP_WARN(logger_, "Fake homing aborted: %s", reason);
    hw_velocity_ = 0.0;
    hw_command_velocity_ = 0.0;
    last_sent_velocity_command_ = 0.0;
    last_target_velocity_counts_per_sec_.store(0);
    return false;
  };

  auto check_abort = [this]() -> bool {
    return !homing_in_progress_.load() || shutdown_requested_.load() || quick_stop_active_.load();
  };

  auto update_fake_switches = [this]() {
    const double physical_pos = homing_complete_.load()
      ? (hw_position_ + home_switch_position_m_) : hw_position_;
    bool lower_sw = false, home_sw = false, upper_sw = false;
    estimate_switch_states(physical_pos, lower_sw, home_sw, upper_sw);
    uint32_t raw = 0U;
    const auto set_fake_physical_di = [this, &raw](int channel, bool active) {
        const bool level_high = di_active_low_ ? !active : active;
        const uint32_t bit = 24U + static_cast<uint32_t>(channel - 1);
        if (level_high) {
          raw |= (1U << bit);
        }
      };
    if (upper_sw) { raw |= (1U << 0U); }
    if (lower_sw) { raw |= (1U << 1U); }
    if (home_sw)  { raw |= (1U << 2U); }
    const auto set_fake_si_bit = [this, &raw](int channel, bool active) {
        const bool level_high = di_active_low_ ? !active : active;
        const uint32_t bit = 4U + static_cast<uint32_t>(channel - 1);
        if (level_high) { raw |= (1U << bit); }
      };
    set_fake_si_bit(home_di_channel_, home_sw);
    set_fake_si_bit(pot_di_channel_, lower_sw);
    set_fake_si_bit(not_di_channel_, upper_sw);
    set_fake_physical_di(not_di_channel_, upper_sw);
    set_fake_physical_di(pot_di_channel_, lower_sw);
    set_fake_physical_di(home_di_channel_, home_sw);
    update_limit_switch_state(raw);
  };

  // Simulation speed: 10x real homing speed for faster testing
  const double sim_speed = std::max(0.01, homing_speed_mps_) * 10.0;
  constexpr double step_sec = 0.050;
  constexpr auto step_duration = std::chrono::milliseconds(50);

  // Determine current physical position
  const double current_physical_pos = homing_complete_.load()
    ? (hw_position_ + home_switch_position_m_) : hw_position_;

  set_homing_state(static_cast<int>(HSC::IN_PROGRESS), static_cast<int>(HD::CONFIGURING));
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  if (check_abort()) { return aborted("externally"); }

  bool lower_switch = false;
  bool home_switch = false;
  bool upper_switch = false;
  estimate_switch_states(current_physical_pos, lower_switch, home_switch, upper_switch);
  bool home_found = home_switch;
  bool search_down_after_upper = upper_switch;

  // Match the real state machine: HOME is the preferred zero reference on
  // the upward path.  The upper-limit/downward search is only a fallback
  // when HOME was not found while travelling upward.
  if (!home_found && !search_down_after_upper) {
    set_homing_state(static_cast<int>(HSC::IN_PROGRESS), static_cast<int>(HD::MOVING_UP_TO_HOME));
    while (true) {
      if (check_abort()) { return aborted("externally"); }
      const double physical_pos_now = homing_complete_.load()
        ? (hw_position_ + home_switch_position_m_) : hw_position_;
      estimate_switch_states(physical_pos_now, lower_switch, home_switch, upper_switch);
      if (home_switch) {
        home_found = true;
        break;
      }
      if (upper_switch) {
        search_down_after_upper = true;
        break;
      }
      hw_velocity_ = sim_speed;
      hw_position_ += sim_speed * step_sec;
      update_fake_switches();
      std::this_thread::sleep_for(step_duration);
    }
    hw_velocity_ = 0.0;
  }

  if (!home_found && search_down_after_upper) {
    set_homing_state(static_cast<int>(HSC::IN_PROGRESS), static_cast<int>(HD::MOVING_DOWN_TO_HOME));
    while (true) {
      if (check_abort()) { return aborted("externally"); }
      const double physical_pos_now = homing_complete_.load()
        ? (hw_position_ + home_switch_position_m_) : hw_position_;
      estimate_switch_states(physical_pos_now, lower_switch, home_switch, upper_switch);
      if (home_switch) {
        home_found = true;
        break;
      }
      if (lower_switch) {
        return aborted("HOME not detected before lower limit");
      }
      hw_velocity_ = -sim_speed;
      hw_position_ -= sim_speed * step_sec;
      update_fake_switches();
      std::this_thread::sleep_for(step_duration);
    }
  }

  if (!home_found) {
    return aborted("HOME not detected");
  }

  // Arrived at home switch: set zero
  hw_position_ = 0.0;
  hw_velocity_ = 0.0;
  hw_command_velocity_ = 0.0;
  last_sent_velocity_command_ = 0.0;
  last_target_velocity_counts_per_sec_.store(0);
  {
    uint32_t raw = (1U << 2U);
    const auto set_fake_physical_di = [this, &raw](int channel, bool active) {
        const bool level_high = di_active_low_ ? !active : active;
        const uint32_t bit = 24U + static_cast<uint32_t>(channel - 1);
        if (level_high) {
          raw |= (1U << bit);
        }
      };
    const auto set_fake_si_bit = [this, &raw](int channel, bool active) {
        const bool level_high = di_active_low_ ? !active : active;
        const uint32_t bit = 4U + static_cast<uint32_t>(channel - 1);
        if (level_high) {
          raw |= (1U << bit);
        }
      };
    set_fake_si_bit(home_di_channel_, true);
    set_fake_si_bit(pot_di_channel_, false);
    set_fake_si_bit(not_di_channel_, false);
    set_fake_physical_di(home_di_channel_, true);
    set_fake_physical_di(pot_di_channel_, false);
    set_fake_physical_di(not_di_channel_, false);
    update_limit_switch_state(raw);
  }
  return true;
}

bool LiftSlideHardwareInterface::parse_bool(const std::string & value, bool default_value) const
{
  if (value.empty()) {
    return default_value;
  }
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), ::tolower);
  return lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on";
}

int LiftSlideHardwareInterface::parse_int(const std::string & value, int default_value) const
{
  if (value.empty()) {
    return default_value;
  }
  try {
    return std::stoi(value);
  } catch (...) {
    return default_value;
  }
}

double LiftSlideHardwareInterface::parse_double(const std::string & value, double default_value) const
{
  if (value.empty()) {
    return default_value;
  }
  try {
    return std::stod(value);
  } catch (...) {
    return default_value;
  }
}

double LiftSlideHardwareInterface::clamp(double value, double lower, double upper) const
{
  return std::max(lower, std::min(upper, value));
}

int32_t LiftSlideHardwareInterface::velocity_mps_to_counts_per_sec(double velocity_mps) const
{
  return static_cast<int32_t>(std::llround(velocity_mps * counts_per_meter_));
}

double LiftSlideHardwareInterface::velocity_counts_per_sec_to_mps(int32_t velocity_counts_per_sec) const
{
  if (std::abs(velocity_counts_per_sec) < 50) {
    velocity_counts_per_sec = 0;
  }

  int32_t physical_counts_per_sec = velocity_counts_per_sec;
  if (invert_feedback_) {
    physical_counts_per_sec = -physical_counts_per_sec;
  }
  return static_cast<double>(physical_counts_per_sec) / counts_per_meter_;
}

int32_t LiftSlideHardwareInterface::position_m_to_counts(double position_m) const
{
  int32_t counts = static_cast<int32_t>(std::llround(position_m * counts_per_meter_));
  if (invert_feedback_) {
    counts = -counts;
  }
  return counts;
}

double LiftSlideHardwareInterface::position_counts_to_m(int32_t position_counts) const
{
  int32_t physical_counts = position_counts;
  if (invert_feedback_) {
    physical_counts = -physical_counts;
  }
  return static_cast<double>(physical_counts) / counts_per_meter_;
}

double LiftSlideHardwareInterface::apply_position_limits(double command_velocity_mps) const
{
  // Primary protection: check physical DI limit switches directly.
  // This ensures the motor stops exactly when the switch is triggered,
  // rather than relying on estimated software positions.
  if (limit_switch_state_valid_) {
    if (command_velocity_mps > 0.0 && limit_switch_active(not_di_channel_)) {
      return 0.0;  // Upper limit switch triggered — block upward movement
    }
    if (command_velocity_mps < 0.0 && limit_switch_active(pot_di_channel_)) {
      return 0.0;  // Lower limit switch triggered — block downward movement
    }
  }

  // Secondary protection: user-coordinate software limits.
  if (hw_position_ <= min_position_m_ && command_velocity_mps < 0.0) {
    return 0.0;
  }
  if (hw_position_ >= max_position_m_ && command_velocity_mps > 0.0) {
    return 0.0;
  }
  return command_velocity_mps;
}

}  // namespace lift_slide_driver

// ===== NEW METHOD IMPLEMENTATIONS =====
namespace lift_slide_driver
{

void LiftSlideHardwareInterface::update_cia402_state(uint16_t statusword)
{
  Cia402State state = Cia402State::UNKNOWN;
  const uint16_t low = statusword & 0x006F;

  if ((low & 0x004F) == 0x0000) {
    state = Cia402State::NOT_READY_TO_SWITCH_ON;
  } else if ((low & 0x004F) == 0x0040) {
    state = Cia402State::SWITCH_ON_DISABLED;
  } else if ((low & 0x006F) == 0x0021) {
    state = Cia402State::READY_TO_SWITCH_ON;
  } else if ((low & 0x006F) == 0x0023) {
    state = Cia402State::SWITCHED_ON;
  } else if ((low & 0x006F) == 0x0027) {
    state = Cia402State::OPERATION_ENABLED;
  } else if ((low & 0x006F) == 0x0007) {
    state = Cia402State::QUICK_STOP_ACTIVE;
  } else if ((low & 0x004F) == 0x000F) {
    state = Cia402State::FAULT_REACTION_ACTIVE;
  } else if ((low & 0x004F) == 0x0008) {
    state = Cia402State::FAULT;
  }

  cia402_state_.store(static_cast<int>(state));
  is_enabled_.store(state == Cia402State::OPERATION_ENABLED);
  is_fault_.store(state == Cia402State::FAULT || state == Cia402State::FAULT_REACTION_ACTIVE);
  is_target_reached_.store((statusword & 0x0400) != 0);
}

std::string LiftSlideHardwareInterface::cia402_state_name() const
{
  switch (static_cast<Cia402State>(cia402_state_.load())) {
    case Cia402State::NOT_READY_TO_SWITCH_ON: return "NOT_READY_TO_SWITCH_ON";
    case Cia402State::SWITCH_ON_DISABLED: return "SWITCH_ON_DISABLED";
    case Cia402State::READY_TO_SWITCH_ON: return "READY_TO_SWITCH_ON";
    case Cia402State::SWITCHED_ON: return "SWITCHED_ON";
    case Cia402State::OPERATION_ENABLED: return "OPERATION_ENABLED";
    case Cia402State::QUICK_STOP_ACTIVE: return "QUICK_STOP_ACTIVE";
    case Cia402State::FAULT_REACTION_ACTIVE: return "FAULT_REACTION_ACTIVE";
    case Cia402State::FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

std::string LiftSlideHardwareInterface::mode_name() const
{
  switch (current_mode_of_operation_.load()) {
    case MODE_NO_MODE: return "no_mode";
    case MODE_POSITION: return "position";
    case MODE_VELOCITY: return "velocity";
    case MODE_HOMING: return "homing";
    default: return "unknown";
  }
}

bool LiftSlideHardwareInterface::hold_current_position()
{
  if (use_fake_hardware_ || !canopen_master_.is_open()) {
    hw_velocity_ = 0.0;
    return true;
  }

  if (!drive_enabled_.load() || homing_in_progress_.load()) {
    return false;
  }

  feedback_poll_paused_.store(true);

  auto finish = [this](bool ok) {
    feedback_poll_paused_.store(false);
    return ok;
  };

  int32_t mode_display_raw = 0;
  if (!sdo_read(OBJ_MODE_OF_OPERATION_DISPLAY, 0x00, mode_display_raw, true)) {
    return finish(false);
  }

  const int8_t mode_display = static_cast<int8_t>(mode_display_raw & 0xFF);
  if (mode_display != MODE_POSITION) {
    if (!set_mode_of_operation(MODE_POSITION)) {
      return finish(false);
    }

    if (!sdo_read(OBJ_MODE_OF_OPERATION_DISPLAY, 0x00, mode_display_raw, true)) {
      return finish(false);
    }

    if (static_cast<int8_t>(mode_display_raw & 0xFF) != MODE_POSITION) {
      return finish(false);
    }
  }

  const uint16_t cw_base = CMD_ENABLE_OPERATION;
  if (!sdo_write(OBJ_CONTROLWORD, 0x00, cw_base | 0x0100, 2, false)) {
    return finish(false);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  int32_t actual_counts = 0;
  if (!sdo_read(OBJ_POSITION_ACTUAL, 0x00, actual_counts, true)) {
    return finish(false);
  }

  if (!sdo_write(OBJ_TARGET_POSITION, 0x00, actual_counts, 4, true)) {
    return finish(false);
  }

  if (!sdo_write(OBJ_CONTROLWORD, 0x00, cw_base & ~CW_NEW_SETPOINT, 2, false)) {
    return finish(false);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if (!sdo_write(
        OBJ_CONTROLWORD,
        0x00,
        cw_base | CW_NEW_SETPOINT | CW_CHANGE_IMMEDIATELY,
        2,
        false))
  {
    return finish(false);
  }

  position_move_in_progress_.store(false);
  (void)read_feedback_once();
  return finish(true);
}

bool LiftSlideHardwareInterface::start_position_move(
  double target_position_m, double profile_velocity_mps)
{
  // Convert target to physical counts (add offset back)
  const double physical_target = target_position_m + position_offset_m_.load();
  int32_t target_counts = position_m_to_counts(physical_target);

  // Set profile velocity
  int32_t profile_vel_counts = velocity_mps_to_counts_per_sec(std::abs(profile_velocity_mps));
  if (profile_vel_counts < 1) profile_vel_counts = 1;
  if (!sdo_write(OBJ_PROFILE_VELOCITY, 0x00, profile_vel_counts, 4, false)) {
    RCLCPP_WARN(logger_, "Failed to write profile velocity for position move");
    return false;
  }

  // Write target position
  if (!sdo_write(OBJ_TARGET_POSITION, 0x00, target_counts, 4, true)) {
    RCLCPP_WARN(logger_, "Failed to write target position");
    return false;
  }

  // PP online update: 0x103F keeps absolute positioning with immediate target changes.
  // After this is active, 607A/6081 updates take effect without retriggering bit4.
  if (!sdo_write(OBJ_CONTROLWORD, 0x00,
    CMD_ENABLE_OPERATION | CW_NEW_SETPOINT | CW_CHANGE_IMMEDIATELY | 0x1000, 2, false))
  {
    return false;
  }

  position_move_in_progress_.store(true);
  RCLCPP_DEBUG(logger_, "Position move: target=%.4fm (counts=%d) speed=%.4f m/s",
    target_position_m, target_counts, profile_velocity_mps);
  return true;
}

bool LiftSlideHardwareInterface::halt_position_motion()
{
  if (use_fake_hardware_ || !canopen_master_.is_open()) {
    hw_velocity_ = 0.0;
    return true;
  }

  if (!drive_enabled_.load() || homing_in_progress_.load()) {
    return false;
  }

  // The real-time write path has already asserted HALT through RPDO.  Latch
  // the measured position as the next profile-position target so clearing the
  // HALT bit cannot resume the stale target.
  return hold_current_position();
}

void LiftSlideHardwareInterface::motor_power_off_sequence()
{
  if (use_fake_hardware_ || !canopen_master_.is_open()) return;

  // 1. Halt
  (void)sdo_write(OBJ_CONTROLWORD, 0x00, 0x010F, 2, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // 2. Quick Stop
  (void)send_rpdo1(CMD_QUICK_STOP, 0, current_mode_of_operation_.load());
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // 3. Disable Voltage
  (void)send_rpdo1(CMD_DISABLE_VOLTAGE, 0, current_mode_of_operation_.load());
  (void)sdo_write(OBJ_CONTROLWORD, 0x00, CMD_DISABLE_VOLTAGE, 2, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // 4. NMT Stop
  (void)send_nmt(0x02);
}

void LiftSlideHardwareInterface::async_read_loop()
{
  int cycle_count = 0;
  while (async_read_running_.load() && !shutdown_requested_.load()) {
    if (feedback_poll_paused_.load() || homing_in_progress_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    // Read position every cycle
    int32_t position_counts = 0;
    if (sdo_read(OBJ_POSITION_ACTUAL, 0x00, position_counts, true)) {
      async_position_.store(position_counts_to_m(position_counts));
    }

    // Read velocity every 2 cycles
    if (cycle_count % 2 == 0) {
      int32_t velocity_counts = 0;
      if (sdo_read(OBJ_VELOCITY_ACTUAL, 0x00, velocity_counts, true)) {
        double vel = velocity_counts_per_sec_to_mps(velocity_counts);
        async_velocity_.store(vel);
      }
    }

    // Read statusword + digital inputs every cycle for responsive limit LEDs
    {
      int32_t sw_raw = 0;
      if (sdo_read(OBJ_STATUSWORD, 0x00, sw_raw, false)) {
        async_statusword_.store(static_cast<uint16_t>(sw_raw & 0xFFFF));
        update_cia402_state(static_cast<uint16_t>(sw_raw & 0xFFFF));
      }

      int32_t di_raw = 0;
      if (sdo_read(OBJ_DIGITAL_INPUTS, 0x00, di_raw, false)) {
        async_digital_inputs_.store(static_cast<uint32_t>(di_raw));
      }
    }

    // Read mode of operation display every 5 cycles (changes infrequently)
    if (cycle_count % 5 == 0) {
      int32_t mode_display = 0;
      if (sdo_read(OBJ_MODE_OF_OPERATION_DISPLAY, 0x00, mode_display, true)) {
        current_mode_of_operation_.store(static_cast<int8_t>(mode_display));
      }
    }

    // Motor status is exposed via state interfaces (read by LiftSlideStateController)

    ++cycle_count;
    const double period_ms = 1000.0 / std::max(1.0, feedback_poll_rate_hz_);
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(period_ms)));
  }
}

void LiftSlideHardwareInterface::start_async_read_thread()
{
  if (async_read_running_.load()) return;
  async_read_running_.store(true);
  async_read_thread_ = std::thread(&LiftSlideHardwareInterface::async_read_loop, this);
}

void LiftSlideHardwareInterface::stop_async_read_thread()
{
  async_read_running_.store(false);
  if (async_read_thread_.joinable()) {
    async_read_thread_.join();
  }
}

void LiftSlideHardwareInterface::run_detached(std::function<void()> task)
{
  run_detached(std::move(task), nullptr);
}

void LiftSlideHardwareInterface::run_detached(
  std::function<void()> task,
  std::function<void()> on_complete)
{
  auto alive = alive_flag_;
  auto count = active_detached_count_;
  count->fetch_add(1);
  std::thread([alive, count, task = std::move(task),
               on_complete = std::move(on_complete)]() {
    if (alive && alive->load()) {
      try {
        task();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("LiftSlideHardwareInterface"),
          "Detached task exception: %s", e.what());
      }
    }
    if (on_complete) {
      try { on_complete(); } catch (...) {}
    }
    count->fetch_sub(1);
  }).detach();
}

void LiftSlideHardwareInterface::shutdown_detached_threads()
{
  if (alive_flag_) {
    alive_flag_->store(false);
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (active_detached_count_ && active_detached_count_->load() > 0 &&
         std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (active_detached_count_ && active_detached_count_->load() > 0) {
    RCLCPP_WARN(logger_, "shutdown_detached_threads: %d threads still active after timeout",
      active_detached_count_->load());
  }
}

bool LiftSlideHardwareInterface::save_calibration() const
{
  if (calibration_file_path_.empty()) {
    return false;
  }
  std::ofstream ofs(calibration_file_path_);
  if (!ofs.is_open()) {
    RCLCPP_WARN(logger_, "Cannot save calibration to %s", calibration_file_path_.c_str());
    return false;
  }
  const std::time_t updated_at = std::time(nullptr);
  std::tm updated_at_utc{};
  gmtime_r(&updated_at, &updated_at_utc);
  ofs << "# Lift slide zero-point calibration (auto-generated)\n";
  const bool reference_valid = reference_valid_.load() && zero_verified_by_drive_;
  const bool provisional_recovery = encoder_reference_lost_.load() && !reference_valid;
  ofs << "schema_version: 3\n";
  ofs << "calibrated: " << (reference_valid ? "true" : "false") << "\n";
  ofs << "zero_source: " << zero_source_ << "\n";
  ofs << "raw_zero_position: " << zero_raw_position_ << "\n";
  ofs << "position_offset_m: " << std::fixed << std::setprecision(8)
      << position_offset_m_.load() << "\n";
  ofs << "can_interface: " << can_interface_ << "\n";
  ofs << "node_id: " << node_id_ << "\n";
  ofs << "verified_by_drive: " << (reference_valid ? "true" : "false") << "\n";
  ofs << "motion_ready: " << (motion_ready_.load() ? "true" : "false") << "\n";
  ofs << "reference_valid: " << (reference_valid ? "true" : "false") << "\n";
  ofs << "encoder_reference_lost: " << (provisional_recovery ? "true" : "false") << "\n";
  ofs << "home_di_channel: " << home_di_channel_ << "\n";
  ofs << "lower_di_channel: " << pot_di_channel_ << "\n";
  ofs << "upper_di_channel: " << not_di_channel_ << "\n";
  ofs << "updated_at: \"" << std::put_time(&updated_at_utc, "%Y-%m-%dT%H:%M:%SZ") << "\"\n";
  ofs.close();
  RCLCPP_INFO(
    logger_, "Calibration saved to %s (source=%s, drive_verified=%d)",
    calibration_file_path_.c_str(), zero_source_.c_str(),
    zero_verified_by_drive_ ? 1 : 0);
  return true;
}

bool LiftSlideHardwareInterface::load_calibration()
{
  if (calibration_file_path_.empty()) {
    return false;
  }
  std::ifstream ifs(calibration_file_path_);
  if (!ifs.is_open()) {
    RCLCPP_INFO(logger_, "No calibration file at %s, homing required", calibration_file_path_.c_str());
    return false;
  }
  int schema_version = 0;
  bool calibrated = false;
  std::string zero_source = "none";
  int32_t raw_zero_position = 0;
  double offset = 0.0;
  std::string stored_can_interface;
  int stored_node_id = -1;
  bool verified_by_drive = false;
  bool motion_ready = false;
  bool reference_valid = false;
  bool encoder_reference_lost = false;
  int stored_home_di = -1;
  int stored_lower_di = -1;
  int stored_upper_di = -1;

  const auto trim = [](std::string value) {
      const auto first = value.find_first_not_of(" \t\r\n");
      if (first == std::string::npos) {
        return std::string{};
      }
      const auto last = value.find_last_not_of(" \t\r\n");
      return value.substr(first, last - first + 1);
    };
  const auto as_bool = [&trim](const std::string & value) {
      const std::string normalized = trim(value);
      return normalized == "true" || normalized == "1" || normalized == "yes";
    };

  std::string line;
  while (std::getline(ifs, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const auto separator = line.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, separator));
    const std::string value = trim(line.substr(separator + 1));
    try {
      if (key == "schema_version") schema_version = std::stoi(value);
      else if (key == "calibrated") calibrated = as_bool(value);
      else if (key == "zero_source") zero_source = value;
      else if (key == "raw_zero_position") raw_zero_position = std::stoi(value);
      else if (key == "position_offset_m") offset = std::stod(value);
      else if (key == "can_interface") stored_can_interface = value;
      else if (key == "node_id") stored_node_id = std::stoi(value);
      else if (key == "verified_by_drive") verified_by_drive = as_bool(value);
      else if (key == "motion_ready") motion_ready = as_bool(value);
      else if (key == "reference_valid") reference_valid = as_bool(value);
      else if (key == "encoder_reference_lost") encoder_reference_lost = as_bool(value);
      else if (key == "home_di_channel") stored_home_di = std::stoi(value);
      else if (key == "lower_di_channel") stored_lower_di = std::stoi(value);
      else if (key == "upper_di_channel") stored_upper_di = std::stoi(value);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        logger_, "Invalid calibration field %s: %s", key.c_str(), e.what());
      return false;
    }
  }
  ifs.close();

  if (schema_version != 2 && schema_version != 3) {
    RCLCPP_WARN(logger_, "Calibration schema %d is unsupported; set-zero is required", schema_version);
    return false;
  }
  if (schema_version == 2) {
    motion_ready = true;
    reference_valid = calibrated && verified_by_drive && zero_source != "none";
  }
  const bool valid_record = reference_valid && calibrated && verified_by_drive && zero_source != "none";
  const bool provisional_record = motion_ready && encoder_reference_lost && !reference_valid &&
    zero_source == "encoder_recovery";
  if (!valid_record && !provisional_record) {
    RCLCPP_WARN(
      logger_, "Calibration is absent, legacy, or not drive-verified; set-zero is required");
    return false;
  }
  if (stored_can_interface != can_interface_ || stored_node_id != node_id_) {
    RCLCPP_WARN(
      logger_,
      "Calibration target mismatch: file=%s/node%d runtime=%s/node%d",
      stored_can_interface.c_str(), stored_node_id, can_interface_.c_str(), node_id_);
    return false;
  }
  if (stored_home_di != home_di_channel_ || stored_lower_di != pot_di_channel_ ||
    stored_upper_di != not_di_channel_)
  {
    RCLCPP_WARN(logger_, "Calibration DI mapping mismatch; set-zero is required");
    return false;
  }
  if (std::abs(offset) > 1e-9) {
    RCLCPP_WARN(
      logger_, "Calibration contains a software offset (%.8f); set-zero is required", offset);
    return false;
  }

  position_offset_m_.store(offset);
  zero_source_ = zero_source;
  zero_raw_position_ = raw_zero_position;
  zero_verified_by_drive_ = valid_record;
  reference_valid_.store(valid_record);
  encoder_reference_lost_.store(provisional_record);
  motion_ready_.store(motion_ready);
  homing_complete_.store(valid_record);
  RCLCPP_INFO(logger_,
    "Calibration loaded from %s (source=%s, raw_zero=%d, motion_ready=%d, reference_valid=%d)",
    calibration_file_path_.c_str(), zero_source_.c_str(), zero_raw_position_,
    motion_ready ? 1 : 0, valid_record ? 1 : 0);
  return true;
}

}  // namespace lift_slide_driver

PLUGINLIB_EXPORT_CLASS(
  lift_slide_driver::LiftSlideHardwareInterface,
  hardware_interface::SystemInterface)
