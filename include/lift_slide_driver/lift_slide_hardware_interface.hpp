#ifndef LIFT_SLIDE_DRIVER__LIFT_SLIDE_HARDWARE_INTERFACE_HPP_
#define LIFT_SLIDE_DRIVER__LIFT_SLIDE_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "openflex_can/lift/ld2_canopen_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace lift_slide_driver
{

/// Homing state codes exposed via state interface
enum class HomingStateCode : int {
  IDLE = 0,
  IN_PROGRESS = 1,
  COMPLETED = 2,
  ERROR = 3
};

/// Homing detail codes exposed via state interface
enum class HomingDetail : int {
  NONE = 0,
  FAKE_HARDWARE = 1,
  READY = 2,
  SERVICE_READY = 3,
  ENABLED = 4,
  DISABLED = 5,
  ACCEPTED = 10,
  CONFIGURING = 11,
  MOVING_UP_TO_HOME = 12,
  MOVING_UP_TO_UPPER = 13,
  MOVING_DOWN_TO_HOME = 14,
  OK = 20,
  CALIBRATION_RESTORED = 21,
  SHUTDOWN = 30,
  QUICK_STOP_ACTIVE = 31,
  ENABLE_FAILED = 32,
  FULL_RESET_FAILED = 33,
  FAILED = 34,
  ABORTED_BY_QUICK_STOP = 35
};

/// CiA402 state machine states
enum class Cia402State : int {
  NOT_READY_TO_SWITCH_ON = 0,
  SWITCH_ON_DISABLED = 1,
  READY_TO_SWITCH_ON = 2,
  SWITCHED_ON = 3,
  OPERATION_ENABLED = 4,
  QUICK_STOP_ACTIVE = 5,
  FAULT_REACTION_ACTIVE = 6,
  FAULT = 7,
  UNKNOWN = 8
};

class LiftSlideHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ~LiftSlideHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct HomingFeedback;

  bool init_can_socket();
  void close_can_socket();
  bool send_can_frame(uint32_t can_id, const std::array<uint8_t, 8> & data, uint8_t dlc);
  bool recv_can_frame(struct can_frame & frame, int timeout_ms);
  void drain_rx();

  bool send_nmt(uint8_t command);
  bool sdo_write(uint16_t index, uint8_t subindex, int32_t value, uint8_t size, bool signed_value);
  bool sdo_read(uint16_t index, uint8_t subindex, int32_t & value, bool signed_value);
  bool send_rpdo1(uint16_t controlword, int32_t target_velocity_counts_per_sec, int8_t mode);

  bool enable_drive();
  bool disable_drive();
  bool reset_fault();
  bool configure_drive();
  bool apply_profile_acceleration_deceleration(int acceleration_counts, int deceleration_counts);
  bool read_feedback_once();
  bool configure_homing_di_mapping();
  bool read_homing_feedback(struct HomingFeedback & feedback);
  bool send_velocity_command_direct(double velocity_mps);
  uint16_t di_config_index_for_channel(int channel) const;
  bool physical_di_channel_level_high(int channel) const;
  bool physical_di_channel_active(int channel) const;
  bool switch_channel_active(int channel) const;
  bool limit_switch_active(int channel) const;
  bool stop_velocity_motion_direct();
  bool set_mode_of_operation(int8_t mode);
  bool restore_velocity_mode();
  bool save_calibration() const;
  bool load_calibration();
  bool clear_homing_di_configuration();
  bool trigger_homing_start();
  bool recover_encoder_reference_loss();
  bool set_drive_zero_point();
  bool start_homing_sequence();
  bool return_to_home_sequence();
  bool hold_current_position();
  void update_limit_switch_state(uint32_t digital_inputs_raw);
  void estimate_switch_states(
    double physical_position_m,
    bool & lower_switch,
    bool & home_switch,
    bool & upper_switch) const;

  // CIA402 state machine
  void update_cia402_state(uint16_t statusword);
  std::string cia402_state_name() const;
  std::string mode_name() const;

  // Position control
  bool start_position_move(double target_position_m, double profile_velocity_mps);
  bool halt_position_motion();

  // Motor power-off sequence
  void motor_power_off_sequence();

  // Async read thread
  void async_read_loop();
  void start_async_read_thread();
  void stop_async_read_thread();

  // Detached thread framework
  void run_detached(std::function<void()> task);
  void run_detached(std::function<void()> task,
                    std::function<void()> on_complete);
  void shutdown_detached_threads();

  void join_homing_thread();
  void set_homing_state(int state, int detail);
  bool simulate_fake_homing_sequence();

  // State/command interface bridge
  void update_state_interface_values();
  void process_command_interfaces();

  bool parse_bool(const std::string & value, bool default_value) const;
  int parse_int(const std::string & value, int default_value) const;
  double parse_double(const std::string & value, double default_value) const;
  double clamp(double value, double lower, double upper) const;

  int32_t velocity_mps_to_counts_per_sec(double velocity_mps) const;
  double velocity_counts_per_sec_to_mps(int32_t velocity_counts_per_sec) const;
  int32_t position_m_to_counts(double position_m) const;
  double position_counts_to_m(int32_t position_counts) const;
  double apply_position_limits(double command_velocity_mps) const;

  struct HomingFeedback
  {
    double physical_position_m{0.0};
    double user_position_m{0.0};
    double velocity_mps{0.0};
    uint16_t statusword{0};
    uint32_t digital_inputs_raw{0U};
    bool lower_switch{false};
    bool home_switch{false};
    bool upper_switch{false};
    bool switch_feedback_valid{false};
    bool fault{false};
  };

  rclcpp::Logger logger_{rclcpp::get_logger("LiftSlideHardwareInterface")};

  std::string can_interface_{"can3"};
  int node_id_{16};
  bool use_fake_hardware_{false};
  double counts_per_meter_{2000000.0};
  std::atomic<int> profile_acceleration_{50000};
  std::atomic<int> profile_deceleration_{50000};
  bool invert_command_{true};
  bool invert_feedback_{true};
  double min_position_m_{-0.750};
  double max_position_m_{0.400};
  double max_velocity_mps_{0.10};
  double command_timeout_sec_{0.5};
  double sdo_timeout_sec_{0.2};
  double feedback_poll_rate_hz_{20.0};

  openflex_can::Ld2CanopenDrive canopen_master_;

  // Standard ros2_control state/command interface backing variables
  double hw_position_{0.0};
  double hw_velocity_{0.0};
  double hw_command_velocity_{0.0};
  double hw_position_command_{std::numeric_limits<double>::quiet_NaN()};

  // Extended state interface backing variables (read by LiftSlideStateController)
  double hw_cia402_state_{0.0};
  double hw_statusword_{0.0};
  double hw_is_enabled_{0.0};
  double hw_is_fault_{0.0};
  double hw_is_target_reached_{0.0};
  double hw_mode_of_operation_{3.0};
  double hw_homing_state_{0.0};
  double hw_homing_detail_{0.0};
  double hw_homing_complete_{0.0};
  double hw_is_homing_{0.0};
  double hw_upper_limit_switch_{0.0};
  double hw_home_switch_{0.0};
  double hw_lower_limit_switch_{0.0};
  double hw_limit_switch_valid_{0.0};
  double hw_digital_inputs_raw_{0.0};
  double hw_physical_position_{0.0};
  double hw_profile_speed_mps_{0.010};
  double hw_profile_accel_mps2_{0.025};
  double hw_profile_decel_mps2_{0.025};
  double hw_drive_enabled_{0.0};
  double hw_quick_stop_active_{0.0};
  double hw_motion_ready_{0.0};
  double hw_reference_valid_{0.0};
  double hw_encoder_reference_lost_{0.0};

  // Extended command interface backing variables (written by LiftSlideStateController)
  double hw_enable_cmd_{0.0};         // 0=noop, 1=enable, -1=disable
  double hw_quick_stop_cmd_{0.0};     // 0=noop, 1=trigger
  double hw_reset_fault_cmd_{0.0};    // 0=noop, 1=trigger
  double hw_full_reset_cmd_{0.0};     // 0=noop, 1=trigger
  double hw_homing_cmd_{0.0};         // 0=noop, 1=start_homing, 2=return_home
  double hw_set_mode_cmd_{0.0};       // 0=noop, 1=position, 3=velocity, 6=homing
  double hw_profile_speed_cmd_{std::numeric_limits<double>::quiet_NaN()};
  double hw_accel_time_cmd_{std::numeric_limits<double>::quiet_NaN()};
  double hw_decel_time_cmd_{std::numeric_limits<double>::quiet_NaN()};
  double hw_manual_halt_cmd_{0.0};   // 0=noop, 1=halt while staying enabled
  double hw_manual_limit_stop_cmd_{0.0};  // 0=noop, 1=quick stop and disable

  double last_sent_velocity_command_{0.0};
  double last_sent_position_command_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic<int32_t> last_target_velocity_counts_per_sec_{std::numeric_limits<int32_t>::min()};
  std::atomic<uint16_t> last_statusword_{0};
  std::atomic<bool> drive_enabled_{false};
  std::atomic<double> position_offset_m_{0.0};

  // CIA402 state tracking
  std::atomic<int> cia402_state_{static_cast<int>(Cia402State::UNKNOWN)};
  std::atomic<bool> is_enabled_{false};
  std::atomic<bool> is_fault_{false};
  std::atomic<bool> is_target_reached_{false};
  std::atomic<int8_t> current_mode_of_operation_{3};
  std::atomic<int8_t> active_mode_{3};

  // Position control
  std::atomic<bool> position_move_in_progress_{false};
  std::atomic<bool> manual_hold_active_{false};
  std::atomic<bool> ignore_current_position_command_after_hold_{false};
  double ignored_position_command_after_hold_{std::numeric_limits<double>::quiet_NaN()};
  double retrigger_last_error_{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point retrigger_last_check_time_{};
  std::atomic<double> profile_speed_mps_{0.010};
  double hw_hold_position_cmd_{0.0};  // 0=noop, 1=hold current position

  // Async read thread
  std::thread async_read_thread_;
  std::atomic<bool> async_read_running_{false};
  std::atomic<double> async_position_{0.0};
  std::atomic<double> async_velocity_{0.0};
  std::atomic<uint16_t> async_statusword_{0};
  std::atomic<uint32_t> async_digital_inputs_{0};

  // Detached thread framework
  std::shared_ptr<std::atomic<bool>> alive_flag_;
  std::shared_ptr<std::atomic<int>> active_detached_count_ =
    std::make_shared<std::atomic<int>>(0);
  std::shared_ptr<std::atomic<bool>> cmd_dispatch_in_progress_ =
    std::make_shared<std::atomic<bool>>(false);

  double homing_speed_mps_{0.010};
  double homing_low_speed_ratio_{0.2};
  int homing_method_{27};
  int homing_acceleration_{50000};
  double homing_timeout_sec_{60.0};
  bool homing_configure_di_{false};
  double lower_switch_position_m_{0.000};
  double home_switch_position_m_{0.650};
  double upper_switch_position_m_{0.950};
  double switch_position_tolerance_m_{0.008};
  bool di_active_low_{true};
  int di6_not_func_{0x02};
  int di4_homing_func_{0x16};
  int di5_pot_func_{0x01};
  int home_di_channel_{4};
  int pot_di_channel_{5};
  int not_di_channel_{6};
  bool homing_di6_config_applied_{false};
  bool homing_di4_config_applied_{false};
  bool homing_di5_config_applied_{false};
  std::string calibration_file_path_;
  std::string zero_source_{"none"};
  int32_t zero_raw_position_{0};
  bool zero_verified_by_drive_{false};

  std::thread homing_thread_;
  std::atomic<bool> homing_in_progress_{false};
  std::atomic<bool> homing_complete_{false};
  std::atomic<bool> homing_error_{false};
  std::atomic<bool> quick_stop_active_{false};
  std::atomic<bool> motion_ready_{false};
  std::atomic<bool> reference_valid_{false};
  std::atomic<bool> encoder_reference_lost_{false};
  std::atomic<bool> feedback_poll_paused_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<int> homing_state_atomic_{0};
  std::atomic<int> homing_detail_atomic_{0};
  std::mutex homing_mutex_;

  uint32_t last_digital_inputs_raw_{0};
  bool limit_switch_state_valid_{false};
  bool cia_not_bit0_{false};
  bool cia_pot_bit1_{false};
  bool cia_home_bit2_{false};
  bool si1_bit4_{false};
  bool si2_bit5_{false};
  bool si3_bit6_{false};
  bool si4_bit7_{false};
  bool si5_bit8_{false};
  bool si6_bit9_{false};
  bool di1_bit16_{false};
  bool di2_bit17_{false};
  bool di3_bit18_{false};
  bool di4_bit19_{false};
  bool di5_bit20_{false};
  bool di6_bit21_{false};
  bool di1_bit24_{false};
  bool di2_bit25_{false};
  bool di3_bit26_{false};
  bool di4_bit27_{false};
  bool di5_bit28_{false};
  bool di6_bit29_{false};
  int feedback_cycle_counter_{0};

  std::chrono::steady_clock::time_point last_feedback_time_;
  std::chrono::steady_clock::time_point last_read_time_;
  std::chrono::steady_clock::time_point last_command_update_time_;
};

}  // namespace lift_slide_driver

#endif  // LIFT_SLIDE_DRIVER__LIFT_SLIDE_HARDWARE_INTERFACE_HPP_
