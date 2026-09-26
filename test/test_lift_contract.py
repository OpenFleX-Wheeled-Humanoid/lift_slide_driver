from pathlib import Path
import os
import re

import pytest
import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
LIFT_DRIVER = WORKSPACE / "src/openflex_lift_slide/lift_slide_driver"
LIFT_BRINGUP = WORKSPACE / "src/openflex_lift_slide/lift_slide_bringup"
INTEGRATED = WORKSPACE / "src/openflex_integrated/openarmx_integrated_bringup"


def _openflex_can_roots():
    configured = os.environ.get("OPENFLEX_CAN_SOURCE")
    if configured:
        yield Path(configured).expanduser()

    for base in (WORKSPACE, *WORKSPACE.parents):
        yield base / "src/openflex_can"
        yield base / "driver/sources/openflex_can"
        yield base / "sources/openflex_can"
    yield Path.home() / "driver/sources/openflex_can"


def _find_can_include_root():
    for source_root in _openflex_can_roots():
        include_root = source_root / "include/openflex_can"
        if (include_root / "lift/lift_canopen_master.hpp").is_file():
            return include_root

    prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    ros_distro = os.environ.get("ROS_DISTRO")
    if ros_distro:
        prefixes.append(f"/opt/ros/{ros_distro}")
    ros_root = Path("/opt/ros")
    if ros_root.is_dir():
        prefixes.extend(str(path) for path in sorted(ros_root.iterdir()))
    for prefix in prefixes:
        include_root = Path(prefix) / "include/openflex_can"
        if (include_root / "lift/lift_canopen_master.hpp").is_file():
            return include_root
    raise FileNotFoundError(
        "openflex_can headers were not found; source the ROS environment or set OPENFLEX_CAN_SOURCE"
    )


def _find_ld2_source():
    for source_root in _openflex_can_roots():
        source = source_root / "src/lift/ld2_canopen_drive.cpp"
        if source.is_file():
            return source
    return None


CAN_INCLUDE_ROOT = _find_can_include_root()
CAN_HEADER = CAN_INCLUDE_ROOT / "lift/lift_canopen_master.hpp"
LD2_HEADER = CAN_INCLUDE_ROOT / "lift/ld2_canopen_drive.hpp"
LD2_SOURCE = _find_ld2_source()


def _parameters(path: Path):
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return document["lift_slide_defaults"]["ros__parameters"]


def test_lift_defaults_are_consistent_between_bringups():
    standalone = _parameters(LIFT_BRINGUP / "config/ros2_controllers.yaml")
    integrated = yaml.safe_load(
        (INTEGRATED / "config/integrated_controllers.yaml").read_text(encoding="utf-8")
    )
    manual = integrated["lift_manual_position_controller"]["ros__parameters"]
    launch = (INTEGRATED / "launch/integrated_robot_bringup.launch.py").read_text(encoding="utf-8")
    assert standalone["node_id"] == 16
    assert re.search(r"'node_id':\s*16", launch)
    assert standalone["min_position_m"] == manual["min_position_m"]
    assert standalone["max_position_m"] == manual["max_position_m"]
    assert standalone["min_position_m"] == -0.750
    assert standalone["max_position_m"] == 0.400
    assert standalone["lower_switch_position_m"] == 0.000
    assert standalone["home_switch_position_m"] == 0.650
    assert standalone["upper_switch_position_m"] == 0.950
    assert manual["min_position_m"] == -0.750
    assert manual["max_position_m"] == 0.400
    assert manual["lower_switch_position_m"] == 0.000
    assert manual["home_switch_position_m"] == 0.650
    assert manual["upper_switch_position_m"] == 0.950


def test_calibration_file_is_package_owned_and_explicitly_passed():
    calibration = LIFT_DRIVER / "config/lift_slide_calibration.yaml"
    assert calibration.exists()
    hardware_cpp = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    assert 'get_param("calibration_file")' in hardware_cpp
    assert 'std::getenv("HOME")' not in hardware_cpp


def test_lower_can_exposes_detailed_sdo_result():
    header = CAN_HEADER.read_text(encoding="utf-8")
    assert "struct SdoResult" in header
    assert "sdo_write_ex" in header
    assert "sdo_read_ex" in header


def test_ld2_protocol_is_owned_by_low_level_driver():
    assert LD2_HEADER.exists()
    header = LD2_HEADER.read_text(encoding="utf-8")
    assert "class Ld2CanopenDrive" in header
    assert "read_actual_position" in header
    assert "write_controlword" in header
    assert "clear_position" in header
    hardware_cpp = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    assert not re.search(r"constexpr\\s+uint16_t\\s+OBJ_[A-Z_]+\\s*=\\s*0x", hardware_cpp)


def test_rviz_and_panel_do_not_default_to_inactive_position_controller():
    launch = (INTEGRATED / "launch/integrated_robot_bringup.launch.py").read_text(encoding="utf-8")
    standalone_launch = (LIFT_BRINGUP / "launch/lift_slide_bringup.launch.py").read_text(encoding="utf-8")
    panel = (WORKSPACE / "src/openflex_lift_slide/lift_slide_panel/src/lift_panel.cpp").read_text(encoding="utf-8")
    vr_lift = (
        WORKSPACE
        / "src/openflex_chassis/system_bringup_layer/swerve_bringup/scripts/vr_lift_control_node.py"
    ).read_text(encoding="utf-8")
    assert "position_command_topic:=/lift_position_controller/commands" not in launch
    assert "position_command_topic:=/lift_position_controller/commands" not in standalone_launch
    assert '"position_command_topic", "/lift_position_controller/commands"' not in panel
    assert 'declare_parameter("position_topic"' not in vr_lift


def test_zeroing_and_return_home_are_distinct_paths():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    start = source.index("bool LiftSlideHardwareInterface::start_homing_sequence()")
    return_start = source.index("bool LiftSlideHardwareInterface::return_to_home_sequence()")
    zeroing = source[start:return_start]
    returning = source[return_start:]
    assert "set_drive_zero_point()" in zeroing
    assert "write_absolute_encoder_command" in source
    assert "MOVING_UP_TO_HOME" in zeroing
    assert "HOME detected while moving up" in zeroing
    assert zeroing.index("HOME detected while moving up") < zeroing.index(
        "Upper hard limit established"
    )
    assert "upper_endpoint_position + lower_endpoint_position" in zeroing
    assert "set_drive_zero_point()" not in returning


def test_fake_zeroing_uses_the_same_home_first_search_policy_as_real_hardware():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    fake_start = source.index("bool LiftSlideHardwareInterface::simulate_fake_homing_sequence()")
    fake_zeroing = source[fake_start:]
    assert "at_or_above_home" not in fake_zeroing
    assert "MOVING_UP_TO_HOME" in fake_zeroing


def test_zeroing_synchronizes_async_position_cache_from_verified_drive_feedback():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    zeroing_start = source.index("if (!set_drive_zero_point())")
    zeroing_end = source.index("feedback_poll_paused_.store(false);", zeroing_start)
    zeroing_block = source[zeroing_start:zeroing_end]
    assert "async_position_.store(position_counts_to_m(final_position_counts))" in zeroing_block
    assert "hw_position_ = position_counts_to_m(final_position_counts)" in zeroing_block


def test_zeroing_reenables_drive_with_full_cia402_sequence():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    zeroing_start = source.index("if (!set_drive_zero_point())")
    zeroing_end = source.index("feedback_poll_paused_.store(false);", zeroing_start)
    zeroing_block = source[zeroing_start:zeroing_end]
    assert "configure_drive()" in zeroing_block
    assert "enable_drive()" in zeroing_block


def test_zeroing_confirms_drive_and_position_transitions_instead_of_fixed_delays():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    start = source.index("bool LiftSlideHardwareInterface::set_drive_zero_point()")
    end = source.index("bool LiftSlideHardwareInterface::configure_homing_di_mapping()", start)
    zeroing = source[start:end]

    disabled_confirm = zeroing.index("SWITCH_ON_DISABLED confirmed")
    write_absolute_encoder = zeroing.index("write_absolute_encoder_command")
    zero_confirm = zeroing.index("6064h reached zero")
    assert zeroing.index("disable_drive()") < disabled_confirm < write_absolute_encoder
    assert write_absolute_encoder < zero_confirm
    assert "for (int poll = 0; poll < 60; ++poll)" in zeroing
    assert "std::this_thread::sleep_for(std::chrono::milliseconds(50))" in zeroing


def test_ld2_clear_position_uses_the_four_byte_object_width():
    if LD2_SOURCE is None:
        pytest.skip("openflex_can source is unavailable; set OPENFLEX_CAN_SOURCE to enable this check")
    source = LD2_SOURCE.read_text(encoding="utf-8")
    clear_start = source.index("bool Ld2CanopenDrive::clear_position")
    clear_end = source.index("bool Ld2CanopenDrive::send_velocity_command", clear_start)
    clear_position = source[clear_start:clear_end]
    assert "Object::CLEAR_POSITION, 0, 1, 4" in clear_position


def test_zeroing_validates_absolute_encoder_parameter_after_command():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    start = source.index("bool LiftSlideHardwareInterface::set_drive_zero_point()")
    end = source.index("bool LiftSlideHardwareInterface::configure_homing_di_mapping()", start)
    zeroing = source[start:end]
    assert "read_absolute_encoder_parameter" in zeroing
    assert "absolute-encoder parameter" in zeroing
    assert "2015h remained 9" in zeroing


def test_zeroing_persists_the_drive_zero_as_raw_zero():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    start = source.index("bool LiftSlideHardwareInterface::set_drive_zero_point()")
    end = source.index("bool LiftSlideHardwareInterface::configure_homing_di_mapping()", start)
    zeroing = source[start:end]
    assert "zero_raw_position_ = 0;" in zeroing
    assert "zero_raw_position_ = position_before_zero" not in zeroing


def test_zeroing_reports_calibration_write_failure():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    assert "calibration_file_path_.empty() && !save_calibration()" in source


def test_normal_position_dispatch_logs_are_debug_level():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    assert 'RCLCPP_DEBUG(\n      logger_,\n      "[manual] position dispatch start:' in source
    assert 'RCLCPP_DEBUG(\n        logger_,\n        "[manual] position dispatch done:' in source
    assert 'RCLCPP_DEBUG(logger_, "Position move:' in source


def test_reenable_zero_check_reports_actual_position_and_uses_realistic_tolerance():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    assert "zero_reenable_tolerance_counts" in source
    assert "final 6064h verification after re-enable failed: value=" in source


def test_zeroing_uses_homing_mode_for_absolute_encoder_zero_write():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    start = source.index("bool LiftSlideHardwareInterface::set_drive_zero_point()")
    end = source.index("bool LiftSlideHardwareInterface::configure_homing_di_mapping()", start)
    zeroing = source[start:end]
    disable = zeroing.index("disable_drive()")
    homing_mode = zeroing.index("MODE_HOMING")
    encoder_write = zeroing.index("write_absolute_encoder_command")
    assert disable < homing_mode < encoder_write
    assert "OBJ_MODE_OF_OPERATION_DISPLAY" in zeroing
    assert "MODE_VELOCITY" in zeroing[encoder_write:]


def test_zeroing_logs_ld2_error_code_when_absolute_encoder_write_is_rejected():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    start = source.index("bool LiftSlideHardwareInterface::set_drive_zero_point()")
    end = source.index("bool LiftSlideHardwareInterface::configure_homing_di_mapping()", start)
    zeroing = source[start:end]
    failure = zeroing.index("LD2 zero-point write 2015h=9 failed")
    assert "OBJ_ERROR_CODE" in zeroing
    assert "603Fh" in zeroing[failure:failure + 800]


def test_zeroing_waits_for_actual_velocity_to_settle_before_disabling_drive():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    start = source.index("bool LiftSlideHardwareInterface::set_drive_zero_point()")
    end = source.index("bool LiftSlideHardwareInterface::configure_homing_di_mapping()", start)
    zeroing = source[start:end]
    velocity_wait = zeroing.index("OBJ_VELOCITY_ACTUAL")
    disable = zeroing.index("disable_drive()")
    assert velocity_wait < disable
    assert "velocity_zero_streak" in zeroing
    assert "actual velocity" in zeroing


def test_zeroing_rechecks_6064_after_reenabling_the_drive():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    homing_start = source.index("bool LiftSlideHardwareInterface::start_homing_sequence()")
    homing_end = source.index("bool LiftSlideHardwareInterface::return_to_home_sequence()")
    start = source.rindex("if (!enable_drive())", homing_start, homing_end)
    end = source.index("feedback_poll_paused_.store(false);", start)
    post_enable = source[start:end]
    assert "final 6064h verified after re-enable" in post_enable
    assert "async_position_.store(position_counts_to_m(final_position_counts))" in post_enable


def test_manual_controller_releases_jog_inhibit_after_homing():
    source = (LIFT_DRIVER / "src/lift_slide_manual_position_controller.cpp").read_text(encoding="utf-8")
    assert "homing_complete\") > 0.5" in source
    assert "inhibit_jog_until_stop_ = false" in source


def test_manual_motion_does_not_require_zero_calibration():
    source = (LIFT_DRIVER / "src/lift_slide_manual_position_controller.cpp").read_text(encoding="utf-8")
    can_move_start = source.index("bool LiftSlideManualPositionController::can_move")
    can_move_end = source.index("void LiftSlideManualPositionController::publish_reference", can_move_start)
    can_move = source[can_move_start:can_move_end]
    assert "zero point is not verified" not in can_move


def test_panel_does_not_gate_manual_buttons_on_zero_calibration():
    panel = (
        WORKSPACE / "src/openflex_lift_slide/lift_slide_panel/src/lift_panel.cpp"
    ).read_text(encoding="utf-8")
    can_operate_start = panel.index("bool LiftPanel::canOperate")
    can_operate_end = panel.index("void LiftPanel::updateButtons", can_operate_start)
    assert "driver_homed_" not in panel[can_operate_start:can_operate_end]


def test_manual_controller_keeps_vla_absolute_position_topic_compatible():
    source = (LIFT_DRIVER / "src/lift_slide_manual_position_controller.cpp").read_text(encoding="utf-8")
    assert "/lift_manual_position_controller/position_command" in source


def test_vla_absolute_position_does_not_require_zero_calibration():
    source = (LIFT_DRIVER / "src/lift_slide_manual_position_controller.cpp").read_text(encoding="utf-8")
    position_start = source.index("void LiftSlideManualPositionController::handle_position")
    position_end = source.index("void LiftSlideManualPositionController::handle_jog_start", position_start)
    position_handler = source[position_start:position_end]
    assert "zero point is not verified" not in position_handler


def test_encoder_reference_recovery_is_explicit_and_keeps_motion_separate_from_reference():
    source = (LIFT_DRIVER / "src/lift_slide_hardware_interface.cpp").read_text(encoding="utf-8")
    header = (LIFT_DRIVER / "include/lift_slide_driver/lift_slide_hardware_interface.hpp").read_text(encoding="utf-8")
    msg = (WORKSPACE / "src/openflex_lift_slide/lift_slide_msgs/msg/MotorStatus.msg").read_text(encoding="utf-8")
    assert "0x7325" in source
    assert "recover_encoder_reference_loss" in source
    assert "motion_ready_" in header
    assert "reference_valid_" in header
    assert "bool motion_ready" in msg
    assert "bool reference_valid" in msg
    assert "bool encoder_reference_lost" in msg


def test_recovery_yaml_has_provisional_reference_fields():
    calibration = LIFT_DRIVER / "config/lift_slide_calibration.yaml"
    text = calibration.read_text(encoding="utf-8")
    assert "motion_ready:" in text
    assert "reference_valid:" in text
    assert "encoder_reference_lost:" in text
