#pragma once
#include <cstdint>

namespace gripper {

// Must stay in sync with gripper_ctrl.h on the STM32
enum class State : uint8_t {
  IDLE              = 0,
  OPENING           = 1,
  APPROACHING       = 2,
  GRASPING          = 3,
  ADMITTANCE_CTRL   = 4,
  FAULT             = 5,
  CALIBRATING_FORCE = 6,
  MANUAL_TUNE       = 7,
  ZEROING_MOTOR     = 8,
  SAVING_FLASH      = 9,
  ABORTED           = 10
};

enum class Fault : uint8_t {
  NONE                    = 0,
  MECH_LIMIT_OPEN         = 1,
  MECH_LIMIT_CLOSE        = 2,
  MOTOR_MODE_ERROR        = 3,
  CAN_TX_JAMMED           = 4,
  MOTOR_RX_TIMEOUT        = 5,
  REALTIME_DEADLINE_MISSED= 6,
  MOTOR_HARDWARE_ERROR    = 7
};

// Maps param_id (0-22) to config field name — mirrors CONFIG_PARAMS in Python
inline const char* param_name(uint8_t id) {
  static const char* names[] = {
    "kp","ki","kd","max_integral","max_torque",
    "open_pos","close_pos","approach_limit",
    "open_kp","open_kd","approach_kd","approach_vel",
    "deadband","lpf_alpha","contact_threshold",
    "pc3_zero_adc","pc2_zero_adc",
    "target_force_grams","telemetry_ms","virtual_mass",
    "virtual_damping","admit_max_vel","admit_kd"
  };
  return (id < 23) ? names[id] : "unknown";
}

constexpr uint32_t CAN_ID_CMD        = 0x200;
constexpr uint32_t CAN_ID_STATUS     = 0x100;
constexpr uint32_t CAN_ID_MOTOR      = 0x101;
constexpr uint32_t CAN_ID_CONFIG_RX  = 0x102;

constexpr uint8_t CMD_CLEAR_FAULT    = 0x01;
constexpr uint8_t CMD_SET_STATE      = 0x02;
constexpr uint8_t CMD_SET_FORCE      = 0x03;
constexpr uint8_t CMD_SET_PARAM      = 0x04;
constexpr uint8_t CMD_MANUAL_POS     = 0x05;
constexpr uint8_t CMD_GET_PARAM      = 0x06;

} // namespace gripper