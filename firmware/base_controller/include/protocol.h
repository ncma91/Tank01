// CAN protocol Jetson <-> Teensy base controller.
// MUST stay in sync with ros2_ws/src/tank01_base/tank01_base/can_protocol.py and
// docs/A_can_protokoll.md (checked by test_can_protocol.py).
#pragma once
#include <stdint.h>

namespace proto {

// Jetson -> Teensy
constexpr uint32_t ID_CMD_DRIVE       = 0x100;
constexpr uint32_t ID_CMD_BLADE       = 0x101;
constexpr uint32_t ID_HEARTBEAT       = 0x102;
constexpr uint32_t ID_TIME_SYNC       = 0x103;
constexpr uint32_t ID_CMD_FAULT_RESET = 0x104;

// Teensy -> Jetson
constexpr uint32_t ID_STATUS       = 0x200;
constexpr uint32_t ID_ENCODERS     = 0x201;
constexpr uint32_t ID_MOTOR_LEFT   = 0x202;
constexpr uint32_t ID_MOTOR_RIGHT  = 0x203;
constexpr uint32_t ID_MOTOR_BLADE  = 0x204;
constexpr uint32_t ID_BLADE_STATUS = 0x205;
constexpr uint32_t ID_ULTRASONIC   = 0x206;
constexpr uint32_t ID_BMS          = 0x207;

constexpr uint8_t FAULT_RESET_MAGIC = 0xA5;

enum class DriveMode : uint8_t { STANDBY = 0, MANUAL = 1, AUTO = 2 };

enum class State : uint8_t { BOOT = 0, STANDBY = 1, ACTIVE = 2, RAMP_DOWN = 3, FAULT = 4, ESTOP = 5 };

enum SafetyFlag : uint8_t {
  CHAIN_OPEN   = 1 << 0,
  TILT_SWITCH  = 1 << 1,
  MOWER_LIFTED = 1 << 2,
  WATCHDOG     = 1 << 3,
  VESC_TIMEOUT = 1 << 4,
  UNDERVOLTAGE = 1 << 5,
  OVERTEMP     = 1 << 6,
};

enum DriveFlag : uint8_t { RELEASE_BRAKE = 1 << 0, BEACON = 1 << 1, HORN = 1 << 2 };

enum FaultCode : uint8_t {
  FAULT_NONE = 0, FAULT_WATCHDOG = 1, FAULT_VESC_TIMEOUT = 2, FAULT_ENCODER = 3,
  FAULT_UNDERVOLTAGE = 4, FAULT_OVERTEMP = 5, FAULT_PRECHARGE = 6,
};

#pragma pack(push, 1)
struct CmdDrive    { int16_t left_mm_s; int16_t right_mm_s; uint8_t mode; uint8_t flags; uint16_t seq; };
struct CmdBlade    { uint16_t rpm; uint16_t height_mm; uint8_t flags; uint8_t pad[3]; };
struct Heartbeat   { uint32_t counter; uint8_t safety_state; uint8_t pad[3]; };
struct TimeSync    { uint64_t unix_us; };
struct Status      { uint8_t state; uint8_t safety_flags; uint8_t vesc_ok_mask; uint8_t fault_code;
                     uint16_t battery_cv; uint8_t soc; int8_t temp_c; };
struct Encoders    { int32_t left_ticks; int32_t right_ticks; };
struct MotorStatus { int16_t rpm; int16_t current_da; int16_t temp_dc; int16_t duty_pm; };
struct BladeStatus { uint16_t rpm; uint16_t height_mm; uint8_t flags; uint8_t pad[3]; };
struct Ultrasonic  { uint8_t dist_2cm[6]; uint8_t pad[2]; };
struct Bms         { uint16_t voltage_cv; int16_t current_da; uint8_t soc; int8_t temp_c; uint16_t cell_min_mv; };
#pragma pack(pop)

static_assert(sizeof(CmdDrive) == 8, "frame size");
static_assert(sizeof(CmdBlade) == 8, "frame size");
static_assert(sizeof(Heartbeat) == 8, "frame size");
static_assert(sizeof(TimeSync) == 8, "frame size");
static_assert(sizeof(Status) == 8, "frame size");
static_assert(sizeof(Encoders) == 8, "frame size");
static_assert(sizeof(MotorStatus) == 8, "frame size");
static_assert(sizeof(BladeStatus) == 8, "frame size");
static_assert(sizeof(Ultrasonic) == 8, "frame size");
static_assert(sizeof(Bms) == 8, "frame size");

}  // namespace proto
