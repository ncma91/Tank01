// Hardware configuration of the Tank01 base controller (Teensy 4.1).
// Pin map: docs/03_elektrik_energie.md, section 6.
#pragma once
#include <stdint.h>

namespace cfg {

// ---- Pins -------------------------------------------------------------------
constexpr uint8_t PIN_ENC_L_A = 2, PIN_ENC_L_B = 3;
constexpr uint8_t PIN_ENC_R_A = 4, PIN_ENC_R_B = 5;
constexpr uint8_t PIN_ESTOP_SENSE = 6;        // HIGH = safety chain closed (optocoupler)
constexpr uint8_t PIN_ESTOP_SW_TRIP = 7;      // HIGH = keep chain closed, LOW = open it
constexpr uint8_t PIN_PRECHARGE = 8;
constexpr uint8_t PIN_SAFETY_RESET = 9;
constexpr uint8_t PIN_BLADE_BRAKE = 10;       // HIGH = brake released
constexpr uint8_t PIN_HEIGHT_PWM = 11, PIN_HEIGHT_DIR = 12;
constexpr uint8_t PIN_HEIGHT_POS = 14;        // A0
constexpr uint8_t PIN_BAT_SENSE = 15;         // A1
constexpr uint8_t PIN_TILT_SWITCH = 16;       // LOW = tilted
constexpr uint8_t PIN_LIFT_SWITCH = 17;       // LOW = mower lifted
constexpr uint8_t PIN_BEACON = 24;
constexpr uint8_t PIN_BUZZER = 25;
constexpr uint8_t PIN_BRAKE_RELEASE = 26;     // HIGH = release spring brakes (via relay)
constexpr uint8_t PIN_LED = 13;

// ---- CAN --------------------------------------------------------------------
constexpr uint32_t CAN_BAUD = 500000;
constexpr uint8_t VESC_ID_LEFT = 10, VESC_ID_RIGHT = 11, VESC_ID_BLADE = 12;

// ---- Drive train ------------------------------------------------------------
constexpr float SPROCKET_RADIUS_M = 0.105f;
constexpr float GEAR_RATIO = 20.0f;           // motor : sprocket
constexpr float MOTOR_POLE_PAIRS = 5.0f;
constexpr float BLADE_POLE_PAIRS = 7.0f;
constexpr float BLADE_BELT_RATIO = 1.25f;     // motor : flail shaft
constexpr bool INVERT_LEFT = false;
constexpr bool INVERT_RIGHT = true;           // right motor is mounted mirrored

// ---- Limits (independent of the Jetson) ------------------------------------
constexpr float MAX_TRACK_SPEED = 2.4f;       // m/s
constexpr float MAX_ACCEL = 1.5f;             // m/s^2
constexpr float RAMP_DOWN_DECEL = 3.0f;       // m/s^2 during watchdog ramp-down
constexpr uint16_t MAX_BLADE_RPM = 2600;
constexpr float MIN_CUT_HEIGHT_M = 0.03f, MAX_CUT_HEIGHT_M = 0.12f;

// ---- Timing -----------------------------------------------------------------
constexpr uint32_t LOOP_US = 1000;            // 1 kHz main loop
constexpr uint32_t WATCHDOG_MS = 200;         // heartbeat / CMD_DRIVE timeout
constexpr uint32_t RAMP_DOWN_MS = 500;        // then FAULT + open chain
constexpr uint32_t VESC_TIMEOUT_MS = 100;
constexpr uint32_t PRECHARGE_MS = 1200;
constexpr uint32_t BLADE_IDLE_STOP_MS = 10000;  // blade off if standing still this long

// ---- Battery ------------------------------------------------------------------
constexpr float BAT_DIVIDER = 21.0f;          // 100k / 5k
constexpr float BAT_UNDERVOLTAGE = 44.0f;     // 16S LiFePO4 → 2.75 V/cell
constexpr uint32_t BMS_CAN_ID = 0x18FF28F4;   // depends on BMS model – adapt!

}  // namespace cfg
