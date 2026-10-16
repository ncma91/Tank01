// Tank01 base controller firmware (Teensy 4.1)
//
// Responsibilities (docs/01_systemarchitektur.md, level 1):
//   * CAN bridge to the Jetson (CAN1) and to the VESC motor controllers (CAN2)
//   * Track encoders, safety inputs (e-stop chain, tilt switch, mower lift switch)
//   * Watchdog: no heartbeat/drive command for 200 ms -> ramp down -> FAULT, chain open
//   * Hard limits for speed, acceleration and blade RPM independent of the Jetson
//   * Pre-charge sequence, mower height actuator, beacon/buzzer
//
// The hardware e-stop chain works without this firmware. The firmware can OPEN the
// chain (PIN_ESTOP_SW_TRIP) but can never bridge it.

#include <Arduino.h>
#include <Encoder.h>
#include <FlexCAN_T4.h>
#include <Servo.h>
#include <string.h>

#include "config.h"
#include "protocol.h"

using namespace proto;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_32> canJetson;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_32> canVesc;

Encoder encLeft(cfg::PIN_ENC_L_A, cfg::PIN_ENC_L_B);
Encoder encRight(cfg::PIN_ENC_R_A, cfg::PIN_ENC_R_B);

// ----------------------------------------------------------------------------- state
struct VescData {
  float rpm_mech = 0;      // output shaft (sprocket / flail shaft)
  float current = 0;       // A
  float temp_motor = 0;    // degC
  float duty = 0;          // 0..1
  uint32_t last_ms = 0;
};

static State state = State::BOOT;
static uint8_t faultCode = FAULT_NONE;
static uint8_t safetyFlags = 0;

static uint32_t lastHeartbeatMs = 0, lastDriveMs = 0, rampStartMs = 0, bootMs = 0;
static uint32_t lastHbCounter = 0;
static bool hbSeen = false;
static uint8_t jetsonSafetyState = 3;  // STOP until told otherwise

static float targetLeft = 0, targetRight = 0;   // m/s from CMD_DRIVE
static float outLeft = 0, outRight = 0;         // ramped setpoints
static uint8_t driveFlags = 0;

static uint16_t bladeRpmCmd = 0;
static float cutHeightCmd = 0.07f;
static bool bladeEnableCmd = false;
static uint32_t standstillSinceMs = 0;

static VescData vLeft, vRight, vBlade;
static Bms bms{};
static uint32_t bmsLastMs = 0;
static uint8_t ultrasonic[6] = {0};

// ------------------------------------------------------------------------- helpers
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static float rampTo(float target, float current, float rate, float dt) {
  const float step = rate * dt;
  return current + clampf(target - current, -step, step);
}

constexpr bool PWM_MODE = cfg::DRIVE_INTERFACE == cfg::DriveInterface::RC_PWM;

// ------------------------------------------------------------- RC receiver input
// Pulse widths are measured with pin-change interrupts (Teensy 4.1: every pin can).
struct RcChannel {
  volatile uint32_t riseUs = 0;
  volatile uint16_t widthUs = 0;
  volatile uint32_t lastMs = 0;
};
static RcChannel rcCh1, rcCh2, rcMode;

static void rcEdge(RcChannel& ch, uint8_t pin) {
  const uint32_t now = micros();
  if (digitalReadFast(pin)) {
    ch.riseUs = now;
  } else {
    const uint32_t w = now - ch.riseUs;
    if (w >= 800 && w <= 2200) {
      ch.widthUs = uint16_t(w);
      ch.lastMs = millis();
    }
  }
}
static void rcIsr1() { rcEdge(rcCh1, cfg::PIN_RC_IN_CH1); }
static void rcIsr2() { rcEdge(rcCh2, cfg::PIN_RC_IN_CH2); }
static void rcIsrMode() { rcEdge(rcMode, cfg::PIN_RC_IN_MODE); }

static bool rcValid(const RcChannel& ch, uint32_t now) {
  return now - ch.lastMs < cfg::RC_TIMEOUT_MS;
}

static bool rcLinkOk(uint32_t now) {
  return !cfg::RC_INPUT_ENABLED || (rcValid(rcCh1, now) && rcValid(rcCh2, now));
}

// MANUAL = RC link ok and mode switch not in AUTO position
static bool rcManual(uint32_t now) {
  return cfg::RC_INPUT_ENABLED && rcLinkOk(now) &&
         !(rcValid(rcMode, now) && rcMode.widthUs > cfg::RC_AUTO_US);
}

static float rcNorm(uint16_t us) {
  const int d = int(us) - int(cfg::PWM_NEUTRAL_US);
  if (abs(d) < int(cfg::PWM_DEADBAND_US)) return 0.0f;
  return clampf(float(d) / cfg::PWM_SPAN_US, -1.0f, 1.0f);
}

// Operator stick positions -> track speed targets (m/s)
static void rcToTracks(float& left, float& right) {
  const float a = rcNorm(rcCh1.widthUs) * (cfg::PWM_INVERT_CH1 ? -1.0f : 1.0f);
  const float b = rcNorm(rcCh2.widthUs) * (cfg::PWM_INVERT_CH2 ? -1.0f : 1.0f);
  if (cfg::PWM_MIXING == cfg::PwmMixing::ARCADE) {   // a = throttle, b = steering (right +)
    left = clampf(a + b, -1, 1);
    right = clampf(a - b, -1, 1);
  } else {
    left = a;
    right = b;
  }
  left *= cfg::RC_MANUAL_MAX_SPEED;
  right *= cfg::RC_MANUAL_MAX_SPEED;
}

// --------------------------------------------------------------- PWM output
Servo pwmCh1, pwmCh2, pwmBlade;

static uint16_t toPulse(float norm, bool invert) {
  if (invert) norm = -norm;
  return uint16_t(cfg::PWM_NEUTRAL_US + clampf(norm, -1, 1) * cfg::PWM_SPAN_US);
}

static void pwmSetTracks(float left, float right) {
  const float l = left / cfg::PWM_FULL_SCALE_SPEED;
  const float r = right / cfg::PWM_FULL_SCALE_SPEED;
  float a, b;
  if (cfg::PWM_MIXING == cfg::PwmMixing::ARCADE) {
    a = 0.5f * (l + r);   // throttle
    b = 0.5f * (l - r);   // steering, positive = turn right (left track faster)
  } else {
    a = l;
    b = r;
  }
  pwmCh1.writeMicroseconds(toPulse(a, cfg::PWM_INVERT_CH1));
  pwmCh2.writeMicroseconds(toPulse(b, cfg::PWM_INVERT_CH2));
}

static void pwmSetBlade(uint16_t rpm) {
  // RC ESC convention: 1000 us = stop, 2000 us = full speed
  const float k = clampf(float(rpm) / cfg::MAX_BLADE_RPM, 0, 1);
  pwmBlade.writeMicroseconds(uint16_t(1000 + 1000 * k));
}

template <typename T>
static void sendJetson(uint32_t id, const T& payload) {
  CAN_message_t m;
  m.id = id;
  m.len = 8;
  memset(m.buf, 0, 8);
  memcpy(m.buf, &payload, sizeof(T));
  canJetson.write(m);
}

// --- VESC protocol (extended IDs, big-endian payload) ------------------------------
enum VescPacket : uint8_t { CAN_PACKET_SET_CURRENT_BRAKE = 2, CAN_PACKET_SET_RPM = 3,
                            CAN_PACKET_STATUS = 9, CAN_PACKET_STATUS_4 = 16 };

static void vescSendInt32(uint8_t vescId, uint8_t cmd, int32_t value) {
  CAN_message_t m;
  m.flags.extended = 1;
  m.id = (uint32_t(cmd) << 8) | vescId;
  m.len = 4;
  m.buf[0] = value >> 24; m.buf[1] = value >> 16; m.buf[2] = value >> 8; m.buf[3] = value;
  canVesc.write(m);
}

static void vescSetTrackSpeed(uint8_t vescId, float mps, bool invert) {
  const float sprocketRpm = mps / (2.0f * PI * cfg::SPROCKET_RADIUS_M) * 60.0f;
  float erpm = sprocketRpm * cfg::GEAR_RATIO * cfg::MOTOR_POLE_PAIRS;
  if (invert) erpm = -erpm;
  if (fabsf(mps) < 0.005f) {
    vescSendInt32(vescId, CAN_PACKET_SET_CURRENT_BRAKE, 20000);  // 20 A brake current (mA)
  } else {
    vescSendInt32(vescId, CAN_PACKET_SET_RPM, int32_t(erpm));
  }
}

static void vescSetBlade(uint16_t rpm) {
  if (rpm == 0) {
    vescSendInt32(cfg::VESC_ID_BLADE, CAN_PACKET_SET_CURRENT_BRAKE, 30000);
  } else {
    const float erpm = rpm * cfg::BLADE_BELT_RATIO * cfg::BLADE_POLE_PAIRS;
    vescSendInt32(cfg::VESC_ID_BLADE, CAN_PACKET_SET_RPM, int32_t(erpm));
  }
}

static int32_t be32(const uint8_t* b) { return int32_t(b[0]) << 24 | int32_t(b[1]) << 16 | int32_t(b[2]) << 8 | b[3]; }
static int16_t be16(const uint8_t* b) { return int16_t(uint16_t(b[0]) << 8 | b[1]); }

static void onVescFrame(const CAN_message_t& m) {
  if (m.flags.extended && m.id == cfg::BMS_CAN_ID) {
    // Example layout - adapt to the BMS in use (docs/03, section 3).
    bms.voltage_cv = uint16_t(m.buf[0] << 8 | m.buf[1]) * 10;
    bms.current_da = be16(&m.buf[2]);
    bms.soc = m.buf[4];
    bms.temp_c = int8_t(m.buf[5] - 40);
    bms.cell_min_mv = uint16_t(m.buf[6] << 8 | m.buf[7]);
    bmsLastMs = millis();
    return;
  }
  if (!m.flags.extended) return;
  const uint8_t id = m.id & 0xFF;
  const uint8_t cmd = (m.id >> 8) & 0xFF;
  VescData* v = id == cfg::VESC_ID_LEFT ? &vLeft : id == cfg::VESC_ID_RIGHT ? &vRight
              : id == cfg::VESC_ID_BLADE ? &vBlade : nullptr;
  if (!v) return;
  if (cmd == CAN_PACKET_STATUS) {
    const float erpm = be32(&m.buf[0]);
    const bool blade = (v == &vBlade);
    const float div = blade ? cfg::BLADE_POLE_PAIRS * cfg::BLADE_BELT_RATIO
                            : cfg::MOTOR_POLE_PAIRS * cfg::GEAR_RATIO;
    v->rpm_mech = erpm / div;
    v->current = be16(&m.buf[4]) / 10.0f;
    v->duty = be16(&m.buf[6]) / 1000.0f;
    v->last_ms = millis();
  } else if (cmd == CAN_PACKET_STATUS_4) {
    v->temp_motor = be16(&m.buf[2]) / 10.0f;
  }
}

// --------------------------------------------------------------------- Jetson input
static void onJetsonFrame(const CAN_message_t& m) {
  const uint32_t now = millis();
  switch (m.id) {
    case ID_CMD_DRIVE: {
      CmdDrive c; memcpy(&c, m.buf, sizeof(c));
      targetLeft = clampf(c.left_mm_s / 1000.0f, -cfg::MAX_TRACK_SPEED, cfg::MAX_TRACK_SPEED);
      targetRight = clampf(c.right_mm_s / 1000.0f, -cfg::MAX_TRACK_SPEED, cfg::MAX_TRACK_SPEED);
      if (c.mode == uint8_t(DriveMode::STANDBY)) targetLeft = targetRight = 0;
      driveFlags = c.flags;
      lastDriveMs = now;
      break;
    }
    case ID_CMD_BLADE: {
      CmdBlade c; memcpy(&c, m.buf, sizeof(c));
      bladeRpmCmd = c.rpm > cfg::MAX_BLADE_RPM ? cfg::MAX_BLADE_RPM : c.rpm;
      cutHeightCmd = clampf(c.height_mm / 1000.0f, cfg::MIN_CUT_HEIGHT_M, cfg::MAX_CUT_HEIGHT_M);
      bladeEnableCmd = c.flags & 1;
      break;
    }
    case ID_HEARTBEAT: {
      Heartbeat h; memcpy(&h, m.buf, sizeof(h));
      // Counter must strictly increase (a hung process repeating old frames is not alive).
      if (!hbSeen || int32_t(h.counter - lastHbCounter) > 0) {
        lastHeartbeatMs = now;
        lastHbCounter = h.counter;
        hbSeen = true;
      }
      jetsonSafetyState = h.safety_state;
      break;
    }
    case ID_TIME_SYNC:
      break;  // TODO: offset estimation for sensor timestamps (docs/04, section 4)
    case ID_CMD_FAULT_RESET:
      if (m.buf[0] == FAULT_RESET_MAGIC && state == State::FAULT) {
        faultCode = FAULT_NONE;
        state = State::STANDBY;
        digitalWrite(cfg::PIN_ESTOP_SW_TRIP, HIGH);
        // Operator must still acknowledge on the safety relay (no automatic restart).
      }
      break;
  }
}

// --------------------------------------------------------------------- state machine
static void enterFault(uint8_t code) {
  faultCode = code;
  state = State::FAULT;
  digitalWrite(cfg::PIN_ESTOP_SW_TRIP, LOW);   // open the safety chain
}

static void readSafetyInputs() {
  safetyFlags = 0;
  if (digitalRead(cfg::PIN_ESTOP_SENSE) == LOW) safetyFlags |= CHAIN_OPEN;
  if (digitalRead(cfg::PIN_TILT_SWITCH) == LOW) safetyFlags |= TILT_SWITCH;
  if (digitalRead(cfg::PIN_LIFT_SWITCH) == LOW) safetyFlags |= MOWER_LIFTED;
  const float vbat = analogRead(cfg::PIN_BAT_SENSE) * (3.3f / 1023.0f) * cfg::BAT_DIVIDER;
  if (millis() - bootMs > 2000 && vbat < cfg::BAT_UNDERVOLTAGE) safetyFlags |= UNDERVOLTAGE;
  if (vLeft.temp_motor > 110 || vRight.temp_motor > 110 || vBlade.temp_motor > 110)
    safetyFlags |= OVERTEMP;
  const uint32_t now = millis();
  if (!rcLinkOk(now)) safetyFlags |= RC_LINK_LOST;
  if (!PWM_MODE && state == State::ACTIVE &&
      (now - vLeft.last_ms > cfg::VESC_TIMEOUT_MS || now - vRight.last_ms > cfg::VESC_TIMEOUT_MS))
    safetyFlags |= VESC_TIMEOUT;
}

static void updateState(uint32_t now) {
  const bool jetsonOk = hbSeen && (now - lastHeartbeatMs < cfg::WATCHDOG_MS) &&
                        (now - lastDriveMs < cfg::WATCHDOG_MS);
  // MANUAL (RC pass-through) does not need the Jetson; AUTO needs Jetson AND RC link.
  const bool commsOk = rcManual(now) || (jetsonOk && rcLinkOk(now));

  if (safetyFlags & CHAIN_OPEN) {
    if (state != State::FAULT) state = State::ESTOP;
    return;
  }
  switch (state) {
    case State::BOOT:
      digitalWrite(cfg::PIN_PRECHARGE, HIGH);
      if (now - bootMs > cfg::PRECHARGE_MS) {
        digitalWrite(cfg::PIN_PRECHARGE, LOW);
        state = State::STANDBY;
      }
      break;
    case State::ESTOP:
      state = State::STANDBY;   // chain closed again (operator acknowledged the relay)
      break;
    case State::STANDBY:
      if (commsOk && (rcManual(now) || jetsonSafetyState < 4)) state = State::ACTIVE;
      break;
    case State::ACTIVE:
      if (!commsOk) {
        safetyFlags |= WATCHDOG;
        state = State::RAMP_DOWN;
        rampStartMs = now;
      } else if (safetyFlags & (TILT_SWITCH | UNDERVOLTAGE | OVERTEMP | VESC_TIMEOUT |
                                RC_LINK_LOST)) {
        state = State::RAMP_DOWN;
        rampStartMs = now;
      }
      break;
    case State::RAMP_DOWN:
      if (now - rampStartMs > cfg::RAMP_DOWN_MS) {
        uint8_t code = FAULT_WATCHDOG;
        if (safetyFlags & VESC_TIMEOUT) code = FAULT_VESC_TIMEOUT;
        else if (safetyFlags & RC_LINK_LOST) code = FAULT_RC_LINK;
        else if (safetyFlags & UNDERVOLTAGE) code = FAULT_UNDERVOLTAGE;
        else if (safetyFlags & OVERTEMP) code = FAULT_OVERTEMP;
        if (code == FAULT_RC_LINK) {
          state = State::STANDBY;   // out of radio range is expected: stop, but no FAULT
        } else {
          enterFault(code);
        }
      }
      break;
    case State::FAULT:
      break;
  }
}

// ----------------------------------------------------------------------- actuators
static void updateDrive(float dt) {
  float tl = targetLeft, tr = targetRight, rate = cfg::MAX_ACCEL;
  if (rcManual(millis())) rcToTracks(tl, tr);
  if (state != State::ACTIVE) {
    tl = tr = 0;
    rate = cfg::RAMP_DOWN_DECEL;
  }
  outLeft = rampTo(tl, outLeft, rate, dt);
  outRight = rampTo(tr, outRight, rate, dt);
}

static bool bladeAllowed(uint32_t now) {
  const bool moving = fabsf(outLeft) > 0.05f || fabsf(outRight) > 0.05f;
  if (moving) standstillSinceMs = now;
  return state == State::ACTIVE && bladeEnableCmd && bladeRpmCmd > 0 && !rcManual(now) &&
         !(safetyFlags & (MOWER_LIFTED | TILT_SWITCH)) &&
         jetsonSafetyState <= 1 &&                      // OK or SLOW only
         now - standstillSinceMs < cfg::BLADE_IDLE_STOP_MS;
}

static void updateHeight() {
  const float pos = analogRead(cfg::PIN_HEIGHT_POS) / 1023.0f;  // 0..1 over travel
  const float target = (cutHeightCmd - cfg::MIN_CUT_HEIGHT_M) /
                       (cfg::MAX_CUT_HEIGHT_M - cfg::MIN_CUT_HEIGHT_M);
  const float err = target - pos;
  if (fabsf(err) < 0.02f || state == State::FAULT || state == State::ESTOP) {
    analogWrite(cfg::PIN_HEIGHT_PWM, 0);
    return;
  }
  digitalWrite(cfg::PIN_HEIGHT_DIR, err > 0 ? HIGH : LOW);
  analogWrite(cfg::PIN_HEIGHT_PWM, int(clampf(fabsf(err) * 2000.0f, 60, 255)));
}

// -------------------------------------------------------------------- status output
static MotorStatus motorStatus(const VescData& v) {
  MotorStatus s;
  s.rpm = int16_t(v.rpm_mech);
  s.current_da = int16_t(v.current * 10);
  s.temp_dc = int16_t(v.temp_motor * 10);
  s.duty_pm = int16_t(v.duty * 1000);
  return s;
}

static void publishStatus() {
  Status s{};
  s.state = uint8_t(state);
  s.safety_flags = safetyFlags;
  const uint32_t now = millis();
  s.vesc_ok_mask = PWM_MODE ? 0 : ((now - vLeft.last_ms < cfg::VESC_TIMEOUT_MS ? 1 : 0) |
                                    (now - vRight.last_ms < cfg::VESC_TIMEOUT_MS ? 2 : 0) |
                                    (now - vBlade.last_ms < cfg::VESC_TIMEOUT_MS ? 4 : 0));
  s.fault_code = faultCode;
  const float vbat = analogRead(cfg::PIN_BAT_SENSE) * (3.3f / 1023.0f) * cfg::BAT_DIVIDER;
  s.battery_cv = bmsLastMs ? bms.voltage_cv : uint16_t(vbat * 100);
  s.soc = bms.soc;
  s.temp_c = int8_t(tempmonGetTemp());
  sendJetson(ID_STATUS, s);
}

// ------------------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  pinMode(cfg::PIN_ESTOP_SENSE, INPUT_PULLDOWN);
  pinMode(cfg::PIN_TILT_SWITCH, INPUT_PULLUP);
  pinMode(cfg::PIN_LIFT_SWITCH, INPUT_PULLUP);
  static const uint8_t outputs[] = {
      cfg::PIN_ESTOP_SW_TRIP, cfg::PIN_PRECHARGE, cfg::PIN_SAFETY_RESET, cfg::PIN_BLADE_BRAKE,
      cfg::PIN_HEIGHT_PWM, cfg::PIN_HEIGHT_DIR, cfg::PIN_BEACON, cfg::PIN_BUZZER, cfg::PIN_LED,
      cfg::PIN_BRAKE_RELEASE};
  for (uint8_t p : outputs) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  digitalWrite(cfg::PIN_ESTOP_SW_TRIP, HIGH);   // allow the chain to close

  canJetson.begin();
  canJetson.setBaudRate(cfg::CAN_BAUD);
  canVesc.begin();
  canVesc.setBaudRate(cfg::CAN_BAUD);

  if (PWM_MODE) {
    pwmCh1.attach(cfg::PIN_PWM_CH1);
    pwmCh2.attach(cfg::PIN_PWM_CH2);
    pwmBlade.attach(cfg::PIN_PWM_BLADE);
    pwmSetTracks(0, 0);
    pwmSetBlade(0);
  }
  if (cfg::RC_INPUT_ENABLED) {
    pinMode(cfg::PIN_RC_IN_CH1, INPUT);
    pinMode(cfg::PIN_RC_IN_CH2, INPUT);
    pinMode(cfg::PIN_RC_IN_MODE, INPUT);
    attachInterrupt(digitalPinToInterrupt(cfg::PIN_RC_IN_CH1), rcIsr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(cfg::PIN_RC_IN_CH2), rcIsr2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(cfg::PIN_RC_IN_MODE), rcIsrMode, CHANGE);
  }

  bootMs = millis();
  Serial.println("Tank01 base controller ready");
}

// -------------------------------------------------------------------------- loop
void loop() {
  static uint32_t lastLoopUs = micros();
  static uint32_t tick = 0;
  const uint32_t nowUs = micros();
  if (nowUs - lastLoopUs < cfg::LOOP_US) return;
  const float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;
  ++tick;
  const uint32_t now = millis();

  CAN_message_t m;
  while (canJetson.read(m)) onJetsonFrame(m);
  while (canVesc.read(m)) onVescFrame(m);

  readSafetyInputs();
  updateState(now);
  updateDrive(dt);
  const bool blade = bladeAllowed(now);

  // Brake release only while ACTIVE / ramping (spring brakes close otherwise)
  const bool moving = fabsf(outLeft) > 0.001f || fabsf(outRight) > 0.001f;
  const bool release = (state == State::ACTIVE && (driveFlags & RELEASE_BRAKE)) ||
                       (state == State::RAMP_DOWN && moving);
  // The brake supply is additionally switched by the safety relay (docs/03, section 4).
  digitalWrite(cfg::PIN_BRAKE_RELEASE, release ? HIGH : LOW);

  if (tick % 20 == 0) {                         // 50 Hz motor commands
    if (PWM_MODE) {
      pwmSetTracks(outLeft, outRight);
      pwmSetBlade(blade ? bladeRpmCmd : 0);
    } else {
      vescSetTrackSpeed(cfg::VESC_ID_LEFT, outLeft, cfg::INVERT_LEFT);
      vescSetTrackSpeed(cfg::VESC_ID_RIGHT, outRight, cfg::INVERT_RIGHT);
      vescSetBlade(blade ? bladeRpmCmd : 0);
    }
    digitalWrite(cfg::PIN_BLADE_BRAKE, blade ? HIGH : LOW);
  }
  if (tick % 10 == 0) {                         // 100 Hz encoders
    Encoders e{int32_t(encLeft.read()), int32_t(encRight.read())};
    sendJetson(ID_ENCODERS, e);
  }
  if (tick % 50 == 0) {                         // 20 Hz status
    publishStatus();
    sendJetson(ID_MOTOR_LEFT, motorStatus(vLeft));
    sendJetson(ID_MOTOR_RIGHT, motorStatus(vRight));
    sendJetson(ID_MOTOR_BLADE, motorStatus(vBlade));
    updateHeight();
  }
  if (tick % 100 == 0) {                        // 10 Hz blade + ultrasonic
    BladeStatus b{};
    b.rpm = uint16_t(fabsf(vBlade.rpm_mech));
    b.height_mm = uint16_t(cutHeightCmd * 1000);
    b.flags = (blade ? 1 : 0) | (blade ? 0 : 2) | ((safetyFlags & MOWER_LIFTED) ? 4 : 0);
    sendJetson(ID_BLADE_STATUS, b);
    Ultrasonic u{};
    memcpy(u.dist_2cm, ultrasonic, 6);          // TODO: RS485 polling (sensor specific)
    sendJetson(ID_ULTRASONIC, u);
  }
  if (tick % 500 == 0 && bmsLastMs) sendJetson(ID_BMS, bms);  // 2 Hz

  // Beacon in ACTIVE, LED: 1 Hz STANDBY, 5 Hz ACTIVE, solid FAULT/ESTOP
  digitalWrite(cfg::PIN_BEACON, state == State::ACTIVE && (driveFlags & BEACON));
  digitalWrite(cfg::PIN_BUZZER, state == State::ACTIVE && (driveFlags & HORN));
  const uint32_t period = state == State::ACTIVE ? 200 : 1000;
  digitalWrite(cfg::PIN_LED, (state == State::FAULT || state == State::ESTOP) ? HIGH
                                                                              : (now % period) < period / 2);

  if (tick % 1000 == 0 && Serial) {
    Serial.printf("state=%u flags=0x%02x fault=%u encL=%ld encR=%ld out=%.2f/%.2f "
                  "rc=%u/%u/%u us manual=%d\n",
                  uint8_t(state), safetyFlags, faultCode, encLeft.read(), encRight.read(),
                  outLeft, outRight, rcCh1.widthUs, rcCh2.widthUs, rcMode.widthUs,
                  int(rcManual(now)));
  }
}
