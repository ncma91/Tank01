# Anhang A – CAN‑Protokoll Jetson ↔ Base‑Controller

- **Bus:** CAN 2.0A (11‑Bit‑IDs), **500 kbit/s**, Abschluss 120 Ω an Jetson und Teensy.
- **Byte‑Reihenfolge:** Little‑Endian. Alle Frames 8 Byte (DLC = 8), ungenutzte Bytes = 0.
- **Referenzimplementierung:** [`tank01_base/can_protocol.py`](../ros2_ws/src/tank01_base/tank01_base/can_protocol.py)
  (Python, `struct`‑Formate) und [`firmware/base_controller/include/protocol.h`](../firmware/base_controller/include/protocol.h) (C++).
  Beide müssen synchron gehalten werden – `test_can_protocol.py` prüft die Formate.
- Niedrige ID = hohe Priorität: Befehle (0x1xx) vor Status (0x2xx).

## 1. Jetson → Teensy

| ID | Name | Rate | Format (`struct`) | Inhalt |
|---|---|---|---|---|
| `0x100` | `CMD_DRIVE` | 50 Hz | `<hhBBH` | `left_mm_s` int16, `right_mm_s` int16 (Kettengeschwindigkeit, + = vorwärts), `mode` uint8, `flags` uint8, `seq` uint16 |
| `0x101` | `CMD_BLADE` | 10 Hz | `<HHB3x` | `rpm` uint16, `height_mm` uint16, `flags` uint8 (Bit0 = enable) |
| `0x102` | `HEARTBEAT` | 20 Hz | `<IB3x` | `counter` uint32 (muss steigen), `safety_state` uint8 |
| `0x103` | `TIME_SYNC` | 1 Hz | `<Q` | Jetson‑Zeit in µs (Unix) |
| `0x104` | `CMD_FAULT_RESET` | Ereignis | `<B7x` | `magic` = `0xA5` |

`mode`: 0 = STANDBY, 1 = MANUAL, 2 = AUTO.
`flags` (CMD_DRIVE): Bit0 = Bremse lüften erlaubt, Bit1 = Warnleuchte, Bit2 = Hupe.

`safety_state`: 0 = OK, 1 = SLOW, 2 = PAUSE, 3 = STOP, 4 = ESTOP (wie `/safety/state`).

## 2. Teensy → Jetson

| ID | Name | Rate | Format | Inhalt |
|---|---|---|---|---|
| `0x200` | `STATUS` | 20 Hz | `<BBBBHBb` | `state` uint8, `safety_flags` uint8, `vesc_ok_mask` uint8, `fault_code` uint8, `battery_cv` uint16 (Spannung in 10 mV), `soc` uint8 [%], `temp_c` int8 (Elektronikbox) |
| `0x201` | `ENCODERS` | 100 Hz | `<ii` | `left_ticks` int32, `right_ticks` int32 (kumulativ, Überlauf beachten) |
| `0x202` | `MOTOR_LEFT` | 20 Hz | `<hhhh` | `rpm` (Abtrieb), `current_da` [0,1 A], `temp_dc` [0,1 °C], `duty_pm` [‰] |
| `0x203` | `MOTOR_RIGHT` | 20 Hz | `<hhhh` | wie oben |
| `0x204` | `MOTOR_BLADE` | 20 Hz | `<hhhh` | wie oben (Mähwelle) |
| `0x205` | `BLADE_STATUS` | 10 Hz | `<HHB3x` | `rpm` uint16, `height_mm` uint16, `flags` uint8 (Bit0 läuft, Bit1 Bremse aktiv, Bit2 angehoben) |
| `0x206` | `ULTRASONIC` | 10 Hz | `<6B2x` | 6 × Abstand in 2‑cm‑Schritten (0 = kein Echo, 255 = ungültig) |
| `0x207` | `BMS` | 2 Hz | `<HhBbH` | `voltage_cv` uint16, `current_da` int16 (+ = Entladen), `soc` uint8, `temp_c` int8, `cell_min_mv` uint16 |

`state` (Teensy): 0 = BOOT, 1 = STANDBY, 2 = ACTIVE, 3 = RAMP_DOWN, 4 = FAULT, 5 = ESTOP.

`safety_flags`: Bit0 = Sicherheitskette offen, Bit1 = Neigungsschalter, Bit2 = Mähwerk
angehoben, Bit3 = Watchdog ausgelöst, Bit4 = VESC‑Timeout, Bit5 = Unterspannung,
Bit6 = Übertemperatur, Bit7 = Funkverbindung zur Fernsteuerung verloren (nur RC‑PWM‑Modus).

`fault_code`: 0 = keiner, 1 = Watchdog Jetson, 2 = VESC‑Timeout, 3 = Encoder‑Plausibilität,
4 = Unterspannung, 5 = Übertemperatur, 6 = Vorladung fehlgeschlagen,
7 = Funkverbindung verloren (führt zu STANDBY, nicht zu FAULT).

## 3. Zeitverhalten & Watchdog

```
Jetson   HB ─┬────┬────┬────┬────┬──── (50 ms)                 ╳ Absturz
             │    │    │    │    │                             │
Teensy   ACTIVE ──────────────────────────────────────────────┼── 200 ms ─▶ RAMP_DOWN ── 500 ms ─▶ FAULT
                                                                              (v → 0)           (Kette auf)
```

- Fehlt `HEARTBEAT` **oder** `CMD_DRIVE` > **200 ms** → `RAMP_DOWN` (Sollwerte linear auf 0
  in 0,5 s, Mähwerk sofort aus), danach `FAULT` mit geöffneter Sicherheitskette.
- `HEARTBEAT.counter` muss strikt steigen (Schutz gegen „hängende“ Prozesse, die alte Werte
  wiederholen).
- `FAULT` wird nur durch `CMD_FAULT_RESET` **und** Quittierung am Sicherheitsrelais
  verlassen.
- Grenzen in der Firmware (unabhängig vom Jetson): |Kettengeschwindigkeit| ≤ 2,4 m/s,
  Beschleunigung ≤ 1,5 m/s², Mähwerk ≤ 2600 U/min.

## 4. Prototyp: Teensy → Chassis‑Regler (RC‑PWM)

Bei `cfg::DRIVE_INTERFACE = RC_PWM` (Standard für den Prototyp, Kapitel 14) ersetzt der
Teensy den Fernsteuer‑Empfänger des Chassis: Servo‑Impulse 1000–2000 µs (1500 µs = Stopp)
auf Pin 28 (CH1) und 29 (CH2), Mähwerk‑ESC auf Pin 33. `vesc_ok_mask` ist in diesem
Modus immer 0, `MOTOR_*`‑Frames enthalten keine Messwerte.

## 5. Teensy ↔ VESC (CAN2)

Standard‑VESC‑Protokoll (29‑Bit‑Extended‑IDs): `CAN_PACKET_SET_RPM` (ID = 3 << 8 | vesc_id)
mit ERPM = mech. RPM × Polpaare × Getriebe, Status über `CAN_PACKET_STATUS` (9) und
`STATUS_4` (16). Polpaare und Getriebeübersetzung sind in `firmware/base_controller/include/config.h`
hinterlegt.
