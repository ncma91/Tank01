# Base‑Controller‑Firmware (Teensy 4.1)

Echtzeit‑I/O‑Ebene von Tank01 (siehe [docs/01_systemarchitektur.md](../../docs/01_systemarchitektur.md),
[docs/03_elektrik_energie.md](../../docs/03_elektrik_energie.md) §6 und
[docs/A_can_protokoll.md](../../docs/A_can_protokoll.md)).

| Datei | Inhalt |
|---|---|
| `include/protocol.h` | CAN‑IDs und Frame‑Layouts (synchron mit `tank01_base/can_protocol.py`) |
| `include/config.h` | Pins, Übersetzungen, Grenzwerte, Zeitkonstanten |
| `src/main.cpp` | Zustandsautomat, Watchdog, VESC‑Ansteuerung, Encoder, Statusframes |

## Bauen & Flashen

```bash
pip install platformio
cd firmware/base_controller
pio run                 # bauen
pio run -t upload       # flashen (Teensy per USB, ggf. Taste am Teensy drücken)
pio device monitor      # Debug‑Ausgabe (1 Hz)
```

## Zustände

```
BOOT ──(Vorladung 1,2 s)──▶ STANDBY ──(Heartbeat + CMD_DRIVE frisch)──▶ ACTIVE
                               ▲                                          │ Timeout 200 ms /
                               │ Kette geschlossen                        │ Kippschalter / VESC‑Timeout
          Kette offen ──▶  ESTOP                                          ▼
                               ▲                                     RAMP_DOWN ──(500 ms)──▶ FAULT
                               └──────── CMD_FAULT_RESET + Quittierung Sicherheitsrelais ◀───┘
```

## Offene Punkte (TODO)

- RS485‑Abfrage der Ultraschallsensoren (sensorabhängig, derzeit Platzhalter = „kein Echo“).
- BMS‑Frame‑Layout an das verwendete BMS anpassen (`cfg::BMS_CAN_ID`, `onVescFrame`).
- `TIME_SYNC` auswerten (Offset‑Schätzung für Zeitstempel).
- Plausibilitätsprüfung Encoder ↔ VESC‑Drehzahl (`FAULT_ENCODER`).
