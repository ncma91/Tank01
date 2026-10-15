# 13 – Wartung & Fehlersuche

## 1. Wartungsplan

| Intervall | Tätigkeit |
|---|---|
| Vor jedem Einsatz | Sichtprüfung (Ketten, Schlegel/Messer, Kabel, Sensoren sauber), Not‑Aus‑Funktionstest (1 Taster + Funk), Akku‑SoC |
| Nach jedem Einsatz | Mähwerk reinigen (Grasreste → Unwucht/Brand), Sensoren (LiDAR‑Fenster, Kamera) mit Mikrofasertuch, Akku laden |
| 25 h | Kettenspannung, Schlegel auf Verschleiß/Bruch prüfen, Riemen Mähwerk |
| 50 h | Lager Laufrollen/Leitrad abschmieren, Schraubverbindungen (Drehmoment), Steckverbinder |
| 100 h | Getriebeöl prüfen, Bremsen (Luftspalt), Schlegel wechseln/schärfen, Wuchten |
| Monatlich | Kompletter Sicherheitsfunktionstest (Kapitel 11 §6), Firmware/Software‑Updates nur mit Test |
| Jährlich | Akku‑Kapazitätstest, Isolationsprüfung, Ketten‑Zustand, Rahmen auf Risse |
| Winter | Akku bei 50–60 % SoC, frostfrei lagern; Logik‑Schalter AUS |

## 2. Fehlersuche

| Symptom | Mögliche Ursache | Prüfung / Abhilfe |
|---|---|---|
| Fahrzeug fährt nicht, `/safety/state` = `ESTOP` | Not‑Aus‑Kette offen | Alle Taster, Bumper, Funk‑Sender prüfen; Quittieren am Fahrzeug |
| `/safety/state` = `STOP: stale imu` | IMU‑Topic veraltet | `ros2 topic hz /imu/data`, USB‑Kabel, udev‑Link `/dev/tank01/imu` |
| `STOP: localization covariance` | Kein RTK‑Fix | NTRIP‑Verbindung (`/rtcm` hz), LTE‑Empfang, Antennen‑Kabel, Abschattung |
| Fahrzeug zieht nach einer Seite | Kettenspannung ungleich, Encoder‑Zuordnung, Regler‑Parameter | Encoder‑Vorzeichen in `base.yaml`, Kettenspannung |
| Fahrzeug „schwingt“ auf Bahn | MPPI zu aggressiv, Heading‑Verzug | `PathAlign`‑Gewicht erhöhen, Zeitstempel/Latenz GNSS‑Heading prüfen |
| Drehungen werden über‑/unterschätzt | Falscher `track_width_factor` | Kalibrieren (Kapitel 11 Schritt 7) oder `estimate_slip: true` |
| Hohe Gras wird als Hindernis gesehen | Bodensegmentierung/STVL zu empfindlich | `min_obstacle_height` erhöhen, Gras‑Klassifikator, Ultraschall nicht in Costmap |
| Mähwerk schaltet ständig ab | Personen‑Fehldetektion, Abhebeschalter | `/perception/nearest_person` loggen, Konfidenzschwelle, Schalterabstand |
| Teensy‑Watchdog löst aus | Jetson überlastet, CAN‑Fehler | `tegrastats`, `ip -details -statistics link show can0` (Bus‑Off?), Abschlusswiderstände 60 Ω gesamt |
| Ketten graben sich ein | Zu nasser Boden, Wenden auf der Stelle | Mission bei Nässe verschieben, Mindestwenderadius erhöhen |
| CAN `bus-off` | Fehlender Abschluss, Masseproblem, falsche Bitrate | Messung zwischen CAN_H/CAN_L ohne Strom: 60 Ω |

## 3. Nützliche Befehle

```bash
# Systemzustand
ros2 topic echo /safety/state
ros2 topic echo /diagnostics_agg
ros2 run rqt_robot_monitor rqt_robot_monitor

# CAN
candump -td can0
ip -details -statistics link show can0

# Jetson
sudo tegrastats
journalctl -u tank01 -f

# Blackbox sichern
cp -r ~/rosbags/blackbox ~/rosbags/incident_$(date +%F_%H%M%S)   # Blackbox läuft mit ./scripts/record_bag.sh blackbox
```
