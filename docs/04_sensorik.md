# 04 – Sensorik

## 1. Übersicht und Aufgabenverteilung

| Sensor | Modell (Empfehlung) | Aufgabe | Schnittstelle | Rate |
|---|---|---|---|---|
| RTK‑GNSS Rover + Moving Base | 2 × u‑blox **ZED‑F9P** (z. B. ArduSimple simpleRTK2B + Lite), 2 × Survey‑Antenne (Helix/Patch mit Groundplane) | Absolute Position (1–2 cm mit RTK‑Fix), absolutes Heading (Dual‑Antenne) | USB (UBX) | 10 Hz |
| IMU | **Xsens MTi‑630** (AHRS), Alternative: Microstrain 3DM‑GX5‑25, Budget: Bosch BNO085 | Lage (Roll/Nick), Drehraten, Beschleunigung | USB/RS232 | 200 Hz |
| 3D‑LiDAR | **Ouster OS1‑32** (120 m, 360°×45°) oder **Livox Mid‑360** (Budget) | Hinderniserkennung, Höhenkarte, LiDAR‑Odometrie | Gigabit‑Ethernet | 10 Hz |
| Stereo‑Tiefenkamera vorne | **Stereolabs ZED X** (GMSL2) oder ZED 2i (USB3) / Luxonis OAK‑D Pro W | Semantik (Personen, Tiere, Weg, Wasser), Nahbereichs‑Tiefe, Video | GMSL2/USB3 | 15–30 Hz |
| Weitwinkel‑Kamera hinten | z. B. Arducam IMX477 / USB‑UVC IP67 | Rückwärtsfahrt, Personen hinten | USB | 15 Hz |
| Ketten‑Encoder | Inkrementalgeber 1024 Imp/U, RS422 (z. B. Kübler 5000) | Ketten‑Odometrie | Teensy | 100 Hz |
| Ultraschall | 6 × wetterfeste Sensoren (z. B. MaxBotix MB7389 / RS485‑Typ), 0,3–5 m | Blindbereich‑Abdeckung, hohes Gras vs. fester Körper | RS485 → Teensy | 10 Hz |
| Bumper | Sicherheits‑Schaltleiste vorne/hinten | Kollisionserkennung, direkt in Not‑Aus‑Kette | Hardware | – |
| Neigung | Mechanischer Neigungsschalter 35° + IMU | Kippschutz (redundant) | GPIO | – |
| Mähwerk‑Abhebung | Induktiver Näherungsschalter | Mähwerk angehoben → Messer aus | GPIO | – |
| Batterie | BMS (CAN) + Spannungsteiler | SoC, Zellzustände, Temperatur | CAN | 2 Hz |
| Motorregler | VESC‑Status (Strom, Drehzahl, Temperatur) | Lastüberwachung, Festfahren erkennen | CAN | 50 Hz |
| Umwelt (optional) | Regensensor, Temperatur/Feuchte | Mähen bei Nässe vermeiden | I²C → Teensy | 0,1 Hz |

## 2. Abdeckung (Draufsicht)

```
                       LiDAR 360° (Reichweite 0,8 … 100 m, vertikal ±22,5°)
                  ·  ·  ·  ·  ·  ·  ·  ·  ·  ·  ·  ·  ·
              ·          Stereo‑Kamera vorne  110° H                ·
           ·              ╲               ╱                           ·
         ·                 ╲   US1  US2  ╱                              ·
        ·             US6 ┌──┴────┴────┴─┐ US3                          ·
        ·                 │   ▓ vorne ▓  │                              ·
        ·                 │  [Bumper ==] │                              ·
        ·                 │              │                              ·
        ·                 │  Tank01      │                              ·
        ·                 │              │                              ·
        ·                 │  [Bumper ==] │                              ·
        ·             US5 └──────────────┘ US4                          ·
         ·                    Rückkamera 120°                          ·
           ·                                                         ·
                  ·  ·  ·  ·  ·  ·  ·  ·  ·  ·  ·  ·  ·
```

Der LiDAR‑Blindbereich (Radius ≈ 0,8–1,5 m je nach Montagehöhe/Neigung) wird durch
Stereo‑Kamera (vorne) und Ultraschall (rundum) abgedeckt.

## 3. Hinweise je Sensor

### 3.1 RTK‑GNSS

- **Moving‑Base‑Konfiguration:** Der hintere F9P (Moving Base) sendet RTCM 4072 an den vorderen
  (Rover). Der Rover liefert `NAV‑PVT` (Position) und `NAV‑RELPOSNED` (Vektor zur Base →
  Heading). Treiber: `ublox_dgnss` oder `ublox` (KumarRobotics) ROS‑2‑Paket.
- **Korrekturdaten:** NTRIP‑Client (`ntrip_client` ROS‑Paket) über LTE; bei Verbindungsabbruch
  bleibt RTK‑Fix ~30–60 s erhalten („RTK‑Float“ danach, ~20–50 cm).
- **Fix‑Qualität** wird ausgewertet: Kovarianz der Position fließt in den EKF ein; im Modus
  `MOWING` ist **RTK‑Fix Pflicht** (sonst Pause/Weiterfahrt mit LiDAR‑Lokalisierung, siehe 07).
- Ausgabe: `/gnss/fix` (`sensor_msgs/NavSatFix`), `/gnss/heading` (`sensor_msgs/Imu` mit nur
  Orientierung), `/gnss/vel`.

### 3.2 IMU

- Montage nahe Drehmittelpunkt, Achsen nach REP‑103 (x vorne, y links, z oben) oder per
  statischer TF korrigiert.
- **Magnetometer deaktivieren** bzw. nicht in die Fusion geben (Störungen durch Motoren und
  Stahlrahmen). Heading kommt aus Dual‑GNSS; die IMU liefert Roll/Nick + Gierrate.
- Kalibrierung Gyro‑Bias im Stand beim Start (2–5 s Stillstand).

### 3.3 LiDAR

- Ouster: Treiber `ouster-ros` (ROS 2), Livox: `livox_ros_driver2`.
- **Self‑Filter:** Punkte auf dem eigenen Fahrzeug (Box um `base_link`) entfernen.
- **Bodensegmentierung** (z. B. Patchwork++) trennt Boden von Hindernissen; Gras bis
  ca. 40 cm wird über Intensität/Dichte und Zeitverhalten als „weich“ klassifiziert.
- Regen/Staub/Gras‑Fasern erzeugen Ausreißer → Radius‑Outlier‑Filter, zeitliche Persistenz
  in der Costmap (STVL‑Layer).

### 3.4 Kamera / KI‑Wahrnehmung

- Personen‑ und Tiererkennung mit YOLO (TensorRT, FP16/INT8) auf der Orin‑GPU,
  ≥ 15 FPS bei 640×640. Isaac ROS DNN‑Inference oder `ultralytics` + TensorRT.
- Semantische Segmentierung (optional): Gras / Weg / Wasser / Gehölz / Hindernis →
  Traversierbarkeits‑Layer.
- Ergebnisse werden mit der Stereo‑Tiefe in 3D projiziert → `/perception/detections_3d`
  → Safety‑Supervisor (Personenzonen) und Costmap.

### 3.5 Ultraschall

- Im hohen Gras liefern Ultraschallsensoren Fehlechos. Deshalb: **nur im Nahbereich**
  (< 1,5 m) und als Plausibilisierung (z. B. Echo + LiDAR‑Hindernis = sicher Hindernis).
- In Nav2 als `range_sensor_layer` (optional) oder ausschließlich im Safety‑Supervisor
  als Geschwindigkeitsbegrenzung.

## 4. Zeitsynchronisation

- Jetson ist **PTP‑Grandmaster** (`ptp4l` auf der Ethernet‑Schnittstelle) für den LiDAR,
  GNSS liefert per PPS (Pulse‑per‑Second) die absolute Zeit (`gpsd` + `chrony`).
- Teensy synchronisiert seine Zeitstempel über eine CAN‑Sync‑Nachricht (siehe Anhang A).
- Ziel: alle Sensor‑Zeitstempel innerhalb ± 1 ms.

## 5. Kalibrierung

| Was | Werkzeug | Ergebnis |
|---|---|---|
| Kamera intrinsisch | `camera_calibration` (Schachbrett) bzw. Werkskalibrierung ZED | `camera_info` |
| LiDAR ↔ Kamera extrinsisch | `direct_visual_lidar_calibration` (koide3) | TF in URDF |
| LiDAR ↔ base_link | Messung + Bodenebenen‑Fit (Rollen/Nick) | TF in URDF |
| GNSS‑Antennen | Vermessen (Maßband, ± 5 mm) | TF in URDF, Basislinie |
| IMU | Statische Ausrichtung + Gyro‑Bias | TF in URDF |
| Ketten‑Odometrie | 50 m Gerade + 10 × 360° Drehung mit RTK als Referenz | `wheel_radius`, `track_width_factor` |
