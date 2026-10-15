# 01 – Systemarchitektur

## 1. Schichtenmodell

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ Ebene 4 – Missionsebene (Jetson)                                             │
│   Feldverwaltung · Mähbahnplanung · Missions‑Zustandsautomat · Bericht/LTE   │
├──────────────────────────────────────────────────────────────────────────────┤
│ Ebene 3 – Navigation & Wahrnehmung (Jetson, ROS 2)                           │
│   EKF‑Lokalisierung · Nav2 (Planer/Controller/Costmaps) · Höhenkarte ·       │
│   Personen‑/Hinderniserkennung (TensorRT) · twist_mux                         │
├──────────────────────────────────────────────────────────────────────────────┤
│ Ebene 2 – Sicherheits‑Supervisor (Jetson, eigener Prozess, hohe Priorität)   │
│   Geofence · Kipp-/Hub‑Erkennung · Heartbeat · Geschwindigkeitsbegrenzung    │
├──────────────────────────────────────────────────────────────────────────────┤
│ Ebene 1 – Echtzeit‑I/O (Teensy 4.1 Base‑Controller, 1 kHz)                   │
│   Motorregler (CAN) · Encoder · Mähwerk · Not‑Aus‑Überwachung · Watchdog     │
├──────────────────────────────────────────────────────────────────────────────┤
│ Ebene 0 – Hardware‑Sicherheit (ohne Software)                                │
│   Not‑Aus‑Kette · Sicherheitsrelais · Hauptschütz · Funk‑Not‑Aus · Bremsen   │
└──────────────────────────────────────────────────────────────────────────────┘
```

**Grundprinzip:** Jede Ebene kann die darüber liegende überstimmen. Fällt eine obere
Ebene aus, bringt die darunter liegende das Fahrzeug in einen sicheren Zustand
(Stillstand, Mähwerk aus, Bremsen geschlossen).

## 2. Blockschaltbild Hardware

```
                 GNSS‑Antenne A (vorne)      GNSS‑Antenne B (hinten)
                          │                          │
                     ┌────┴──────────────────────────┴────┐
                     │ u‑blox ZED‑F9P (Rover) + ZED‑F9P    │──USB──┐
                     │ (Moving Base) → RTK + Heading       │       │
                     └─────────────────────────────────────┘       │
   ┌─────────────┐  Ethernet ┌──────────────────────────────┐      │
   │ 3D‑LiDAR    │──────────▶│                              │◀─────┘
   └─────────────┘           │                              │◀── USB ── IMU (Xsens MTi‑630)
   ┌─────────────┐  GMSL2 /  │     NVIDIA Jetson AGX Orin   │
   │ Stereo‑Kam. │──USB3────▶│     (ROS 2 Humble)           │◀── Ethernet ── LTE/WLAN‑Router
   └─────────────┘           │                              │
   ┌─────────────┐  USB      │                              │
   │ Rückkamera  │──────────▶│                              │
   └─────────────┘           └──────────────┬───────────────┘
                                            │ CAN0 (500 kbit/s, Jetson‑intern / USB‑CAN)
                                            ▼
                             ┌──────────────────────────────┐   GPIO   ┌─────────────────┐
   Encoder L/R ─────────────▶│      Teensy 4.1               │◀────────│ Not‑Aus‑Kette   │
   Bumper vorne/hinten ─────▶│      Base‑Controller          │────────▶│ Sicherheitsrelais│
   Ultraschall ×6 (RS485) ──▶│      (Firmware, 1 kHz)        │         └────────┬────────┘
   Neigungsschalter ────────▶│                              │                  │
   BMS (CAN) ──────────────▶ │                              │                  ▼
                             └──────────────┬───────────────┘           Hauptschütz 48 V
                                            │ CAN1 (VESC‑Bus, 500 kbit/s)
                     ┌──────────────────────┼───────────────────────┐
                     ▼                      ▼                       ▼
              Motorregler L          Motorregler R           Motorregler Mähwerk
              (VESC 75/300)          (VESC 75/300)           (VESC 75/300)
                     │                      │                       │
               BLDC + Getriebe        BLDC + Getriebe          BLDC Mähwelle
               + Federspeicher‑       + Federspeicher‑         + Höhenverstell‑
                 bremse                 bremse                   Linearaktor
```

## 3. Datenfluss Software (vereinfacht)

```
 Sensoren ──▶ Treiber‑Nodes ──▶ /points, /imu/data, /gnss/fix, /camera/*
                                     │
                     ┌───────────────┼──────────────────────────────┐
                     ▼               ▼                              ▼
              LiDAR‑Odometrie   robot_localization             Wahrnehmung
              (KISS‑ICP)        ekf_local (odom)               (Personen/Tiere,
                     │          ekf_global (map) +              Höhenkarte)
                     └────────▶ navsat_transform                    │
                                     │ TF map→odom→base_link        ▼
                                     ▼                         Costmaps (Nav2)
 Missions‑Node ── NavigateThroughPoses ──▶ Nav2 BT ── Planer ── Controller (MPPI)
      │                                                             │ /cmd_vel_nav
      │ /mower/blade_cmd                                            ▼
      │                         Teleop (/cmd_vel_teleop) ──▶ twist_mux ──▶ /cmd_vel_mux
      │                                                             │
      └────────────────────────────────▶ Safety‑Supervisor ◀────────┘
                                         (Gate, Limits)
                                              │ /cmd_vel_safe, /blade/enable
                                              ▼
                                        tank01_base (Kinematik, CAN)
                                              │ CAN
                                              ▼
                                        Teensy Base‑Controller
```

## 4. Koordinatensysteme (REP‑103/105)

| Frame | Bedeutung | Quelle |
|---|---|---|
| `earth` / WGS84 | Globale geodätische Koordinaten | GNSS |
| `map` | Lokales ENU‑System, Ursprung = Datum der Anlage (fest, z. B. Hofeinfahrt) | `navsat_transform` / `ekf_global` |
| `odom` | Kontinuierlich, driftend, ohne Sprünge | `ekf_local` |
| `base_link` | Fahrzeugmitte auf Höhe der Kettenachsen, x vorne, y links, z oben | – |
| `base_footprint` | Projektion von `base_link` auf den Boden | URDF |
| `imu_link`, `gnss_front`, `gnss_rear`, `lidar_link`, `camera_link`, `mower_link` | Sensor‑/Anbaurahmen | URDF (statisch) |

Für km‑große Areale wird **ein fester Datumspunkt** pro Betriebsgebiet verwendet. Da ENU
nur lokal eben ist, sollte der Radius um den Datumspunkt **≤ 10 km** bleiben (Fehler durch
Erdkrümmung < 1 cm/100 m Strecke). Für größere Gebiete werden mehrere Datumspunkte
(„Zonen“) definiert und beim Zonenwechsel umgeschaltet (siehe Kapitel 07).

## 5. Betriebsmodi

| Modus | Beschreibung | Mähwerk | Max. v |
|---|---|---|---|
| `STANDBY` | Fahrzeug bereit, Bremsen geschlossen | aus | 0 |
| `MANUAL` | Fernsteuerung (Funk oder Web‑UI), Sicherheitsfunktionen aktiv | freigebbar | 1,5 m/s |
| `TRANSIT` | Autonome Fahrt entlang Routengraph | aus, angehoben | 2,2 m/s |
| `MOWING` | Autonomes Mähen der Bahnen | an | 1,4 m/s |
| `DOCKING` | Anfahrt Ladestation | aus | 0,3 m/s |
| `PAUSED` | Hindernis/Person/Bediener‑Pause | aus | 0 |
| `FAULT` | Fehler – manuelle Quittierung erforderlich | aus | 0 |
| `ESTOP` | Not‑Aus ausgelöst (Hardware) | stromlos | 0 |
