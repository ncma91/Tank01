# 05 – Rechner: NVIDIA Jetson AGX Orin

## 1. Auswahl

| Modul | KI‑Leistung | RAM | Leistung | Eignung |
|---|---|---|---|---|
| **Jetson AGX Orin 64 GB** | 275 TOPS | 64 GB | 15–60 W | **Empfohlen** – Reserve für LiDAR + 2 Kameras + Segmentierung |
| Jetson AGX Orin 32 GB | 200 TOPS | 32 GB | 15–40 W | Ausreichend |
| Jetson Orin NX 16 GB | 100 TOPS | 16 GB | 10–25 W | Minimal (ohne Segmentierung, 1 Kamera) |

**Carrier‑Board:** Für den Prototyp das Developer‑Kit, für den Feldeinsatz ein
industrielles Carrier/Box‑PC (z. B. Connect Tech Rudi‑AGX, Seeed reComputer Industrial,
Forecr DSBOX) mit **CAN, isoliertem 9–36 V Eingang, M12‑Ethernet, GMSL2** und IP‑Gehäuse.

## 2. Software‑Basis

| Komponente | Version |
|---|---|
| JetPack | 6.x (L4T 36.x, Ubuntu 22.04, CUDA 12, TensorRT 10) |
| ROS 2 | Humble Hawksbill (LTS bis 05/2027) – native oder im Container |
| Container | `dustynv/ros:humble-desktop-l4t-r36.x` bzw. Isaac ROS Base |
| DDS | Cyclone DDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`), auf lokales Interface begrenzt |

> Migration auf ROS 2 Jazzy ist vorgesehen, sobald JetPack 7 / Ubuntu 24.04 stabil
> verfügbar ist; die Pakete in diesem Repo sind dafür ohne Änderung kompatibel
> (reines Python + Standard‑Nachrichten).

## 3. Einrichtung (Kurzfassung)

Das Skript [`scripts/setup_jetson.sh`](../scripts/setup_jetson.sh) automatisiert:

1. Leistungsmodus: `sudo nvpmodel -m 0` (MAXN) und `sudo jetson_clocks`
2. ROS 2 Humble + Abhängigkeiten (Nav2, robot_localization, twist_mux, …)
3. CAN‑Schnittstelle `can0` mit 500 kbit/s als systemd‑Dienst (`scripts/can_setup.sh`)
4. udev‑Regeln für feste Gerätenamen (`/dev/tank01/gnss`, `/dev/tank01/imu`, …)
5. Netzwerk: statische IP für LiDAR‑Interface (`eth1` 192.168.10.1/24), PTP
6. Chrony + gpsd (PPS) für Zeitsynchronisation
7. systemd‑Dienst `tank01.service` für den Autostart des Bringups

## 4. Ressourcenplanung

| Prozess | CPU | GPU | RAM |
|---|---|---|---|
| LiDAR‑Treiber + Vorverarbeitung | 1 Kern | – | 0,5 GB |
| KISS‑ICP LiDAR‑Odometrie | 2 Kerne | – | 1 GB |
| Kamera‑Treiber + Stereo‑Tiefe (ZED SDK) | 1 Kern | 20 % | 2 GB |
| YOLO Personen/Tiere (TensorRT) | 0,5 Kern | 25 % | 1 GB |
| Höhenkarte (elevation_mapping_cupy) | 0,5 Kern | 20 % | 2 GB |
| Nav2 (Costmaps, Planer, MPPI) | 3 Kerne | – | 1,5 GB |
| EKF, navsat, twist_mux, Safety, Base, Mission | 1 Kern | – | 0,5 GB |
| Rosbag‑Aufzeichnung (MCAP, komprimiert) | 1 Kern | – | 0,5 GB |
| **Summe** | **≈ 10 von 12 Kernen** | **≈ 65 %** | **≈ 9 GB** |

Der Safety‑Supervisor und die Base‑Brücke laufen mit **Echtzeit‑Priorität**
(`chrt -f 80`, CPU‑Isolation auf Kern 11 möglich).

## 5. Speicher & Logging

- NVMe‑SSD 1–2 TB (industrietauglich, −20…+70 °C) für Rosbags und Karten.
- Rosbag‑Profil „Blackbox“: Ringpuffer der letzten 10 min aller sicherheitsrelevanten Topics
  (ohne Rohbilder); bei Ereignis (Not‑Aus, Kollision, Fault) wird der Puffer gesichert.
- Logrotation für `~/.ros/log`.

## 6. Thermik

- Das Gehäuse muss bei 40 °C Außentemperatur und Sonneneinstrahlung ≤ 60 °C innen halten:
  Kühlkörper nach außen (IP‑geschützt), Lüfter mit Filter, Sonnenschutzdach.
- `tegrastats` wird vom Diagnose‑Node ausgewertet; bei > 85 °C SoC‑Temperatur wird die
  Mission pausiert.
