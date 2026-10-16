# Tank01 – Autonomer, kettengetriebener ATV‑Mähroboter

Tank01 ist ein offener Entwurf für einen **autonomen Mähroboter im ATV‑Format** mit
**Kettenantrieb**, gesteuert von einem **NVIDIA Jetson AGX Orin**. Ziel ist es, große,
unebene und teils unbekannte Flächen (Wiesen, Böschungen, Streuobstwiesen, Photovoltaik‑
Freiflächen, Weg‑ und Feldränder) im **Kilometer‑Maßstab** selbstständig anzufahren, zu
mähen und dabei Hindernisse, Menschen und Tiere sicher zu erkennen.

```
             ┌──────── RTK‑GNSS (2 Antennen, Heading) ────────┐
             │                                                 │
   ┌─────────┴─────────────────────────────────────────────────┴───────┐
   │  3D‑LiDAR   Stereo‑/Tiefenkamera   Jetson AGX Orin   LTE / Funk    │
   │──────────────────────────────────────────────────────────────────│
   │  IMU · Encoder · Bumper · Ultraschall · Neigung · Not‑Aus‑Kette   │
   │════════════════ Mähwerk (Mulch/Schlegel, höhenverstellbar) ══════│
   ╰─(@)═══════════(@)═══════════(@)─╯     ╰─(@)═══════════(@)═══(@)─╯
          linke Kette (BLDC 48 V)               rechte Kette (BLDC 48 V)
```

> 💡 **Einstieg mit kleinem Budget:** Der [Prototyp v0](docs/14_prototyp.md) nutzt ein
> fertiges Kettenchassis (48 V, 2 × 1 kW, 3 km/h), Jetson Orin Nano Super, Unitree L2 und
> UM982 – Ziel ≤ 3 000 €. Start mit `profile:=prototyp`.

## Kernmerkmale

| Bereich | Lösung |
|---|---|
| Fahrwerk | Gummiketten, Skid‑Steer, 2 × BLDC 48 V / 3 kW mit Planetengetriebe, Bodenfreiheit ≥ 200 mm |
| Mähwerk | Schlegel‑/Mulchmähwerk 1,0 m, BLDC 48 V / 4 kW, elektrische Höhenverstellung 30–120 mm |
| Rechner | NVIDIA Jetson AGX Orin 64 GB (JetPack 6, Ubuntu 22.04, ROS 2 Humble) |
| Echtzeit‑I/O | Teensy 4.1 „Base‑Controller“ (3 × CAN): Motorregler, Encoder, Not‑Aus, Watchdog |
| Lokalisierung | RTK‑GNSS (u‑blox ZED‑F9P, Dual‑Antenne) + IMU + Ketten‑Odometrie + LiDAR‑Odometrie, fusioniert im EKF |
| Wahrnehmung | 3D‑LiDAR 360° (Ouster OS1‑32 / Livox Mid‑360), Stereo‑Tiefenkamera (ZED X / ZED 2i), Ultraschall, Bumper |
| Navigation | Nav2 (Smac Hybrid‑A*, MPPI‑Controller), Höhenkarte/Traversierbarkeit, GNSS‑Routengraph für km‑Strecken |
| Mähplanung | Bahnplanung (Boustrophedon) mit Vorgewende, Überlappung und Streifen aus GeoJSON‑Feldgrenzen |
| Sicherheit | Hardware‑Not‑Aus‑Kette, Funk‑Not‑Aus, Heartbeat‑Watchdog, Geofence, Kipp‑/Hub‑Erkennung, Personen­erkennung |
| Kommunikation | LTE‑Router + NTRIP‑Korrekturdaten, WLAN im Hof, 868‑MHz Funk‑Fernsteuerung |

## Repository‑Struktur

```
Tank01/
├── docs/                       Ausführliche Dokumentation (Deutsch) – START HIER
├── hardware/                   Stückliste, Verdrahtung, CAD‑Platzhalter
├── firmware/base_controller/   Teensy‑4.1‑Firmware (PlatformIO) – Echtzeit‑I/O & Watchdog
├── ros2_ws/src/
│   ├── tank01_description/     URDF/Xacro‑Modell, TF‑Baum, RViz‑Konfiguration
│   ├── tank01_bringup/         Launch‑Dateien & Parameter (Sensoren, EKF, Nav2, twist_mux)
│   ├── tank01_base/            Kettenkinematik, Odometrie, CAN‑Brücke zum Base‑Controller
│   ├── tank01_safety/          Sicherheits‑Supervisor, Geofence, Kipp‑/Heartbeat‑Überwachung
│   ├── tank01_gnss/            Treiber Unicore UM982 (RTK + Dual‑Antennen‑Heading, Prototyp)
│   └── tank01_mower/           Feldverwaltung, Mähbahnplanung, Missionsablauf, Mähwerk
├── docker/                     Container für Jetson (L4T) und Entwicklungs‑PC
├── scripts/                    Jetson‑Setup, CAN‑Einrichtung, Hilfsskripte
├── udev/                       Feste Gerätenamen für GNSS, IMU, Funk
└── .github/workflows/          CI: Lint + Unit‑Tests der ROS‑unabhängigen Kernlogik
```

## Dokumentation

| Nr. | Kapitel |
|---|---|
| 00 | [Überblick & Anforderungen](docs/00_ueberblick_anforderungen.md) |
| 01 | [Systemarchitektur](docs/01_systemarchitektur.md) |
| 02 | [Mechanik: Fahrwerk, Kettenantrieb, Mähwerk](docs/02_mechanik.md) |
| 03 | [Elektrik & Energieversorgung](docs/03_elektrik_energie.md) |
| 04 | [Sensorik](docs/04_sensorik.md) |
| 05 | [Rechner: Jetson AGX Orin](docs/05_jetson.md) |
| 06 | [Software‑Architektur (ROS 2)](docs/06_software_architektur.md) |
| 07 | [Lokalisierung & Navigation im km‑Gelände](docs/07_lokalisierung_navigation.md) |
| 08 | [Mähplanung & Missionen](docs/08_maehplanung_missionen.md) |
| 09 | [Sicherheitskonzept](docs/09_sicherheitskonzept.md) |
| 10 | [Kommunikation & Fernüberwachung](docs/10_kommunikation.md) |
| 11 | [Inbetriebnahme Schritt für Schritt](docs/11_inbetriebnahme.md) |
| 12 | [Test, Simulation & Validierung](docs/12_test_simulation.md) |
| 13 | [Wartung & Fehlersuche](docs/13_wartung_fehlersuche.md) |
| 14 | [**Prototyp v0 (≤ 3 000 €)**](docs/14_prototyp.md) |
| A | [CAN‑Protokoll Jetson ↔ Base‑Controller](docs/A_can_protokoll.md) |
| B | [Stückliste (BOM) & Kosten](hardware/stueckliste.md) |
| C | [Glossar](docs/C_glossar.md) |

## Schnellstart (Software)

```bash
# Auf dem Jetson (JetPack 6.x) – einmalig
git clone <repo-url> ~/Tank01 && cd ~/Tank01
sudo ./scripts/setup_jetson.sh          # ROS 2 Humble, Abhängigkeiten, CAN, udev

# Workspace bauen
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Gesamtsystem starten (Sensoren, Lokalisierung, Navigation, Sicherheit)
ros2 launch tank01_bringup bringup.launch.py
# … bzw. für den Budget-Prototyp (docs/14_prototyp.md)
ros2 launch tank01_bringup bringup.launch.py profile:=prototyp

# Mähmission für ein Feld starten
ros2 launch tank01_mower mission.launch.py field:=$(ros2 pkg prefix tank01_bringup)/share/tank01_bringup/missions/beispiel_wiese.geojson
```

Unit‑Tests der Kernlogik laufen ohne ROS:

```bash
pip install pytest
python -m pytest ros2_ws/src -q
```

## Projektstatus

> ⚠️ **Prototyp / Referenzentwurf.** Tank01 ist eine Maschine mit rotierenden Messern und
> mehreren hundert Kilogramm Masse. Vor jedem autonomen Betrieb ist das
> [Sicherheitskonzept](docs/09_sicherheitskonzept.md) umzusetzen und zu validieren.
> Autonomer Betrieb nur auf abgesperrtem Privatgelände und unter Aufsicht, bis eine
> Risikobeurteilung nach Maschinenrichtlinie / Maschinenverordnung (EU) 2023/1230 vorliegt.

## Lizenz

Software: MIT (siehe [LICENSE](LICENSE)). Dokumentation: CC BY‑SA 4.0.
