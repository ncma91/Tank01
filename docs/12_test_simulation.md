# 12 – Test, Simulation & Validierung

## 1. Teststufen

| Stufe | Was | Wo | Automatisch |
|---|---|---|---|
| Unit‑Tests | Kinematik, CAN‑Protokoll, Geofence, Safety‑Logik, Bahnplanung, Geo‑Umrechnung | `ros2_ws/src/*/test/` (pytest, ohne ROS) | ✅ CI bei jedem Push |
| Lint | flake8 | CI | ✅ |
| Firmware‑Build | PlatformIO Build für Teensy 4.1 | CI (optional) | ✅ |
| Integration | Launch mit Simulation, Rosbag‑Replay | Entwickler‑PC | teilweise |
| HIL | Jetson + Teensy + VESC am Prüfstand (aufgebockt) | Werkstatt | manuell |
| Feldtest | Testfeld, Protokoll | Gelände | manuell |

```bash
# Unit‑Tests lokal
pip install pytest
python -m pytest ros2_ws/src -q

# Mit ROS (colcon)
cd ros2_ws && colcon test && colcon test-result --verbose
```

## 2. Simulation

Empfohlen: **Gazebo (Harmonic)** mit `ros_gz` oder **Isaac Sim** (realistischere Sensoren,
GPU erforderlich).

- Kettenfahrzeug in Gazebo: `gz-sim-tracked-vehicle-system` (Ketten als
  Kontakt‑Oberflächen) oder vereinfacht als Diff‑Drive mit hoher Reibung + Schlupfmodell.
- Gelände: Heightmap aus Laserscan‑DGM (z. B. Land Tirol / basemap.at / BEV 1‑m‑DGM)
  der echten Fläche → realistische Hänge.
- Sensoren: GPU‑LiDAR, IMU, NavSat‑Sensor (mit Rauschen), Tiefenkamera.
- Aufruf (geplant): `ros2 launch tank01_bringup sim.launch.py world:=wiese_hang`
  → startet Gazebo, Brücken, und `bringup.launch.py use_sensors:=false use_sim_time:=true`.

## 3. Rosbag‑basierte Regressionstests

- Für jeden Feldtest wird ein MCAP‑Bag aufgezeichnet (`scripts/record_bag.sh`).
- Interessante Situationen (Person, Graben, GNSS‑Ausfall) werden als Szenario‑Bags
  archiviert; die Wahrnehmungs‑ und Lokalisierungs‑Nodes werden darauf erneut
  ausgeführt und Kennzahlen verglichen (z. B. Positionsfehler gegenüber RTK‑Referenz,
  Erkennungsrate Personen).

## 4. Kennzahlen (KPIs) für Feldtests

| KPI | Ziel |
|---|---|
| Querabweichung von der Mähbahn (RMS) bei RTK‑Fix | < 5 cm |
| Nicht gemähte Fläche (Lücken) | < 1 % |
| Mittlere Flächenleistung | ≥ 0,3 ha/h |
| Eingriffe pro Betriebsstunde (Bediener muss eingreifen) | < 0,2 |
| Fehlauslösungen Personen‑Stopp | < 1 pro Stunde |
| Verpasste Personen (Dummy‑Test, 20 Durchläufe) | 0 |
| Positionsdrift nach 200 m ohne GNSS | < 1 m |
| Energieverbrauch Mähen | Dokumentieren (kWh/ha) |
