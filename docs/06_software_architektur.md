# 06 – Software‑Architektur (ROS 2)

## 1. Pakete in diesem Repository

| Paket | Typ | Inhalt |
|---|---|---|
| `tank01_description` | ament_cmake | URDF/Xacro, Meshes, RViz‑Konfiguration, `robot_state_publisher`‑Launch |
| `tank01_bringup` | ament_cmake | Launch‑Dateien für Gesamtsystem und Teilsysteme, alle Parameter‑YAMLs, Beispiel‑Missionen |
| `tank01_base` | ament_python | Kettenkinematik, Odometrie, CAN‑Protokoll und Brücke zum Teensy |
| `tank01_safety` | ament_python | Safety‑Supervisor (Geschwindigkeits‑Gate, Mähwerk‑Freigabe), Geofence, Kipp/Heartbeat |
| `tank01_mower` | ament_python | GeoJSON‑Felder, WGS84↔ENU, Mähbahnplanung, Missions‑Zustandsautomat |

Die **Kernlogik** jedes Python‑Pakets (z. B. `kinematics.py`, `coverage.py`, `geofence.py`,
`safety_logic.py`, `can_protocol.py`) ist **frei von ROS‑Abhängigkeiten** und wird mit
`pytest` getestet. Die ROS‑Nodes sind dünne Hüllen darum.

## 2. Externe Pakete (über rosdep / apt)

| Zweck | Paket |
|---|---|
| Zustandsschätzung | `robot_localization` (EKF, `navsat_transform_node`) |
| Navigation | `navigation2`, `nav2_bringup`, `nav2_mppi_controller`, `nav2_smac_planner` |
| Costmap‑Layer | `spatio_temporal_voxel_layer` (STVL) |
| Geschwindigkeits‑Multiplexer | `twist_mux` |
| LiDAR‑Odometrie | `kiss_icp` |
| GNSS | `ublox_dgnss` (oder `ublox`), `ntrip_client` |
| IMU | `xsens_mti_driver` |
| LiDAR | `ouster_ros` / `livox_ros_driver2` |
| Kamera | `zed_wrapper` (Stereolabs) / `depthai_ros` |
| Höhenkarte | `elevation_mapping_cupy` (GPU) oder `grid_map` |
| Fernsteuerung | `joy`, `teleop_twist_joy` |
| Visualisierung / Fernzugriff | `foxglove_bridge` |
| Diagnose | `diagnostic_aggregator` |

## 3. Node‑Graph

```
                                        ┌──────────────────────┐
  /joy ──▶ teleop_twist_joy ──/cmd_vel_teleop──▶│                      │
                                        │      twist_mux       │──/cmd_vel_mux──┐
  Nav2 velocity_smoother ──/cmd_vel_nav──────▶│ (Prioritäten, Lock)  │                │
                                        └──────────────────────┘                ▼
                                                                    ┌────────────────────────┐
  /imu/data, /gnss/fix, /odometry/global ─────────────────────────▶│  safety_supervisor     │
  /base/estop, /base/status, /perception/nearest_person ──────────▶│  (tank01_safety)       │
  /mower/blade_request ───────────────────────────────────────────▶│                        │
                                                                    └───┬───────────┬────────┘
                                                        /cmd_vel_safe   │           │ /blade/cmd
                                                                        ▼           ▼
                                                                    ┌────────────────────────┐
                                                                    │ base_node (tank01_base)│◀── /mower/height_cmd
                                                                    │ Kinematik + CAN‑Brücke │
                                                                    └───┬────────────────────┘
                                  /odom/wheel, /joint_states,           │ can0
                                  /battery/state, /base/estop           ▼
                                                                    Teensy Base‑Controller
```

## 4. Topic‑Referenz (eigene Pakete)

| Topic | Typ | Publisher | Subscriber | Bemerkung |
|---|---|---|---|---|
| `/cmd_vel_mux` | `geometry_msgs/Twist` | twist_mux | safety_supervisor | Gewählter Fahrbefehl |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | safety_supervisor | base_node | Begrenzter, freigegebener Fahrbefehl |
| `/blade/cmd` | `std_msgs/Float32` | safety_supervisor | base_node | Mähwerk‑Solldrehzahl [U/min], 0 = aus |
| `/mower/blade_request` | `std_msgs/Float32` | mission_node | safety_supervisor | Gewünschte Drehzahl |
| `/mower/height_cmd` | `std_msgs/Float32` | mission_node | base_node | Schnitthöhe [m] |
| `/odom/wheel` | `nav_msgs/Odometry` | base_node | ekf_local | Ketten‑Odometrie mit Kovarianz |
| `/joint_states` | `sensor_msgs/JointState` | base_node | robot_state_publisher | Turasrad‑Winkel |
| `/battery/state` | `sensor_msgs/BatteryState` | base_node | mission_node, UI | Aus BMS |
| `/base/estop` | `std_msgs/Bool` | base_node | safety_supervisor | `true` = Hardware‑Kette offen |
| `/base/status` | `diagnostic_msgs/DiagnosticArray` | base_node | diagnostic_aggregator | Regler‑Temperaturen, Ströme |
| `/safety/state` | `std_msgs/String` | safety_supervisor | mission_node, UI | `OK`, `SLOW`, `PAUSE`, `STOP`, `ESTOP` + Grund |
| `/safety/heartbeat` | `std_msgs/UInt32` | safety_supervisor | base_node | Lebenszeichen (→ Teensy‑Watchdog) |
| `/perception/nearest_person` | `std_msgs/Float32` | Wahrnehmung | safety_supervisor | Abstand nächste Person/Tier [m], `inf` = keine |
| `/mission/state` | `std_msgs/String` | mission_node | UI | Zustand + Fortschritt (JSON) |
| `/mission/transit` | `std_msgs/Bool` | mission_node | safety_supervisor | Transitfahrt (Mähwerk aus) → höhere Geschwindigkeitsgrenze |
| `/pause` | `std_msgs/Bool` | UI / Bediener | twist_mux (Lock), safety_supervisor | Bediener‑Pause |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 velocity_smoother | twist_mux | Nav2‑Fahrbefehl |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | teleop_twist_joy | twist_mux | Fernsteuerung |
| `/mission/coverage_path` | `nav_msgs/Path` | mission_node | RViz/Foxglove | Geplante Mähbahnen |

## 5. Prioritäten im `twist_mux`

| Quelle | Topic | Priorität | Timeout |
|---|---|---|---|
| Fernsteuerung (Joystick/Funk) | `/cmd_vel_teleop` | 100 | 0,5 s |
| Web‑UI (Foxglove) | `/cmd_vel_web` | 90 | 0,5 s |
| Nav2 | `/cmd_vel_nav` | 10 | 0,5 s |
| Lock: Pause durch Bediener | `/pause` (Bool) | 255 | – |

## 6. Lifecycle & Start‑Reihenfolge

1. `robot_state_publisher` (TF statisch)
2. Sensor‑Treiber (GNSS, IMU, LiDAR, Kamera)
3. `base_node` – stellt CAN‑Verbindung her, Fahrzeug bleibt in `STANDBY`
4. `safety_supervisor` – erst wenn alle Pflicht‑Topics frisch sind, wird `OK` gemeldet
5. Lokalisierung (`ekf_local`, `navsat_transform`, `ekf_global`) – wartet auf RTK‑Fix + Heading
6. Nav2 (Lifecycle‑Manager aktiviert Server)
7. `mission_node` – nimmt Missionen an

Der Launch [`bringup.launch.py`](../ros2_ws/src/tank01_bringup/launch/bringup.launch.py)
startet alles; Teilsysteme können einzeln über Argumente deaktiviert werden
(`use_sensors:=false` für Simulation, `use_nav:=false` für reine Fernsteuerung).

## 7. Konventionen

- Einheiten SI (REP‑103), Winkel in rad, Geschwindigkeiten in m/s.
- Parameter immer in YAML unter `tank01_bringup/config/`, nie hart im Code.
- QoS: Sensordaten `SensorDataQoS` (best effort), Befehle und Zustände `reliable`, Depth 10.
- Logging deutsch oder englisch – einheitlich englisch im Code, Doku deutsch.
- Python: PEP8 (flake8, max. 100 Zeichen), Typannotationen.
