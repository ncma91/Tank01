# 11 – Inbetriebnahme Schritt für Schritt

Die Inbetriebnahme erfolgt **von unten nach oben** (Ebene 0 → 4). Jeder Schritt hat ein
Abnahmekriterium; erst wenn es erfüllt ist, geht es weiter.

## Schritt 1 – Mechanik aufgebockt

- Fahrzeug auf Böcke, Ketten frei drehend. Mähwerk ohne Messer/Schlegel oder demontiert.
- ✅ Ketten laufen leichtgängig, Spannung korrekt (Durchhang 15–25 mm), keine Schleifgeräusche.

## Schritt 2 – Elektrik ohne Motoren

1. Akku angeschlossen, Wartungsschalter AUS, Sicherungen gezogen. Polarität prüfen.
2. Logik‑Versorgung einschalten: 24/19/12/5 V messen.
3. Not‑Aus‑Kette: jeden Taster einzeln betätigen → Sicherheitsrelais fällt ab
   (LED). Funk‑Not‑Aus: Sender ausschalten → Relais fällt ab nach < 500 ms.
- ✅ Alle Spannungen ± 5 %, jeder Not‑Aus‑Kontakt wirkt.

## Schritt 3 – Base‑Controller‑Firmware

```bash
cd firmware/base_controller
pio run -t upload          # PlatformIO, Teensy 4.1 per USB
pio device monitor         # Debug‑Ausgabe 115200 Baud
```

- ✅ Status‑LED blinkt 1 Hz (STANDBY), serielle Ausgabe zeigt Encoder‑Zählerstände beim
  Drehen der Ketten von Hand, `ESTOP_SENSE` folgt der Kette.

## Schritt 4 – Motorregler (VESC)

1. Mit VESC Tool jeden Regler einzeln einrichten: Motorerkennung (FOC), Stromgrenzen
   (Fahr: 120 A Motor / 60 A Akku; Mähwerk: 100 A / 80 A), CAN‑ID (L=10, R=11, Mähwerk=12),
   CAN‑Status‑Nachrichten 1+4 mit 50 Hz, **Timeout 100 ms** → Bremsstrom.
2. Drehrichtung prüfen (vorwärts = beide Ketten vorwärts).
- ✅ Regler erscheinen auf CAN2, Teensy meldet `vesc_ok=3`.

## Schritt 5 – Jetson & ROS 2

```bash
sudo ./scripts/setup_jetson.sh
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
candump can0                                    # Teensy‑Statusframes sichtbar?
ros2 launch tank01_bringup bringup.launch.py use_nav:=false use_sensors:=false
ros2 topic echo /base/estop                     # muss Kettenzustand spiegeln
```

- ✅ `/odom/wheel` ändert sich beim Drehen der Ketten von Hand, Richtung korrekt.

## Schritt 6 – Sicherheitsfunktionen prüfen (aufgebockt!)

| Test | Durchführung | Erwartung | ✔ |
|---|---|---|---|
| Not‑Halt je Taster | Fahrbefehl 0,3 m/s, Taster drücken | Ketten stehen < 0,5 s, Bremse fällt | ☐ |
| Funk‑Not‑Aus | Sender aus / Taste | wie oben | ☐ |
| Bumper vorne/hinten | Leiste drücken | wie oben | ☐ |
| Watchdog Jetson | `base_node` beenden (Ctrl‑C) | Rampe auf 0 in ≤ 0,5 s, danach Kette offen | ☐ |
| Watchdog CAN | CAN‑Stecker Jetson ziehen | wie oben | ☐ |
| Watchdog VESC | CAN‑Stecker Regler ziehen | Regler bremst nach 100 ms | ☐ |
| Kippschutz IMU | Roll‑Grenze in `safety.yaml` auf 5° setzen, Fahrzeug leicht neigen | Zustand STOP | ☐ |
| Neigungsschalter | Schalter von Hand kippen | Not‑Halt | ☐ |
| Mähwerk Abhebeschalter | Mähwerk anheben bei laufender Welle | Welle stoppt < 3 s | ☐ |
| Kein Wiederanlauf | Not‑Halt lösen ohne Quittierung | Fahrzeug bleibt stehen | ☐ |

## Schritt 7 – Fernsteuerung auf dem Boden

- Mähwerk aus. Freie Fläche, Person mit Funk‑Not‑Aus.
- `ros2 launch tank01_bringup bringup.launch.py use_nav:=false require_perception:=false geofence:=/pfad/eigenes_gelaende.geojson`
  + Joystick (LB = Totmannschalter). `require_perception:=false` nur, solange noch kein
  Personendetektor `/perception/nearest_person` publiziert – danach immer `true`.
- Geschwindigkeit langsam steigern, Wenden, Hang (zuerst 10°, dann 20°).
- **Odometrie kalibrieren:** 50 m gerade fahren (RTK als Referenz) → `wheel_radius`
  anpassen; 10 Drehungen auf der Stelle auf Gras → `track_width_factor` anpassen.
- ✅ Fahrzeug folgt der Fernsteuerung, Odometrie‑Fehler < 2 % (gerade).

## Schritt 8 – Sensoren & Lokalisierung

1. Sensoren einzeln prüfen (`ros2 topic hz /points /imu/data /gnss/fix`).
2. Extrinsische Kalibrierung (Kapitel 04 §5), Werte in `tank01.urdf.xacro`.
3. Datum der Anlage in `ekf.yaml` (`navsat_transform.datum`) **und** als `datum` in der
   eigenen GeoJSON‑Datei eintragen (beide müssen identisch sein). Das mitgelieferte
   `beispiel_wiese.geojson` ist fiktiv – mit dessen Geofence steht das Fahrzeug an jedem
   realen Ort sofort auf `STOP: outside geofence`.
4. Rechteck 20 × 20 m abfahren, `/odometry/global` gegen `/gnss/fix` in Foxglove vergleichen.
- ✅ Position glatt, Heading stimmt (± 1°) auch im Stand, keine Sprünge > 5 cm bei RTK‑Fix.

## Schritt 9 – Autonome Navigation ohne Mähwerk

1. `ros2 launch tank01_bringup bringup.launch.py`
2. In Foxglove/RViz ein Ziel 20 m entfernt setzen. Dann Hindernis (Karton, später Person mit
   Schutzausrüstung bzw. Dummy) in den Weg stellen.
3. Geofence testen: Ziel außerhalb setzen → muss abgelehnt bzw. an der Grenze gestoppt werden.
- ✅ Ziele werden erreicht (± 0,2 m), Hindernisse umfahren, Stopp vor Dummy.

## Schritt 10 – Mähen

1. Kleines Testfeld (20 × 30 m) als GeoJSON anlegen, ohne Sperrflächen.
2. Offline‑Planung prüfen: `python3 -m tank01_mower.coverage feld.geojson`.
3. Mission starten, Mähwerk zunächst mit niedriger Drehzahl.
4. Schrittweise: Sperrflächen, Hanglagen, größere Felder, Transitstrecken.
- ✅ Streifen ohne Lücken, Querabweichung < 10 cm, Mähwerk nur auf Streifen aktiv.

## Schritt 11 – Dauerbetrieb

- Rosbag‑Blackbox aktiv, `tank01.service` aktiviert (`sudo systemctl enable tank01`).
- Wartungsplan (Kapitel 13) beginnen.
