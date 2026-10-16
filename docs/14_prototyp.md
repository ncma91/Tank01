# 14 – Prototyp v0 (Budget ≤ 3 000 €)

Der Prototyp soll mit möglichst wenig Geld die **Software, die Lokalisierung und das
Zusammenspiel der Komponenten** im echten Gelände erproben. Er verwendet ein fertiges
Kettenchassis (Alibaba) und günstige Sensoren. Die Software ist dieselbe wie für die
Vollversion; die Unterschiede stecken im Parameter‑Profil `prototyp` und in der
Firmware‑Einstellung `DRIVE_INTERFACE = RC_PWM`.

> Ziel v0: Fernsteuern → autonom fahren (ohne Messer) → autonom mähen auf einem
> abgesperrten Testfeld (≤ 0,5 ha). Kilometerweite Transitfahrten sind mit 3 km/h
> möglich, aber nicht das Ziel dieser Stufe.

## 1. Chassis (Datenblatt Händler)

| Merkmal | Wert | Bewertung |
|---|---|---|
| Abmessungen L × B × H | 1250 × 800 × 410 mm | gut; Spurweite Mitte–Mitte ≈ 620 mm (**nachmessen**) |
| Kettenbreite | 180 mm | Bodendruck ≈ 300 kg / (2 × 0,18 × ~1,0 m) ≈ 0,08 kg/cm² – schont die Grasnarbe |
| Eigengewicht | 300 kg | schwer → Not‑Aus und Bremsen sind Pflicht, nicht optional |
| Zuladung | 500 kg | Sensoren, Mähwerk und Mast (~40 kg) sind unkritisch |
| Antrieb | 2 × 1 kW, 48 V | ausreichend; mehr als genug für 3 km/h |
| Höchstgeschwindigkeit | 3 km/h (0,83 m/s) | gut für Mähen, langsam für Transit (1 km ≈ 20 min) |
| Akku | 48 V / 70 Ah (≈ 3,4 kWh) | ≈ 2–2,5 h Mähbetrieb (siehe §6) |
| Steigfähigkeit | ≤ 45° (Herstellerangabe) | Werbewert – Software begrenzt auf 15° quer / 20° längs bis zu eigenen Tests |
| Steuerung | Fernsteuerung, Reichweite ≤ 500 m | wird für MANUAL weiterverwendet (über den Teensy, §3) |
| Steuersignal | **PWM / CAN** | PWM: sofort nutzbar; CAN: Protokoll vom Hersteller nötig |

### 1.1 Offene Fragen an den Händler

Bitte vor der Bestellung klären (schriftlich):

1. **Preis inkl. Versand nach AT**, Lieferzeit, Incoterm (DAP/DDP?). Bei Import aus China
   kommen 20 % Einfuhrumsatzsteuer + ggf. Zoll + Abfertigungsgebühr dazu.
2. **PWM‑Schnittstelle:** Kanalbelegung (Gas/Lenkung = „Arcade“ oder links/rechts =
   „Tank“?), Pulsbreiten (1000–2000 µs, Neutral 1500 µs?), Pegel (3,3 V oder 5 V?),
   Stecker, **Verhalten bei Signalverlust** (Failsafe = Stopp?).
3. **CAN‑Schnittstelle:** Protokoll‑Dokumentation (IDs, Bitrate, Geschwindigkeitsvorgabe,
   Rückmeldung von Drehzahl/Strom/Fehlern), Regler‑Typ/Hersteller.
4. **Encoder / Drehzahlrückmeldung** vorhanden? (Hall‑Sensoren der Motoren über CAN?)
5. **Bremsen:** elektromagnetische Haltebremse vorhanden? Verhalten am Hang ohne Strom?
6. Gibt es einen **Hauptschalter/Schütz** und einen Anschluss für einen externen Not‑Aus?
7. Getriebeübersetzung und Turasrad‑Durchmesser (für Encoder‑Nachrüstung).
8. Akku: Chemie (LiFePO4?), BMS, Ladegerät enthalten, Ladeanschluss.
9. Schutzart (IP) von Regler und Akku, Betriebstemperatur.
10. Montagepunkte/Lochbild auf dem Chassis für Aufbauten.

## 2. Unterschiede zur Vollversion

| Bereich | Vollversion (Kap. 00–13) | Prototyp v0 |
|---|---|---|
| Chassis | Eigenbau, 1,6 m, 8 km/h | Fertig‑Chassis 1,25 m, 3 km/h |
| Rechner | Jetson AGX Orin 64 GB | **Jetson Orin Nano Super 8 GB** |
| LiDAR | Ouster OS1‑32 | **Unitree 4D LiDAR L2** (360°, eingebaute 6‑Achs‑IMU) |
| GNSS | 2 × u‑blox ZED‑F9P (Moving Base) | **1 × Unicore UM982** (RTK + Dual‑Antennen‑Heading) |
| IMU | Xsens MTi‑630 | IMU des L2 + Madgwick‑Filter (Roll/Nick) |
| Kamera | ZED X Stereo + Personenerkennung | zunächst keine (Stufe 2: OAK‑D Lite) |
| Motoransteuerung | VESC über CAN | **Servo‑PWM** vom Teensy in den Chassis‑Regler |
| Odometrie | Encoder an den Turasrädern | aus Fahrbefehlen (Encoder‑Nachrüstung empfohlen, §5) |
| Mähwerk | Schlegelmulcher 1,0 m, 4 kW | Sichelmähwerk 0,55 m, ~1 kW BLDC + RC‑ESC |
| Not‑Aus | Sicherheitsrelais, 4 Taster, Funk‑Not‑Aus, Bumper | 2 Taster + Funk‑Not‑Aus + Schütz (§4) |

## 3. Aufbau und Signalfluss

```
                    UM982 ── ANT1 (hinten) / ANT2 (vorne), 1,0 m Basislinie
                      │ USB
  Unitree L2 ──Eth──▶ Jetson Orin Nano Super ──USB/LTE──▶ NTRIP, Fernzugriff (VPN)
  (+ IMU)             │ CAN (USB‑CAN‑Adapter, 500 kbit/s)
                      ▼
  Fernsteuer‑   ──PWM──▶ Teensy 4.1 ──PWM CH1/CH2──▶ Chassis‑Motorregler ──▶ 2 × 1 kW
  Empfänger     (CH1,CH2,      │      (statt Empfänger)
  (Original)     Modus)        └──PWM──▶ Mähwerk‑ESC ──▶ BLDC + Messer
                               │
                Not‑Aus‑Kette ─┘ (Sense)       Schütz in der 48‑V‑Leitung zum Regler
```

- Der **Empfänger der Original‑Fernsteuerung** wird nicht mehr direkt am Regler
  angeschlossen, sondern am Teensy (Pins 34/35/36). Ein freier Schalterkanal der
  Fernsteuerung wählt **MANUAL** (Durchreichen der Knüppel, funktioniert auch ohne Jetson)
  oder **AUTO** (Fahrbefehle vom Jetson).
- **Funkverlust stoppt immer** – auch im AUTO‑Modus. Die Fernsteuerung ist damit ein
  Totmann: Ist der Bediener außer Reichweite (≤ 500 m), bleibt das Fahrzeug stehen.
- Der Teensy erzeugt die Servo‑Impulse für den Chassis‑Regler. Mischung, Neutralpunkt,
  Spanne und Invertierung stehen in
  [`firmware/base_controller/include/config.h`](../firmware/base_controller/include/config.h)
  (`PWM_*`, `RC_*`) und müssen nach §8 Schritt 2 eingestellt werden.
- Jetson ↔ Teensy bleibt CAN (Orin Nano Developer Kit hat keinen CAN‑Port →
  **USB‑CAN‑Adapter**, z. B. CANable 2.0, ~40 €).

## 4. Sicherheit am Prototyp (nicht verhandelbar)

Ein 300‑kg‑Fahrzeug mit rotierendem Messer ist auch als Prototyp gefährlich.

1. **Schütz in der 48‑V‑Leitung zum Motorregler und zum Mähwerk‑ESC**, angesteuert über
   die Not‑Aus‑Kette (2 Pilztaster vorne/hinten + Empfängerrelais eines **separaten
   Funk‑Not‑Aus** mit Selbstüberwachung). Die Original‑Fernsteuerung ersetzt **keinen**
   Not‑Aus.
2. Prüfen, was der Chassis‑Regler bei fehlendem PWM‑Signal tut (Teensy‑Stecker ziehen):
   Er **muss** stehen bleiben. Wenn nicht → Freigabe‑Eingang/Schütz zwingend nutzen.
3. Mähwerk: geschlossenes Gehäuse, Kettenvorhang vorne, Messer nur im AUTO‑Modus und nur
   bei RTK‑Fix (Software), Abhebeschalter am Deck.
4. Testgelände abgesperrt, eine Person mit Funk‑Not‑Aus immer in Sichtweite.
5. Neigungsgrenzen bleiben bei 15° quer / 20° längs, bis Hangtests mit Sicherung
   (Seil) durchgeführt sind.
6. `require_perception:=false` ist nur zulässig, solange kein Mähwerk montiert ist oder
   der Bediener permanent in Reichweite des Not‑Aus steht.

## 5. Odometrie ohne Encoder

Laut Datenblatt liefert das Chassis keine Drehzahlrückmeldung. Bis das geklärt ist,
nutzt `base_node` `odometry_source: command` (Odometrie aus den Sollgeschwindigkeiten
mit 25‑fach größerer Kovarianz). Die Position kommt dann im Wesentlichen aus **RTK‑GNSS
und KISS‑ICP (LiDAR)** – bei RTK‑Fix ausreichend.

**Empfehlung:** Encoder nachrüsten (je ~20–40 €), z. B. magnetischer Absolutwertgeber
AS5047P oder kapazitiver AMT102 an einer frei zugänglichen Welle (Turasrad oder
Leitrad), Anschluss an die Teensy‑Pins 2–5. Danach `odometry_source: encoders`,
`ticks_per_rev` und `sprocket_radius` setzen. Liefert der Regler Drehzahlen über CAN,
kann stattdessen das CAN‑Protokoll eingebunden werden (Treiber auf Anfrage).

## 6. Energie und Flächenleistung

| Größe | Wert |
|---|---|
| Akku nutzbar | 48 V × 70 Ah × 0,9 ≈ 3,0 kWh |
| Fahren beim Mähen (2 × ~300 W) | 0,6 kW |
| Mähwerk 0,55 m (Mittelwert) | 0,6 kW |
| Jetson, LiDAR, GNSS, LTE | 0,04 kW |
| **Laufzeit Mähen** | **≈ 2,4 h** |
| Flächenleistung | 0,55 m × 0,88 × 0,6 m/s ≈ 1 050 m²/h, abzüglich ~20 % Wenden ≈ **0,08 ha/h** |
| Fläche pro Akkuladung | **≈ 0,2 ha** |

Für eine größere Flächenleistung wäre ein breiteres Mähwerk (0,8–1,0 m) nötig; das
Chassis ist 800 mm breit, ein Frontmähwerk bis ~0,9 m ist mechanisch möglich, braucht
aber mehr Leistung (Akku!).

## 7. Stückliste und Kosten

Preise: Richtwerte netto/brutto gemischt, Stand 10/2026, ohne Gewähr.

| Pos. | Komponente | Beispiel | ca. € |
|---|---|---|---|
| P1 | **Kettenchassis** inkl. Akku, Regler, Fernsteuerung | Alibaba (Datenblatt §1) | **Preis offen** – Budget max. ≈ 1 000–1 100 € inkl. Versand, EUSt, Zoll |
| P2 | Jetson Orin Nano Super Developer Kit (8 GB) | NVIDIA (Preis 2026 erhöht auf $399) | 350 |
| P3 | NVMe‑SSD 256 GB | – | 35 |
| P4 | Unitree 4D LiDAR L2 | Unitree Shop / RobotShop | 420 |
| P5 | UM982‑Board (Dual‑Antenne, USB‑C) | gnss.store ELT0211 | 180 |
| P6 | 2 × Multiband‑GNSS‑Antenne (L1/L2/L5) + Kabel | – | 120 |
| P7 | Teensy 4.1 + 2 × CAN‑Transceiver + Platine | PJRC | 60 |
| P8 | USB‑CAN‑Adapter | CANable 2.0 | 40 |
| P9 | Mähwerk: 48‑V‑BLDC ~1 kW + RC‑ESC + Messer/Teller + Deck‑Blech | Ersatzteile / Eigenbau | 250 |
| P10 | Not‑Aus: 2 Pilztaster, Schütz 48 V/200 A, Funk‑Not‑Aus‑Set | – | 220 |
| P11 | DC/DC 48→19 V (Jetson), 48→12 V, 48→5 V; Sicherungen, Kabel, Gehäuse IP65 | – | 150 |
| P12 | LTE‑Stick oder Handy‑Hotspot | – | 0–50 |
| P13 | Encoder‑Nachrüstung (2 Stück) – empfohlen | AS5047P / AMT102 | 60 |
| | **Summe ohne Chassis** | | **≈ 1 900–1 950 €** |
| | **Summe mit Chassis (≤ 1 050 €)** | | **≈ 3 000 €** |

Später (Stufe 2, nicht im Budget): Kamera OAK‑D Lite (~150 €) für Personenerkennung,
Korrekturdaten (APOS‑Abo oder eigene Basis mit UM980 ~150 € + Antenne).

**Einsparmöglichkeiten**, falls das Chassis teurer wird: LiDAR zunächst weglassen
(nur RTK + Fernüberwachung, kein autonomes Ausweichen → dann nur unter ständiger
Aufsicht fahren) oder Mähwerk erst in Stufe 2.

## 8. Inbetriebnahme Prototyp (Ergänzung zu Kapitel 11)

1. **Chassis im Originalzustand** mit der Original‑Fernsteuerung fahren. Failsafe prüfen:
   Sender aus → Fahrzeug muss stoppen.
2. **PWM vermessen:** Empfängerausgänge an den Teensy (Pins 34/35/36), Teensy mit
   `DRIVE_INTERFACE = RC_PWM` flashen, im seriellen Monitor die Pulsbreiten ablesen
   (Neutral, Vollausschlag, Schalterstellungen). Werte in `config.h` eintragen
   (`PWM_NEUTRAL_US`, `PWM_SPAN_US`, `PWM_MIXING`, `PWM_INVERT_*`, `RC_AUTO_US`).
3. **Teensy dazwischen:** Teensy‑PWM‑Ausgänge an den Chassis‑Regler, aufgebockt im
   MANUAL‑Modus fahren (Durchreichen). Funkverlust testen → Stopp.
4. **Höchstgeschwindigkeit kalibrieren:** 20 m bei Vollausschlag fahren, Zeit messen →
   `PWM_FULL_SCALE_SPEED` (Firmware) und `max_track_speed` (profiles/prototyp/base.yaml).
5. **Jetson:** `sudo ./scripts/setup_jetson.sh`, Unitree‑Treiber
   (`unilidar_sdk2`) im Workspace bauen, dann
   ```bash
   ros2 launch tank01_bringup bringup.launch.py profile:=prototyp use_nav:=false \
        require_perception:=false geofence:=/pfad/testfeld.geojson
   ```
   Fernsteuer‑Schalter auf AUTO, Joystick am Jetson → Fahrzeug folgt.
6. **GNSS:** UM982 einrichten (`config/um982.yaml`), Topics prüfen:
   `ros2 topic echo /gnss/status` → `rtk_fixed`; `ros2 topic hz /gnss/heading` ≈ 10 Hz.
7. **Autonom ohne Messer:** wie Kapitel 11 Schritt 9, mit `profile:=prototyp`.
8. **Mähen:** `ros2 launch tank01_mower mission.launch.py profile:=prototyp field:=… start:=true`.

## 9. Weg zur Vollversion

Alles, was am Prototyp gelernt wird (Parameter, Karten, Feld‑GeoJSONs, Rosbags,
Bahnplanung), lässt sich direkt übernehmen. Für die Vollversion werden ausgetauscht:
Chassis/Antrieb (VESC über CAN, `DRIVE_INTERFACE = VESC_CAN`), Rechner, Sensoren
(Profil leer lassen) und das Mähwerk.
