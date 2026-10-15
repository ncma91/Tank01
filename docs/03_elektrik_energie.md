# 03 – Elektrik & Energieversorgung

> ⚠️ 48 V DC gilt noch als Schutzkleinspannung (< 60 V DC), die Akkuenergie (≈ 10 kWh) und
> Kurzschlussströme (> 2 kA) sind aber gefährlich. Arbeiten nur bei gezogenem
> Wartungsschalter, Sicherungen direkt am Akku‑Pol.

## 1. Energiebedarf

| Verbraucher | Mittel | Spitze |
|---|---|---|
| Fahrantrieb (2 Motoren), Mähen 3,5 km/h, leichtes Gelände | 1,2 kW | 6 kW |
| Fahrantrieb, Transit 6 km/h | 1,2 kW | 6 kW |
| Mähwerk (Schlegel, mittlerer Aufwuchs) | 2,0 kW | 4 kW |
| Jetson AGX Orin (MAXN) | 50 W | 60 W |
| LiDAR, Kameras, GNSS, IMU | 35 W | 45 W |
| LTE‑Router, Funk, Teensy, Ultraschall | 20 W | 25 W |
| Beleuchtung / Warnleuchte | 15 W | 30 W |
| **Summe Mähen** | **≈ 3,3 kW** | **≈ 10 kW** |

**Akku:** LiFePO4 16S, 51,2 V nominal, **200 Ah = 10,2 kWh**, nutzbar 90 % ≈ 9,2 kWh
→ **≈ 2,7 h Mähen** bzw. **≈ 7 h / 40 km Transit**. Für längere Einsätze: Wechselakku
oder 300‑Ah‑Variante (+35 kg).

## 2. Spannungsebenen

| Ebene | Erzeugung | Verbraucher |
|---|---|---|
| 48 V (40–58,4 V) | Akku über Hauptschütz | Motorregler Fahrantrieb und Mähwerk |
| 24 V / 10 A | DC/DC isoliert (Mean Well RSD‑300C‑24 o. ä.) | Linearaktor, Bremsen, LiDAR, Warnleuchte, Sicherheitsrelais |
| 19 V / 5 A | DC/DC (für Jetson AGX Orin, 9–20 V Eingang) | Jetson |
| 12 V / 10 A | DC/DC | LTE‑Router, Kamera(s), Ultraschall, Lüfter |
| 5 V / 5 A | DC/DC | Teensy, GNSS, IMU, Funkempfänger |

Die **Logik‑Versorgung (24/19/12/5 V)** hängt **vor** dem Hauptschütz am Akku (über eigenen
Schalter + Sicherung). Dadurch bleiben Jetson und Sensoren bei Not‑Aus aktiv – das Fahrzeug
kann den Vorfall protokollieren und melden, während die Antriebe stromlos sind.

## 3. Hauptstromkreis

```
 Akku 48 V ──[ANL 250 A]──[Wartungsschalter]──┬──[Vorladewiderstand + Relais]──┐
   (BMS)                                       │                                │
                                               └──[Hauptschütz K1 (48 V, 300 A)]─┴── 48 V‑Schiene
                                                        ▲ Spule                         │
                                                        │                   ┌───────────┼───────────┐
                                    Sicherheitsrelais (PNOZ / PSR) Ausgang  [125 A]    [125 A]    [100 A]
                                                                             VESC L     VESC R     VESC Mähwerk
```

- **Vorladung:** Motorregler haben große Zwischenkreis‑Kondensatoren. Vor dem Schließen von
  K1 lädt ein Widerstand (z. B. 50 Ω / 25 W) die Kondensatoren für ~1 s vor (Steuerung durch
  Teensy, aber K1 nur bei geschlossener Sicherheitskette).
- **BMS** mit CAN‑Schnittstelle (z. B. JK‑BMS, Daly, REC): SoC, Zellspannungen, Temperatur,
  Fehler → Teensy → ROS‑Topic `/battery/state`.
- **Ladeanschluss:** Rema‑ oder Anderson‑SB175‑Stecker, optional Ladekontaktplatte für Dock.

## 4. Not‑Aus‑Kette (zweikanalig)

```
 +24 V ─┬─[Not‑Aus vorne]─[Not‑Aus hinten]─[Not‑Aus links]─[Not‑Aus rechts]─[Funk‑Not‑Aus Relais]─[Bumper vorne]─[Bumper hinten]─┐
        │                         (Öffner, zwangsöffnend, 2 Kanäle)                                                               │
        └────────────────────────────────────────────────────────────────────────── Sicherheitsrelais Kanal 1 / Kanal 2 ◀─────────┘
                                                                                           │
                         Teensy „Freigabe“‑Ausgang (Software‑Watchdog) ──▶ Start/Quittier‑Eingang
                                                                                           ▼
                                                                    K1 Hauptschütz + Bremsen‑Lüftung (24 V)
```

- Not‑Aus‑Taster an **allen vier Seiten** (Pilzkopf, rot/gelb).
- **Funk‑Not‑Aus** (z. B. HRI, Autec, Tele Radio) mit eigenem Empfänger und Selbstüberwachung:
  Verbindungsverlust > 500 ms → Kette offen.
- **Bumper** als Schaltleiste (Sicherheits‑Kontaktleiste mit 8,2‑kΩ‑Abschluss) vorne und hinten.
- Der Teensy liest den Zustand der Kette über Optokoppler (`ESTOP_SENSE`) und kann die
  Kette zusätzlich öffnen (`ESTOP_SW_TRIP`), aber **nicht überbrücken**.
- **Federspeicherbremsen** werden über 24 V gelüftet, die ebenfalls hinter dem
  Sicherheitsrelais liegen → bei Not‑Aus greifen die Bremsen sofort.
- Kategorie‑Ziel: PL d / Kat. 3 nach EN ISO 13849‑1 für die Not‑Halt‑Funktion.

## 5. Verkabelung & Steckverbinder

| Signal | Kabel | Stecker |
|---|---|---|
| 48 V Leistung | 35 mm² (Akku/Schütz), 16 mm² (je Regler) | M8‑Kabelschuhe, Anderson |
| Motorphasen | 10 mm², geschirmt | Amass AS150 / Ringöse |
| CAN (Jetson↔Teensy, Teensy↔VESC) | 2×2×0,25 mm² paarverseilt, geschirmt, 120 Ω an beiden Enden | Deutsch DT 4‑pol. |
| Encoder | 4×0,25 mm² geschirmt, Differenzial (RS422) | M12 A‑kodiert 8‑pol. |
| Ultraschall RS485 | 2×2×0,25 mm² | M12 A‑kodiert 5‑pol. |
| Ethernet LiDAR | Cat6A S/FTP | M12 X‑kodiert |

Siehe `hardware/elektrik/` für Stromlaufpläne (KiCad) und Pinbelegungen.

## 6. Pinbelegung Teensy 4.1 (Base‑Controller)

| Pin | Funktion | Richtung |
|---|---|---|
| 22/23 (CAN1) | CAN zu Jetson (500 kbit/s) | – |
| 0/1 (CAN2) | CAN zu VESC‑Reglern + BMS (500 kbit/s) | – |
| 2/3 | Encoder links A/B (über RS422‑Empfänger) | Ein |
| 4/5 | Encoder rechts A/B | Ein |
| 6 | `ESTOP_SENSE` – Zustand Sicherheitskette (Optokoppler) | Ein |
| 7 | `ESTOP_SW_TRIP` – Software‑Auslösung Kette | Aus |
| 8 | `RELAY_PRECHARGE` | Aus |
| 9 | `SAFETY_RELAY_RESET` (Quittierung) | Aus |
| 10 | `BLADE_BRAKE` | Aus |
| 11/12 | `HEIGHT_ACT_PWM` / `HEIGHT_ACT_DIR` | Aus |
| A0 (14) | `HEIGHT_ACT_POS` – Potentiometer Mähhöhe | Ein (analog) |
| A1 (15) | `BAT_SENSE` – Spannungsteiler 48 V (Redundanz zum BMS) | Ein (analog) |
| 16 | `TILT_SWITCH` – mechanischer Neigungsschalter (> 35°) | Ein |
| 17 | `LIFT_SWITCH` – Mähwerk‑Abhebeschalter | Ein |
| 20/21 (Serial5) | RS485 Ultraschall‑Bus | – |
| 24 | `BEACON` – Warnleuchte | Aus |
| 25 | `BUZZER` | Aus |
| 26 | `BRAKE_RELEASE` – Federspeicherbremsen lüften (Relais, Versorgung hinter Sicherheitsrelais) | Aus |
| 13 | Status‑LED | Aus |

## 7. EMV & Erdung

- Sternpunkt‑Masse an der Minus‑Sammelschiene, Logik‑DC/DC isoliert.
- Motorphasenkabel kurz und verdrillt, Ferrite an Reglerausgängen.
- Schirme einseitig (Elektronikbox) auflegen, CAN‑Schirm beidseitig über RC.
- GNSS‑Antennenkabel getrennt von Motorleitungen führen.
