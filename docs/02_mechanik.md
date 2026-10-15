# 02 – Mechanik: Fahrwerk, Kettenantrieb, Mähwerk

## 1. Rahmen

- **Leiterrahmen** aus Stahl‑Vierkantrohr S355, 60 × 40 × 3 mm, verzinkt oder pulverbeschichtet.
- Zwei Längsholme tragen links/rechts je ein Kettenlaufwerk; Querträger nehmen Akku‑Wanne
  (tiefster Punkt → niedriger Schwerpunkt), Elektronikbox und Mähwerksaufhängung auf.
- **Schwerpunkt‑Ziel:** ≤ 350 mm über Boden, mittig zwischen den Ketten
  → statische Kippgrenze quer: `arctan((Spurweite/2) / h_S) = arctan(0,475/0,35) ≈ 53°`.
  Mit dynamischer Reserve und Mähwerk‑Seitenkräften wird der Betrieb auf **25° quer** begrenzt.
- Sensormast (Alu‑Profil 40×40) vorne für LiDAR und Kamera, klappbar für Transport.
- Abdeckhaube aus GFK oder HDPE, Sensoren mit Schutzbügel (Äste!).

## 2. Kettenlaufwerk

| Parameter | Wert |
|---|---|
| Kettenart | Gummikette mit Stahleinlage, 180 mm breit, Teilung 72 mm (z. B. Minibagger‑/Raupendumper‑Kette) |
| Aufstandslänge | ≈ 900 mm je Seite |
| Spurweite (Mitte–Mitte) | 950 mm |
| Bodendruck | 350 kg / (2 × 0,18 m × 0,9 m) ≈ 1080 kg/m² ≈ 0,11 kg/cm² (Mensch ≈ 0,25 kg/cm²) |
| Antrieb | Hinten liegendes Turasrad (Kettenrad), z = 9, Teilkreis‑Ø ≈ 210 mm |
| Laufrollen | 4 je Seite, gefedert (Gummitorsion oder Schwinge) |
| Spannung | Vorne liegendes Leitrad mit Fett‑Spannzylinder + Feder |

### 2.1 Antriebsauslegung

Annahmen: Masse m = 400 kg (mit Zuladung), Steigung α = 30°, Rollwiderstand Gras
c_r = 0,12, Wenden auf der Stelle als Auslegungsfall für Drehmoment.

```
Steigungskraft        F_s = m·g·sin α        = 400·9,81·0,5   ≈ 1962 N
Rollwiderstand        F_r = m·g·cos α · c_r  = 400·9,81·0,866·0,12 ≈ 408 N
Gesamt (beide Ketten) F   ≈ 2370 N   → je Kette ≈ 1185 N
Moment am Turasrad    M = F·r = 1185 N · 0,105 m ≈ 125 Nm   (+ 50 % Reserve → ≈ 190 Nm)

Wenden auf der Stelle (Skid‑Steer):  Bohrwiderstand ~ μ_l·m·g·L/4
  μ_l (seitlich, Gras) ≈ 0,5 → zusätzlich ≈ 0,5·400·9,81·0,9/4 / (Spurweite/2) …
  Praxiswert: Drehmomentbedarf beim Wenden ≈ 1,5–2 × Steigungsfall → Auslegung 250 Nm je Seite
```

Gewählt: **BLDC 48 V, 3 kW Nenn, 3000 min⁻¹** + **Planetengetriebe i = 20**
→ Abtrieb 150 min⁻¹ → v = 150/60 · π · 0,21 m ≈ **1,65 m/s (5,9 km/h) Nenn**,
Spitze bei Feldschwächung ≈ 2,2 m/s (8 km/h). Moment: 9,5 Nm · 20 · 0,9 ≈ 171 Nm Nenn,
**> 300 Nm Spitze** (kurzzeitig 2× Nennstrom).

- Hall‑Sensoren im Motor + **Inkrementalgeber 1024 Imp./U** an der Getriebeabtriebswelle
  (Ketten‑Odometrie, unabhängig vom Motorregler).
- **Federspeicher‑Bremse** (stromlos geschlossen) an jedem Motor: hält das Fahrzeug am Hang
  bei Not‑Aus oder Stromausfall.

### 2.2 Kinematik (Skid‑Steer)

```
v_l = v − ω · B_eff / 2        v_r = v + ω · B_eff / 2
B_eff = B · χ                  (χ ≈ 1,3 … 1,8 = Schlupffaktor, geländeabhängig)
```

Der effektive Spurweitenfaktor χ wird in `tank01_base` parametriert und kann online aus
dem Vergleich Ketten‑Gierrate ↔ IMU‑Gierrate geschätzt werden (siehe Kapitel 07, §4).

## 3. Mähwerk

| Parameter | Wert |
|---|---|
| Bauart | **Schlegelmulcher** (robust gegen Steine/Äste, verträgt hohen Aufwuchs) – Alternative: Sichelmulcher mit 2 Messern für Rasenflächen |
| Arbeitsbreite | 1000 mm, **Frontanbau** vor den Ketten (Ketten fahren auf gemähtem Gras); Gehäuse um ± 150 mm seitlich verschiebbar (Linearführung) für randnahes Mähen an Zäunen/Modulreihen |
| Antrieb | BLDC 48 V / 4 kW, Riemen auf Schlegelwelle, 2400 min⁻¹ an der Welle |
| Schnitthöhe | 30–120 mm, elektrischer Linearaktor (24 V, 1500 N) mit Potentiometer‑Rückmeldung |
| Aufhängung | Pendelnd (± 8°) zur Bodenanpassung, Tastrollen hinten |
| Schutz | Ketten‑/Gummivorhang vorne und hinten, Prallblech, Auswurf nach unten |
| Stoppzeit | Aktive Bremsung über Motorregler + mechanische Bremse < 3 s |

Der Mähwerksstrom wird als **Lastsignal** verwendet: Steigt er über einen Schwellwert
(dichter Aufwuchs), reduziert der Missions‑Node die Fahrgeschwindigkeit (siehe Kapitel 08).

## 4. Gewichtsbilanz (Schätzung)

| Baugruppe | Masse |
|---|---|
| Rahmen, Haube, Sensormast | 70 kg |
| Kettenlaufwerke inkl. Ketten, Rollen | 90 kg |
| Fahrmotoren + Getriebe + Bremsen (2×) | 40 kg |
| Mähwerk inkl. Motor und Aktor | 65 kg |
| Akku 48 V / 200 Ah LiFePO4 | 75 kg |
| Elektronik, Kabel, Sensoren | 15 kg |
| **Summe** | **≈ 355 kg** |

## 5. Konstruktionshinweise

- Alle Sensoren **schwingungsentkoppelt** (Gummipuffer), IMU nahe Drehmittelpunkt.
- LiDAR auf ≥ 0,9 m Höhe, leicht nach vorne geneigt (≈ 5–10°), damit der Nahbereich vor
  dem Mähwerk erfasst wird. Ein Blindbereich direkt um das Fahrzeug wird durch Ultraschall
  und Bumper abgedeckt.
- GNSS‑Antennen mit **≥ 1 m Basislinie** (vorne/hinten) für Heading‑Genauigkeit ≈ 0,3°.
  Antennen mit Groundplane, frei von Abschattung durch Mast/LiDAR.
- Kabel geschützt im Wellrohr, Steckverbinder Deutsch DT / M12.
- Einfache Zugänglichkeit von Akku (Wechselakku optional) und Elektronikbox (Klappdeckel).
- CAD‑Dateien gehören nach `hardware/cad/` (STEP + natives Format).
