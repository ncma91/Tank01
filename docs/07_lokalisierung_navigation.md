# 07 – Lokalisierung & Navigation im km‑Gelände

Das Ziel: Tank01 soll **sich auf mehreren Quadratkilometern zurechtfinden**, Verbindungswege
von mehreren Kilometern zurücklegen und auf dem Feld zentimetergenau Bahnen fahren – auch
wenn GNSS zeitweise schlecht ist (Waldrand, Hecken, Gebäude, PV‑Module).

## 1. Lokalisierungskonzept

```
                    ┌──────────────────────── ekf_global (map‑Frame, 30 Hz) ─────────────────┐
                    │ Eingänge: /odom/wheel (vx, ωz)  /imu/data (roll, pitch, ωz)            │
                    │           /odometry/gnss (x, y aus navsat_transform)                    │
                    │           /gnss/heading (yaw, absolut aus Dual‑Antenne)                 │
                    │           /kiss_icp/odometry (vx, vy, ωz)                               │
                    └─────────────────────────────▶ TF map → odom  +  /odometry/global ──────┘
                    ┌──────────────────────── ekf_local (odom‑Frame, 50 Hz) ─────────────────┐
                    │ Eingänge: /odom/wheel, /imu/data, /kiss_icp/odometry (nur Geschw.)     │
                    └─────────────────────────────▶ TF odom → base_link + /odometry/local ───┘
                    ┌──────────────────────── navsat_transform_node ─────────────────────────┐
                    │ /gnss/fix + /imu/data(heading) + /odometry/global → /odometry/gnss     │
                    │ datum = fester Bezugspunkt der Anlage (use_manual_datum)               │
                    └────────────────────────────────────────────────────────────────────────┘
```

**Warum zwei EKFs?** `odom` ist glatt und sprungfrei (gut für den Controller und lokale
Costmaps), `map` ist global korrekt, kann aber springen, wenn nach einem GNSS‑Ausfall wieder
RTK‑Fix kommt. Nav2 plant global in `map`, regelt lokal in `odom`.

### 1.1 Gewichtung über Kovarianzen

| Quelle | Kovarianz Position | Kovarianz Heading | Bemerkung |
|---|---|---|---|
| RTK‑Fix | (0,02 m)² | (0,3°)² Dual‑Antenne | Normalbetrieb |
| RTK‑Float | (0,3 m)² | (1°)² | Mähen pausiert, Transit erlaubt |
| DGNSS/Single | (1,5–3 m)² | – | nur Transit auf breiten Wegen, v ≤ 1 m/s |
| Ketten‑Odometrie | – | Gierrate schlecht (Schlupf!) | v_x mit (0,05 m/s)², ω_z mit (0,1 rad/s)² |
| IMU | – | Gierrate (0,01 rad/s)² | Roll/Nick absolut |
| KISS‑ICP | Geschwindigkeiten | Gierrate (0,02 rad/s)² | Degeneriert auf freiem Feld (wenig Struktur) |

Die GNSS‑Kovarianz kommt direkt vom Empfänger (hAcc/vAcc aus UBX‑NAV‑PVT). Ein kleiner
Filter‑Node prüft zusätzlich die **Plausibilität** (Sprünge > 3σ gegenüber Prädiktion →
Messung verwerfen, Mahalanobis‑Schwelle `*_rejection_threshold` in `ekf.yaml`).

### 1.2 Verhalten bei GNSS‑Ausfall (Koppelnavigation)

| Dauer ohne RTK | Erwartete Drift | Verhalten |
|---|---|---|
| < 30 s | < 0,3 m | Weiterfahren (Mähen erlaubt, wenn Bahn ≥ 0,5 m von Feldgrenze) |
| 30 s – 3 min | 0,3 – 2 m | Mähen pausieren, Transit mit v ≤ 1 m/s auf bekannter Route |
| > 3 min oder Kovarianz > Grenzwert | > 2 m | Stopp, Meldung, Warten auf Fix oder Fernsteuerung |

Für dauerhaft GNSS‑schwache Bereiche (Hofgelände, Waldwege) kann zusätzlich eine
**LiDAR‑Karte** aufgezeichnet werden (z. B. mit FAST‑LIO2/GLIM als Kartierung, im Betrieb
`nav2_amcl`‑ähnliche 3D‑Lokalisierung via `hdl_localization` oder Isaac ROS Visual SLAM).
Diese wird über den gleichen `map`‑Frame georeferenziert (Ausrichtung beim Kartieren mit RTK).

## 2. Weltmodell für km‑Strecken

Eine einzige Rasterkarte für mehrere km² wäre zu groß (5 km × 5 km bei 5 cm = 10¹⁰ Zellen).
Deshalb ist die Navigation **hierarchisch**:

| Ebene | Repräsentation | Umfang | Zweck |
|---|---|---|---|
| **Routengraph** | GeoJSON‑LineStrings (Wege) + Knoten (Kreuzungen, Tore, Depot, Feldeinfahrten) | gesamtes Betriebsgebiet | Globale Route Depot → Feld (Dijkstra/A* auf Graph) |
| **Feldkarte** | Polygon(e) + Sperrflächen + Mähbahnen | je Feld (bis ~20 ha) | Mähbahnplanung |
| **Rollende globale Costmap** | 2D‑Gitter 200 m × 200 m, 0,1 m, `rolling_window: true` | um das Fahrzeug | Nav2‑Planer zwischen Routenpunkten |
| **Lokale Costmap** | 2D‑Gitter 12 m × 12 m, 0,05 m | um das Fahrzeug | MPPI‑Controller, Hindernisse |
| **Höhenkarte / Traversierbarkeit** | `grid_map` 2,5D, 20 m × 20 m, 0,1 m | um das Fahrzeug | Neigung, Stufen, Rauigkeit → Costmap‑Layer |

Ablauf einer Transitfahrt:

1. `mission_node` berechnet auf dem Routengraph die Knotenfolge.
2. Die LineStrings werden in **Wegpunkte im Abstand von ~20 m** umgewandelt (WGS84 → map/ENU).
3. Nav2 `NavigateThroughPoses` fährt die Wegpunkte ab; zwischen den Wegpunkten plant der
   Smac‑Planer in der rollenden Costmap um Hindernisse herum.
4. Optional: `keepout`‑ und `speed`‑Filter‑Masken (Nav2 Costmap Filters) für Gebiete
   (z. B. „Hofbereich: max. 0,5 m/s“).

## 3. Nav2‑Konfiguration (Kernpunkte)

Siehe [`nav2_params.yaml`](../ros2_ws/src/tank01_bringup/config/nav2_params.yaml).

- **Planer:** `SmacPlannerHybrid` mit Motion‑Model `DUBIN`/`REEDS_SHEPP` → kinematisch
  glatte Pfade; Wenden auf der Stelle wird über `minimum_turning_radius: 0.0`‑nahe Werte
  bzw. `SmacPlanner2D` + `RotationShimController` ermöglicht. Auf der Grasnarbe ist
  **Wenden auf der Stelle schädlich** (Ketten reißen die Narbe auf) → im Mähmodus wird ein
  Mindestradius von 1,5 m bevorzugt, Wendemanöver im Vorgewende als „Schlaufe“ (siehe 08).
- **Controller:** `MPPIController` (Modell `DiffDrive`), Critics: `PathFollow`, `PathAlign`,
  `GoalCritic`, `ObstaclesCritic`, `CostCritic`, `PreferForward`. 56 Hz auf der Orin.
- **Costmap‑Layer:** `static_layer` (aus Feld/Keepout), `stvl_layer` (LiDAR + Tiefenkamera,
  Zerfallszeit 5 s, mit Bodenfilter), `traversability_layer` (Höhenkarte), `inflation_layer`.
- **Behavior Tree:** angepasster BT mit Recovery „rückwärts 1 m → neu planen → warten
  30 s → Meldung“; **kein Spin‑Recovery** auf Gras.

## 4. Kettenschlupf und Odometrie

Skid‑Steer‑Fahrzeuge rutschen beim Drehen systematisch. Statt des geometrischen
Spurabstands B wird eine effektive Spurweite **B_eff = χ · B** verwendet. Der Faktor χ
hängt vom Untergrund ab (Asphalt ≈ 1,2, trockene Wiese ≈ 1,5, nasse Wiese ≈ 1,8).

`tank01_base` schätzt χ online (optional, `estimate_slip: true`):

```
ω_ketten = (v_r − v_l) / B_eff          ω_imu = gemessene Gierrate
χ_neu = χ_alt + k · (ω_ketten_geo/ω_imu − χ_alt)   (nur wenn |ω_imu| > 0,15 rad/s, v stabil)
```

Zusätzlich wird der **Längsschlupf** über den Vergleich Kettengeschwindigkeit ↔
GNSS/LiDAR‑Geschwindigkeit überwacht: > 40 % Schlupf über 2 s → „festgefahren“ →
Stopp + Meldung (Ketten graben sich sonst ein).

## 5. Gelände‑Traversierbarkeit

Aus der Höhenkarte werden pro Zelle berechnet:

| Merkmal | Grenze „unbefahrbar“ | Grenze „Kosten hoch“ |
|---|---|---|
| Neigung (in Fahrtrichtung) | > 30° | > 20° |
| Neigung (quer) | > 25° | > 15° |
| Stufenhöhe | > 0,25 m | > 0,12 m |
| Rauigkeit (Std.abw. Höhe) | > 0,10 m | > 0,05 m |
| Negative Hindernisse (Graben, Loch: fehlende Bodenpunkte vor dem Fahrzeug) | Lücke > 0,4 m | – |
| Wasser (Kamera‑Segmentierung, LiDAR‑Spiegelung/Ausfall) | erkannt | – |

**Hohes Gras vs. Hindernis:** LiDAR‑Punkte bis 40–60 cm über Boden mit geringer Dichte
und hoher Durchdringung (Punkte „hinter“ dem Gras sichtbar) werden als befahrbar gewertet.
Das ist der schwierigste Teil der Wahrnehmung im Mähbetrieb; empfohlen ist ein
**gelernter Traversierbarkeits‑Klassifikator** (Kamera + LiDAR), trainiert mit
selbst aufgezeichneten Daten (vom Fahrzeug tatsächlich befahrene Flächen = positive Labels).

## 6. Mehrere Betriebszonen (> 10 km Ausdehnung)

```yaml
# tank01_bringup/config/zones.yaml (Beispiel)
zones:
  hof:      {datum: [47.2601, 11.3921, 0.0], radius_m: 3000}
  almfeld:  {datum: [47.2810, 11.4512, 0.0], radius_m: 3000}
```

Beim Verlassen des Radius einer Zone wird im Stillstand der Datumspunkt von
`navsat_transform` umgeschaltet (`/datum`‑Service) und die Karten der neuen Zone geladen.
