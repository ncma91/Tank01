# 00 – Überblick & Anforderungen

## 1. Zweck

Tank01 ist ein autonomer Geräteträger mit Mähwerk für **große Außenflächen**, die für
klassische Mähroboter (Begrenzungsdraht, < 1 ha, ebener Rasen) ungeeignet sind:

- Wiesen und Weiden mit hohem Aufwuchs (bis ~60 cm), Grasnarbe mit Maulwurfshügeln
- Böschungen und Hänge bis 25° (≈ 47 %) Querneigung bzw. 30° Längsneigung
- Photovoltaik‑Freiflächen (Mähen zwischen und unter Modulreihen)
- Streuobstwiesen, Weg‑ und Feldränder, Ausgleichsflächen
- Verbindungsfahrten zwischen mehreren Flächen über Feld‑/Forstwege (**mehrere km**)

## 2. Betriebsszenario („Concept of Operations“)

1. **Einrichten:** Der Betreiber zeichnet Feldgrenzen, Sperrflächen und Verbindungswege als
   GeoJSON (z. B. in QGIS oder per Abfahren mit RTK‑Rover/Fernsteuerung).
2. **Missionsplanung:** Tank01 berechnet Mähbahnen (Streifen + Vorgewende) und die Route vom
   Depot zum Feld.
3. **Anfahrt:** Fahrt mit hochgeklapptem/stillstehendem Mähwerk über den Routengraph
   (bis 8 km/h), Hindernisvermeidung aktiv.
4. **Mähen:** Abfahren der Bahnen mit 2–5 km/h, Mähwerk an, Geschwindigkeit wird an
   Aufwuchs (Mähwerkslast) und Neigung angepasst.
5. **Unterbrechung:** Bei Hindernis/Person: Stop, Mähwerk aus, warten/umfahren, Meldung.
   Bei niedrigem Akku: Rückkehr zum Depot, Laden, Fortsetzen am letzten Bahnpunkt.
6. **Abschluss:** Rückfahrt, Bericht (gemähte Fläche, Zeit, Energie, Ereignisse) per LTE.

## 3. Anforderungen

### 3.1 Funktionale Anforderungen

| ID | Anforderung | Priorität |
|---|---|---|
| F‑01 | Autonomes Abfahren von Mähbahnen innerhalb eines Polygons mit ≤ 10 cm Querabweichung (RTK‑Fix) | Muss |
| F‑02 | Autonome Verbindungsfahrt über ≥ 5 km entlang eines Routengraphen | Muss |
| F‑03 | Erkennen und Umfahren/Anhalten bei statischen Hindernissen ≥ 20 cm Höhe | Muss |
| F‑04 | Erkennen von Personen/Tieren im Umkreis von 10 m, Mähwerk‑Stopp bei < 5 m | Muss |
| F‑05 | Lokalisierung bei kurzzeitigem GNSS‑Ausfall (Wald, Gebäude) für ≥ 200 m mit < 1 m Drift | Muss |
| F‑06 | Erkennung nicht befahrbaren Geländes (Graben, Stufe > 25 cm, Neigung > Grenzwert, Wasser) | Muss |
| F‑07 | Fortsetzen einer unterbrochenen Mission am letzten Bahnpunkt | Soll |
| F‑08 | Fernüberwachung und Fernsteuerung über LTE/WLAN, Live‑Video | Soll |
| F‑09 | Automatische Rückkehr zur Ladestation, Laden über Kontaktplatte | Kann |
| F‑10 | Karte der Mähqualität / Aufwuchshöhe (aus Kamera und Mähwerksstrom) | Kann |

### 3.2 Nicht‑funktionale Anforderungen

| ID | Anforderung | Wert |
|---|---|---|
| N‑01 | Leermasse | ≈ 350 kg (Ziel < 400 kg) |
| N‑02 | Abmessungen L × B × H | ≈ 1,6 × 1,2 × 0,9 m (ohne Antennenmast) |
| N‑03 | Mähbreite | 1,0 m |
| N‑04 | Flächenleistung | ≈ 0,3 ha/h bei 3,5 km/h und 15 % Überlappung/Wenden |
| N‑05 | Laufzeit | ≥ 2,5 h Mähen bzw. ≥ 25 km Transitfahrt (Akku 9,6 kWh, siehe Kap. 03) |
| N‑06 | Höchstgeschwindigkeit | 8 km/h (Transit), 5 km/h (Mähen), 1 km/h (Nähe Personen) |
| N‑07 | Schutzart | Elektronik IP65, Steckverbinder IP67 |
| N‑08 | Temperaturbereich | –5 … +40 °C Betrieb |
| N‑09 | Steigfähigkeit | 30° längs, 25° quer (statisch kippsicher bis 35°) |
| N‑10 | Reaktionszeit Not‑Aus | Fahrantrieb und Mähwerk stromlos < 100 ms, Messer‑Stillstand < 3 s |

### 3.3 Randbedingungen

- Betrieb **nur auf privatem, nicht öffentlich zugänglichem Gelände** (Verbindungsfahrten über
  öffentliche Wege sind rechtlich nicht abgedeckt – siehe Kapitel 09).
- Korrekturdaten über NTRIP (z. B. APOS/EPOSA in Österreich, SAPOS in Deutschland) oder eigene
  RTK‑Basisstation.
- Mobilfunkabdeckung ist nicht garantiert → alle sicherheitsrelevanten Funktionen laufen lokal.

## 4. Abgrenzung

Nicht Teil des Entwurfs: Straßenzulassung, Transport von Personen, Mähen von Böschungen
> 35°, Betrieb bei Nacht ohne zusätzliche Beleuchtung/Freigabe, Schneeräumung (als
Erweiterung über den Geräteträger aber möglich).

## 5. Designentscheidungen im Überblick

| Entscheidung | Begründung | Alternative |
|---|---|---|
| **Kettenantrieb** statt Räder | Geringer Bodendruck (≈ 0,15 kg/cm²), Hangstabilität, Traktion auf nassem Gras | 4×4 mit Rädern: effizienter auf Wegen, aber Rutschgefahr am Hang |
| **Skid‑Steer** (Differenzial) | Wenden auf der Stelle, einfache Mechanik | Knicklenkung: schont Grasnarbe, aber größerer Wenderadius |
| **Elektrisch 48 V** | Leise, emissionsfrei, präzise regelbar, keine Abgase für Sensoren | Hybrid (Benzin‑Generator): mehr Reichweite, mehr Wartung |
| **Jetson AGX Orin** | GPU für LiDAR/Kamera‑KI, CUDA/TensorRT, Isaac ROS, ausreichend Schnittstellen | x86 Industrie‑PC + GPU: mehr Strom, größer |
| **Separater Mikrocontroller** | Harte Echtzeit, Watchdog unabhängig vom Linux‑System | Motoren direkt vom Jetson: unsicher bei Absturz |
| **RTK‑GNSS Dual‑Antenne** | cm‑Genauigkeit und absolutes Heading auch im Stand | Single‑Antenne + Magnetometer: Heading unzuverlässig nahe Motoren |
| **ROS 2 Humble + Nav2** | Ausgereifter Navigations‑Stack, große Community, Jetson‑Unterstützung | Eigenentwicklung: hoher Aufwand |
