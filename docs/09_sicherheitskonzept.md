# 09 – Sicherheitskonzept

> Dieses Kapitel ist ein **technischer Leitfaden, keine Konformitätsbewertung.** Wer Tank01
> baut und betreibt, ist verantwortlich für Risikobeurteilung, CE‑Kennzeichnung (sofern in
> Verkehr gebracht) und die Einhaltung lokaler Vorschriften.

## 1. Gefährdungen (Auszug Risikobeurteilung)

| Nr. | Gefährdung | Ursache | Maßnahme(n) | Ebene |
|---|---|---|---|---|
| G1 | Schnittverletzung durch Mähwerk | Person/Tier nähert sich, Hand/Fuß unter Haube | Geschlossenes Gehäuse, Kettenvorhang, Personen­erkennung (Stopp < 5 m), Messer‑Stopp < 3 s, Abhebeschalter | 0, 2, 3 |
| G2 | Herausgeschleuderte Steine/Teile | Schlegel trifft Stein | Schlegelmulcher (Auswurf nach unten), Vorhänge, Abstand zu Personen ≥ 15 m im Mähbetrieb (Warnschilder) | 0, 3 |
| G3 | Überfahren / Quetschen | Hindernis nicht erkannt, Fehllokalisierung | LiDAR + Kamera + Ultraschall + Bumper, Geschwindigkeitsbegrenzung, Bumper in Not‑Aus‑Kette | 0, 2, 3 |
| G4 | Umkippen am Hang | Neigung, Bodenwelle, Graben | Niedriger Schwerpunkt, Neigungsgrenzen (IMU + Schalter), Traversierbarkeitskarte | 0, 2, 3 |
| G5 | Verlassen des Geländes | Lokalisierungsfehler, Softwarefehler | Geofence (Safety‑Supervisor), Lokalisierungs‑Plausibilität, Watchdog | 1, 2 |
| G6 | Unkontrollierte Bewegung | Absturz Jetson, Kommunikationsfehler | Heartbeat‑Watchdog im Teensy (200 ms), CAN‑Timeout, Federspeicherbremse | 0, 1 |
| G7 | Brand / Thermisches Durchgehen | Akku‑Defekt, Kurzschluss | LiFePO4, BMS, Sicherungen, Temperaturüberwachung, Löschdecke im Depot | 0, 1 |
| G8 | Wegrollen am Hang bei Stromausfall | Akku leer, Not‑Aus | Federspeicherbremse (stromlos geschlossen) | 0 |
| G9 | Unbefugte Bedienung / Cyberangriff | Offener Fernzugriff | VPN (WireGuard), keine offenen Ports, signierte Missionsdateien, Funk‑Not‑Aus unabhängig | – |

## 2. Sicherheitsfunktionen

| SF | Funktion | Auslöser | Reaktion | Realisierung |
|---|---|---|---|---|
| SF1 | **Not‑Halt** | Taster, Funk‑Not‑Aus, Bumper | Hauptschütz öffnet, Bremsen fallen ein (Stopp‑Kategorie 0) | Hardware, Sicherheitsrelais |
| SF2 | **Watchdog** | Kein Heartbeat Jetson → Teensy > 200 ms, kein CAN zu Reglern > 100 ms | Rampe auf 0 in 0,5 s, Mähwerk aus, dann Kette öffnen (Kategorie 1) | Teensy‑Firmware |
| SF3 | **Kippschutz** | Roll > 25° / Nick > 30° (IMU) oder Neigungsschalter > 35° | Stopp, Mähwerk aus; Schalter → Not‑Halt | Safety‑Supervisor + Hardware |
| SF4 | **Mähwerk‑Verriegelung** | Mähwerk angehoben, Fahrzeug steht > 10 s, Modus ≠ MOWING/MANUAL, Person < 5 m | Mähwerk aus + gebremst | Teensy + Safety‑Supervisor |
| SF5 | **Personenschutzzonen** | Person/Tier erkannt | < 10 m: v ≤ 0,5 m/s; < 5 m: Mähwerk aus; < 3 m: Stopp | Safety‑Supervisor |
| SF6 | **Geofence** | Position außerhalb `geofence`‑Polygon oder < 0,5 m an Grenze mit Fahrtrichtung nach außen | Stopp, FAULT | Safety‑Supervisor |
| SF7 | **Lokalisierungs‑Integrität** | Positions‑Kovarianz > Grenzwert, Heading unsicher | MOWING: PAUSE; TRANSIT: v ≤ 0,5 m/s, später Stopp | Safety‑Supervisor |
| SF8 | **Festgefahren / Blockade** | Schlupf > 40 % für 2 s, Motorstrom am Limit | Stopp, Meldung | Base + Safety |
| SF9 | **Geschwindigkeitsbegrenzung** | Modus | Harte Grenzen in Firmware und Supervisor | Teensy + Supervisor |

Die Implementierung der Software‑Sicherheitsfunktionen (SF3–SF7, SF9) liegt in
[`tank01_safety/safety_logic.py`](../ros2_ws/src/tank01_safety/tank01_safety/safety_logic.py)
und ist vollständig unit‑getestet. Die Firmware‑Funktionen (SF2, SF4‑Teil, SF9) liegen in
[`firmware/base_controller/src/main.cpp`](../firmware/base_controller/src/main.cpp).

## 3. Grundsätze

1. **Fail‑safe:** Jeder Ausfall (Kabelbruch, Stromausfall, Absturz) führt zu Stillstand und
   Mähwerk aus. Kette und Bremsen arbeiten nach dem Ruhestromprinzip.
2. **Unabhängigkeit:** Not‑Halt funktioniert ohne Software. Watchdog funktioniert ohne Jetson.
3. **Diversität:** Kippschutz über IMU (Software) **und** mechanischen Schalter.
4. **Kein Überbrücken:** Software kann die Sicherheitskette öffnen, nie schließen; die
   Quittierung (Reset) erfordert eine bewusste Bedienhandlung (Taster am Fahrzeug oder
   Funk‑Sender) – kein automatischer Wiederanlauf nach Not‑Halt.
5. **Sichtbarkeit:** Gelbe Rundumleuchte im Autonombetrieb, akustisches Signal 3 s vor
   Anfahren und Mähwerkstart.

## 4. Rechtliche Hinweise (AT/DE/EU, Stand 2026 – ohne Gewähr)

- **Maschinenverordnung (EU) 2023/1230** gilt ab 20.01.2027 (bis dahin Maschinenrichtlinie
  2006/42/EG). Autonome mobile Maschinen haben dort zusätzliche Anforderungen
  (u. a. Überwachungsfunktion, sichere Fernabschaltung).
- Relevante Normen: **EN ISO 13849‑1/‑2** (Sicherheitssteuerung), **EN ISO 12100**
  (Risikobeurteilung), **EN ISO 18497** (Sicherheit hochautomatisierter
  Landmaschinen), **EN ISO 5395** (Rasenmäher), **ISO 25119** (Landtechnik
  Sicherheitssteuerungen), **IEC 62061**.
- **Funk:** 868 MHz SRD (Duty‑Cycle beachten), LTE, WLAN – nur zugelassene Module.
- **Öffentliche Straßen/Wege:** Ein autonomes Fahrzeug ohne Zulassung darf nicht auf
  öffentlichen Verkehrsflächen fahren. Verbindungsfahrten nur über Privatwege oder mit
  Begleitperson und entsprechender Genehmigung.
- **Haftpflicht:** Versicherungsschutz für den Betrieb klären.
- **Datenschutz:** Kameras erfassen ggf. Personen → Aufzeichnungen minimieren,
  Personen in gespeicherten Bildern verpixeln, Löschfristen.

## 5. Validierung

Vor der ersten autonomen Fahrt ist **jede Sicherheitsfunktion einzeln zu prüfen** und zu
protokollieren (Checkliste in Kapitel 11, §6). Wiederholung nach jeder Änderung an
Firmware, Sicherheits‑Software oder Verdrahtung sowie mindestens monatlich.
