# Anhang B – Stückliste (BOM) & Kostenschätzung

Preise: grobe Richtwerte netto, Stand 2026, ohne Gewähr. Hersteller/Typen sind Beispiele;
gleichwertige Alternativen sind möglich.

## 1. Rechner & Elektronik

| Pos. | Anzahl | Komponente | Beispiel | ca. € |
|---|---|---|---|---|
| 1.1 | 1 | Jetson AGX Orin 64 GB Modul + Industrie‑Carrier/Box | Connect Tech Rudi‑AGX / Seeed reComputer Industrial | 2 800 |
| 1.2 | 1 | NVMe SSD 2 TB industrial | – | 250 |
| 1.3 | 1 | Teensy 4.1 + CAN‑Transceiver (2×, z. B. SN65HVD230) + Platine | PJRC | 80 |
| 1.4 | 1 | USB‑CAN‑Adapter (falls Carrier ohne CAN) | PEAK PCAN‑USB / CANable 2.0 | 60–250 |
| 1.5 | 1 | LTE/5G‑Router mit GNSS‑Ausgang + Antennen | Teltonika RUTX50 | 450 |
| 1.6 | 1 | Gehäuse IP65 mit Kühlkörper, Kabelverschraubungen | Rittal / Spelsberg | 250 |

## 2. Sensorik

| Pos. | Anzahl | Komponente | Beispiel | ca. € |
|---|---|---|---|---|
| 2.1 | 2 | RTK‑GNSS‑Empfänger ZED‑F9P (Rover + Moving Base) | ArduSimple simpleRTK2B + simpleRTK2Blite | 500 |
| 2.2 | 2 | GNSS‑Multiband‑Antenne mit Groundplane | ArduSimple Survey Calibrated / Harxon | 300 |
| 2.3 | 1 | IMU/AHRS industriell | Xsens MTi‑630 (alt.: Microstrain 3DM‑GX5‑25) | 1 500 |
| 2.4 | 1 | 3D‑LiDAR | Ouster OS1‑32 (≈ 8 000) **oder** Livox Mid‑360 (≈ 800) | 800–8 000 |
| 2.5 | 1 | Stereo‑Tiefenkamera | Stereolabs ZED X (GMSL2) / ZED 2i (USB) | 500–700 |
| 2.6 | 1 | Rückfahrkamera IP67 USB | – | 120 |
| 2.7 | 2 | Inkrementalgeber RS422 1024 Imp. | Kübler 5000 / Sick DFS60 | 400 |
| 2.8 | 6 | Ultraschallsensor wetterfest | MaxBotix MB7389 + RS485‑Wandler | 600 |
| 2.9 | 2 | Sicherheits‑Schaltleiste mit Auswertung | Mayser / ASO | 500 |
| 2.10 | 1 | Neigungsschalter 35°, Näherungsschalter | – | 60 |

## 3. Antrieb & Energie

| Pos. | Anzahl | Komponente | Beispiel | ca. € |
|---|---|---|---|---|
| 3.1 | 2 | BLDC 48 V 3 kW + Planetengetriebe i = 20 + Federspeicherbremse | QS Motor / Golden Motor + Neugart/Apex Getriebe | 2 400 |
| 3.2 | 1 | BLDC 48 V 4 kW (Mähwerk) | Golden Motor HPM‑5000 | 500 |
| 3.3 | 3 | Motorregler mit CAN | Trampa VESC 75/300 / Flipsky 75200 | 1 200 |
| 3.4 | 1 | Akku LiFePO4 16S 200 Ah mit CAN‑BMS, IP65‑Gehäuse | Eigenbau (EVE LF280K/LF230) oder Fertigakku | 2 500 |
| 3.5 | 1 | Ladegerät 48 V / 40 A | – | 450 |
| 3.6 | 1 | Hauptschütz 48 V / 300 A, Vorladung, ANL‑Sicherung, Wartungsschalter | Albright / Gigavac | 300 |
| 3.7 | 4 | DC/DC‑Wandler (24/19/12/5 V) | Mean Well RSD / DDR | 300 |
| 3.8 | 1 | Linearaktor 24 V 1500 N mit Poti (Schnitthöhe) | – | 150 |

## 4. Sicherheit

| Pos. | Anzahl | Komponente | Beispiel | ca. € |
|---|---|---|---|---|
| 4.1 | 4 | Not‑Aus‑Taster IP67 (zwangsöffnend, 2 Öffner) | Eaton / Schneider | 200 |
| 4.2 | 1 | Sicherheitsrelais 2‑kanalig | Pilz PNOZ s4 / Phoenix PSR | 250 |
| 4.3 | 1 | Funk‑Not‑Aus / Industrie‑Fernsteuerung | HRI / Tele Radio Panther | 900 |
| 4.4 | 1 | Rundumleuchte LED gelb, Summer | – | 80 |

## 5. Mechanik

| Pos. | Anzahl | Komponente | Beispiel | ca. € |
|---|---|---|---|---|
| 5.1 | 2 | Gummikette 180 × 72 × ~38 Glieder | Minidumper‑/Minibagger‑Ersatzteil | 700 |
| 5.2 | 1 | Satz Laufrollen, Leiträder, Turasräder, Spanner | Minidumper‑Ersatzteile / Eigenbau | 900 |
| 5.3 | 1 | Rahmenmaterial S355, Zuschnitt, Schweißen, Verzinken | Lohnfertigung | 1 200 |
| 5.4 | 1 | Schlegelmulcher 1,0 m (Anbaugerät, umgebaut auf E‑Antrieb) | Kleintraktor‑Mulcher | 900 |
| 5.5 | 1 | Haube, Sensormast, Schutzbügel, Kleinteile | – | 600 |
| 5.6 | 1 | Kabel, Stecker (Deutsch DT, M12), Wellrohr, Sicherungen | – | 600 |

## 6. Summe (grob)

| Variante | Summe |
|---|---|
| **Budget** (Livox Mid‑360, ZED 2i, Orin NX, BNO085 statt Xsens) | ≈ 18 000 € |
| **Empfohlen** (wie oben gelistet, Ouster) | ≈ 30 000 € |

Hinzu kommen Werkzeuge, Prüfmittel, ggf. RTK‑Basisstation (≈ 800 €) und NTRIP‑Abo
(je nach Anbieter 0–600 €/Jahr; in Österreich z. B. APOS, in Deutschland SAPOS).
