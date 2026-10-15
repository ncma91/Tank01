# 10 – Kommunikation & Fernüberwachung

## 1. Verbindungen

| Verbindung | Technik | Zweck | Reichweite |
|---|---|---|---|
| Mobilfunk | LTE/5G‑Router (z. B. Teltonika RUTX50) mit 2 MIMO‑Antennen | NTRIP‑Korrekturdaten, Telemetrie, Fernzugriff, Video | netzabhängig |
| WLAN | Router als Client im Hof‑WLAN, alternativ Access Point | Entwicklung, große Downloads (Rosbags) | ~100 m |
| Funk‑Fernsteuerung | 868 MHz / 2,4 GHz Industrie‑Funk mit Not‑Aus (z. B. HRI, Tele Radio Panther) | Manuelles Fahren, Not‑Aus | 300–1000 m |
| RTK‑Basis (optional) | Eigene Basisstation, Korrekturdaten per LoRa 868 MHz (RTCM3) | RTK ohne Mobilfunk | 2–5 km |

## 2. Netzwerk am Fahrzeug

```
            LTE/5G ─────┐                     ┌──── WLAN (Hof)
                        ▼                     ▼
                ┌───────────────────────────────────┐
                │  Router 192.168.1.1                │  WireGuard‑VPN zum Server/Laptop
                └──────────────┬────────────────────┘
                               │ eth0 192.168.1.10
                        ┌──────┴───────┐
                        │ Jetson Orin  │ eth1 192.168.10.1 ───── LiDAR 192.168.10.20 (PTP)
                        └──────────────┘
```

- **Keine Portfreigaben** ins Internet. Zugriff ausschließlich über **WireGuard‑VPN**
  (Server mit fester IP oder Dienst wie Tailscale/Headscale).
- DDS bleibt auf das Fahrzeug beschränkt (`ROS_LOCALHOST_ONLY=0`, Cyclone‑Konfiguration nur
  `lo` + `eth0`); Fernzugriff läuft über **Foxglove Bridge** (WebSocket, Port 8765, nur im VPN).
- Bandbreite LTE: Telemetrie ≈ 20 kbit/s, Video (H.264, 640×360, 10 FPS) ≈ 500 kbit/s,
  NTRIP ≈ 5 kbit/s.

## 3. Fernüberwachung

- **Foxglove Studio** (Layout in `tank01_bringup/config/foxglove_layout.json` – anlegen):
  Karte mit Position und Mähbahnen, Kamera, Zustände, Akku, Buttons für Pause/Fortsetzen.
- **Benachrichtigungen:** Bei `FAULT`, `ESTOP`, Mission beendet → Push (z. B. ntfy.sh selbst
  gehostet, Telegram‑Bot, E‑Mail) aus einem kleinen Node `tank01_notify` (geplant).
- **Fernsteuerung über Netz** ist nur in Sichtweite oder mit Live‑Video und Latenz < 300 ms
  zulässig; der Funk‑Not‑Aus muss immer in Reichweite einer verantwortlichen Person sein,
  bis die Risikobeurteilung etwas anderes erlaubt.

## 4. NTRIP

```yaml
# tank01_bringup/config/secrets.yaml  (NICHT ins Repo – siehe .gitignore)
ntrip:
  host: "ntrip.example.at"
  port: 2101
  mountpoint: "VRS3_MSM"
  username: "..."
  password: "..."
```

Der `ntrip_client` sendet die aktuelle Position als NMEA‑GGA (für VRS) und gibt RTCM an den
GNSS‑Treiber weiter (Topic `/rtcm`).
