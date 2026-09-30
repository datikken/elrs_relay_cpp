# ELRS Relay v2 - WiFi CRSF bridge via VPS with pair routing

## Architecture

```
TX12 --3 wires--> ESP32 #1 (pilot)
                     | WiFi STA -> router -> internet
                     |
                     +-- UDP --> VPS (relay) <-- UDP --+
                          (pairId + role)               |
                                                 ESP32 #2 (drone)
                                                 | WiFi STA
                                                 |
                                          4 wires -- JR Module Bandit
```

## Packet structure

```
[PAIR_ID: 4 bytes] [ROLE: 1 byte] [CRSF frame: N bytes]
                    0x00 = pilot
                    0x01 = drone
```

Relay parses header, finds pair by pairId, forwards pure CRSF to peer.

## Files

| File | Purpose |
|---|---|
| `platformio.ini` | PlatformIO: two envs (pilot / drone) |
| `src/pilot/main.cpp` | ESP32 #1 - in transmitter |
| `src/drone/main.cpp` | ESP32 #2 - near Bandit |
| `relay.py` | relay server for VPS |

## Setup before flashing

In both `main.cpp` replace:

```cpp
#define WIFI_SSID       "YOUR_WIFI"
#define WIFI_PASS       "YOUR_PASSWORD"
#define RELAY_IP        "185.xxx.xxx.xxx"
```

Make sure PAIR_ID is the same for pilot and drone.

## Flashing

```bash
pio run -e pilot -t upload
pio run -e drone -t upload
```

## Running relay on VPS

```bash
scp relay.py user@VPS_IP:~/
sudo ufw allow 14550/udp
python3 relay.py
```

## Multiple pairs

Each pair has its own PAIR_ID (4 bytes). Change PAIR_ID_BYTE0..3 in both main.cpp.

## Power-on order

1. Turn on drone -> wait 5 sec
2. Turn on transmitter -> wait 5 sec
3. Check sticks

## Power-off order

1. Turn off transmitter
2. Turn off drone
