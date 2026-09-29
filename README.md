# ELRS Relay — C++ (ESP32)

Две прошивки для ESP32, заменяющие Go-бинарники:
- **Pilot** (ESP32 #1) — читает CRSF из пульта, отправляет по WiFi UDP
- **Drone** (ESP32 #2) — принимает UDP, отправляет CRSF в JR-модуль (Bandit Micro)

Обе платы работают автономно — без компьютера.

## Сборка

```bash
pip install platformio
pio run -e pilot -t upload    # ESP32 #1 (пилот)
pio run -e drone -t upload     # ESP32 #2 (дрон)
```

## Подключение

### Pilot (ESP32 #1 <-> Пульт)
| ESP32 GPIO | Пульт JR bay | Описание |
|------------|-------------|----------|
| GND        | Pin 1 (GND) | Земля |
| GPIO17 (TX) | Pin 3 (TX) | Телеметрия в пульт |
| GPIO16 (RX) | Pin 4 (RX) | Каналы из пульта |

### Drone (ESP32 #2 <-> Bandit Micro)
| ESP32 GPIO | Bandit JR bay | Описание |
|------------|--------------|----------|
| GND        | Pin 1 (GND)  | Земля |
| GPIO17 (TX) | Pin 3 (TX)   | Каналы в модуль |
| GPIO16 (RX) | Pin 4 (RX)   | Телеметрия из модуля |

### Питание Bandit Micro
Pin 2 (VBAT) — внешний BEC 7.4–16.8V. НЕ подключать к ESP32!

## Конфигурация
Все настройки — `#define` в начале каждого `main.cpp`.
