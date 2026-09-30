# ELRS Relay — WiFi CRSF-мост через VPS

Проект пересылает CRSF-кадры между пультом (TX12) и JR-модулем (Bandit) через интернет
с помощью релейного сервера на VPS.

## Структура

```
elrs_relay/
├── platformio.ini          # конфигурация PlatformIO (два окружения: pilot, drone)
├── src/
│   ├── pilot/
│   │   └── main.cpp         # прошивка для ESP32 #1 (в пульте)
│   └── drone/
│       └── main.cpp         # прошивка для ESP32 #2 (рядом с Bandit)
├── relay.py                 # релейный сервер для VPS
└── README.md
```

## Схема

```
[ TX12 пульт ] ──JR-bay──→ [ ESP32 #1 pilot ]
                                   │
                              WiFi → роутер → интернет
                                   │
                              [ VPS: relay.py ] ← UDP 14550
                                   │
                              интернет → роутер → WiFi
                                   │
[ Bandit модуль ] ←─UART2─ [ ESP32 #2 drone ]
```

## Что нужно изменить перед прошивкой

В обоих файлах (src/pilot/main.cpp и src/drone/main.cpp):

```cpp
#define WIFI_SSID       "YOUR_WIFI"        // имя WiFi-сети
#define WIFI_PASS       "YOUR_PASSWORD"    // пароль WiFi
#define RELAY_IP        "185.xxx.xxx.xxx"  // публичный IP твоего VPS
```

У пилота и дрона SSID/пароль могут быть разными — каждый подключается к своему WiFi.
IP релея — одинаковый.

## Прошивка

```bash
# Пилот
pio run -e pilot -t upload

# Дрон
pio run -e drone -t upload

# Монитор
pio device monitor
```

## Запуск релея на VPS

```bash
scp relay.py user@185.xxx.xxx.xxx:~/
sudo ufw allow 14550/udp
python3 relay.py
```

## Пины ESP32 (30-pin NodeMCU)

| Пин     | Назначение         |
|---------|--------------------|
| GPIO16  | RX2 (CRSF приём)   |
| GPIO17  | TX2 (CRSF передача)|
| GPIO2   | LED (индикатор)    |
| GND     | Земля              |

## Порядок включения

1. Включить дрон (ESP32 #2 + Bandit)
2. Подождать 5 секунд
3. Включить пульт TX12 (с ESP32 #1 в JR-bay)
4. Проверить стики в Betaflight

## Светодиод

| Режим          | Мигание   |
|----------------|-----------|
| Кадры идут     | 100 мс    |
| Кадров нет     | 1000 мс   |
| Failsafe (дрон)| 200 мс    |
