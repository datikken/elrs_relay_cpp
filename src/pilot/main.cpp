#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// === Network ===
#define WIFI_SSID       "YOUR_WIFI"
#define WIFI_PASS       "YOUR_PASSWORD"
#define RELAY_IP        "185.xxx.xxx.xxx"
#define RELAY_PORT      14550
#define LOCAL_PORT      14551

// === Pair routing ===
#define PAIR_ID_BYTE0    0xAB
#define PAIR_ID_BYTE1    0x12
#define PAIR_ID_BYTE2    0x34
#define PAIR_ID_BYTE3    0x56
#define ROLE_PILOT       0x00

// === CRSF ===
#define CRSF_BAUD        420000
#define CRSF_SYNC        0xC8
#define CRSF_MAX_FRAME   64

// === Pins ===
#define RX_PIN           16
#define TX_PIN           17
#define LED_PIN          2

WiFiUDP udp;
uint8_t  rxBuf[CRSF_MAX_FRAME];
uint8_t  rxIdx = 0, rxLen = 0;
uint8_t  udpBuf[CRSF_MAX_FRAME];
uint8_t  txBuf[CRSF_MAX_FRAME + 5];
uint32_t framesSent = 0, framesReceived = 0, lastStatsMs = 0;
bool wifiConnected = false;

uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
    return crc;
}

void connectWiFi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500); Serial.print("."); attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.print("WiFi OK. IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi FAILED");
    }
}

bool readCrsfFrame() {
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        if (rxIdx == 0 && b != CRSF_SYNC) continue;
        rxBuf[rxIdx++] = b;
        if (rxIdx == 2) {
            rxLen = b;
            if (rxLen == 0 || rxLen > CRSF_MAX_FRAME - 3) { rxIdx = 0; rxLen = 0; continue; }
        }
        if (rxIdx >= rxLen + 3) {
            uint8_t crc = crsf_crc8(rxBuf + 2, rxLen);
            if (crc == rxBuf[rxIdx - 1]) return true;
            rxIdx = 0; rxLen = 0;
        }
        if (rxIdx >= CRSF_MAX_FRAME) { rxIdx = 0; rxLen = 0; }
    }
    return false;
}

void sendToRelay(const uint8_t *data, uint8_t len) {
    if (!wifiConnected) return;
    txBuf[0] = PAIR_ID_BYTE0;
    txBuf[1] = PAIR_ID_BYTE1;
    txBuf[2] = PAIR_ID_BYTE2;
    txBuf[3] = PAIR_ID_BYTE3;
    txBuf[4] = ROLE_PILOT;
    memcpy(txBuf + 5, data, len);

    udp.beginPacket(RELAY_IP, RELAY_PORT);
    udp.write(txBuf, len + 5);
    udp.endPacket();
    framesSent++;
}

void receiveFromRelay() {
    int sz = udp.parsePacket();
    if (sz > 0) {
        int len = udp.read(udpBuf, min(sz, (int)CRSF_MAX_FRAME));
        if (len > 0) {
            Serial2.write(udpBuf, len);
            framesReceived++;
        }
    }
}

void setup() {
    Serial.begin(115200); delay(200);
    Serial.println("\n=== ELRS RELAY v2: PILOT ===");
    Serial.printf("PairID: %02X%02X%02X%02X Role: PILOT\n",
        PAIR_ID_BYTE0, PAIR_ID_BYTE1, PAIR_ID_BYTE2, PAIR_ID_BYTE3);
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
    Serial2.begin(CRSF_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("UART2 @ 420000");
    connectWiFi();
    udp.begin(LOCAL_PORT);
    lastStatsMs = millis();
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiConnected) { wifiConnected = false; Serial.println("WiFi lost"); }
        WiFi.reconnect(); delay(100);
        if (WiFi.status() == WL_CONNECTED) {
            wifiConnected = true;
            Serial.print("WiFi back. IP: ");
            Serial.println(WiFi.localIP());
        }
    }

    if (readCrsfFrame()) {
        sendToRelay(rxBuf, rxIdx);
        rxIdx = 0; rxLen = 0;
    }

    receiveFromRelay();

    static uint32_t lastBlink = 0;
    uint32_t interval = (framesSent > 0) ? 100 : 1000;
    if (millis() - lastBlink > interval) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = millis();
    }

    if (millis() - lastStatsMs > 5000) {
        lastStatsMs = millis();
        Serial.printf("[STATS] sent=%u recv=%u wifi=%s rssi=%d\n",
            framesSent, framesReceived,
            wifiConnected ? "OK" : "FAIL",
            wifiConnected ? WiFi.RSSI() : 0);
    }
}
