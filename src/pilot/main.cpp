#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "crsf.h"

// ============================================================
// CONFIGURATION
//
// In Go we used a .env file read by godotenv.
// On ESP32 there's no filesystem by default, so config is
// hardcoded as #define macros — like Go `const` at package level.
// To change: edit, recompile, reflash. No hot-reload.
// ============================================================

#define WIFI_SSID       "ELRS_RELAY"
#define WIFI_PASS       "12345678"
#define DRONE_IP        "192.168.4.2"
#define DRONE_PORT      14550
#define PILOT_PORT      14551

// UART2 pins on ESP32 (NodeMCU 30-pin DevKit V1)
// Go: serial.Open("/dev/ttyUSB0", 420000)
// C++: HardwareSerial on GPIO pins (no /dev path)
#define UART_TX_PIN     17   // GPIO17 -> JR bay pin 3 (TX)
#define UART_RX_PIN     16   // GPIO16 -> JR bay pin 4 (RX)
#define UART_BAUD       420000

// ============================================================
// GLOBAL STATE
//
// In Go, these would be package-level vars or struct fields.
// In C++ on ESP32, globals live in BSS/data segment — fine.
//
// `volatile` = "compiler, don't cache this in a register,
//  it can change from interrupts." Go uses sync.Mutex or
// atomic for this; C++ on bare metal uses volatile + careful
// design (single producer, single consumer).
// ============================================================

WiFiUDP udp;   // Like Go's net.UDPConn

// Ring buffer for frames (Go: buffered channel)
#define BUF_SIZE 8
static uint8_t frameBuf[BUF_SIZE][CRSF_RC_FRAME_SIZE];
static volatile int bufHead = 0;  // write index
static volatile int bufTail = 0;  // read index

static uint8_t lastFrame[CRSF_RC_FRAME_SIZE];
static bool hasLastFrame = false;

// UART RX accumulation
static uint8_t rxBuf[64];
static volatile uint16_t rxLen = 0;
static volatile bool frameReady = false;

// UDP sequence number
static uint32_t seqNum = 0;

// Telemetry buffer (drone -> pilot -> radio)
static uint8_t telemBuf[64];

// ============================================================
// UART RX — Interrupt Service Routine (ISR)
//
// Go equivalent:
//   go func() {
//       buf := make([]byte, 64)
//       for { n, _ := port.Read(buf); ... }
//   }()
//
// In C++ on ESP32: UART fires a hardware interrupt when data
// arrives. We grab bytes in the ISR, then process in loop().
//
// IRAM_ATTR = "put this function in internal RAM" (faster,
//   needed for ISRs). Flash RAM is slower and can't be
//   accessed during certain flash operations.
//
// ISR rules (like a Go goroutine with no preemption):
// - No Serial.print (slow, may block)
// - No malloc/new (heap not thread-safe in ISR)
// - No delay()
// - Just grab data, set flag, return
// ============================================================
void IRAM_ATTR onUartRx() {
    while (Serial2.available()) {
        if (rxLen < sizeof(rxBuf)) {
            rxBuf[rxLen++] = Serial2.read();
        } else {
            Serial2.read();  // discard on overflow
        }
    }
    // Check if we have a complete frame
    if (rxLen >= 4 && rxBuf[0] == CRSF_SYNC_BYTE_TX) {
        uint8_t expectedLen = rxBuf[1] + 2;
        if (rxLen >= expectedLen) {
            frameReady = true;
        }
    }
}

// ============================================================
// processFrame — called from loop() when frameReady is true
//
// Go: func processFrame(frame []byte) { ... }
// ============================================================
void processFrame() {
    if (!frameReady) return;
    frameReady = false;

    uint8_t frameLen = rxBuf[1] + 2;

    // Verify CRC
    uint8_t crc = crsf_crc8(&rxBuf[2], rxBuf[1] - 1);
    if (crc != rxBuf[frameLen - 1]) {
        rxLen = 0;  // bad CRC — discard
        return;
    }

    // If RC channels frame, forward via UDP to drone
    if (rxBuf[2] == CRSF_FRAME_TYPE_RC) {
        // Store in ring buffer
        memcpy(frameBuf[bufHead], rxBuf, CRSF_RC_FRAME_SIZE);
        bufHead = (bufHead + 1) % BUF_SIZE;

        // Build UDP packet
        UdpPacket pkt;
        pkt.seq = seqNum++;
        pkt.timestamp = millis();  // like time.Now().UnixMilli()
        memcpy(pkt.crsf, rxBuf, CRSF_RC_FRAME_SIZE);

        // Serialize and send
        uint8_t udpData[UDP_PAYLOAD_SIZE];
        udp_packet_serialize(&pkt, udpData);

        // Go: conn.WriteToUDP(udpData, dst)
        udp.beginPacket(DRONE_IP, DRONE_PORT);
        udp.write(udpData, UDP_PAYLOAD_SIZE);
        udp.endPacket();
    }

    rxLen = 0;  // reset for next frame
}

// ============================================================
// receiveTelemetry — UDP RX from drone
//
// Go:
//   buf := make([]byte, 1024)
//   n, _, _ := conn.ReadFromUDP(buf)
// ============================================================
void receiveTelemetry() {
    int packetSize = udp.parsePacket();  // like checking conn for data
    if (packetSize > 0 && packetSize <= (int)sizeof(telemBuf)) {
        udp.read(telemBuf, packetSize);

        // Forward to radio's telemetry input
        if (telemBuf[0] == CRSF_SYNC_BYTE_RX) {
            Serial2.write(telemBuf, packetSize);
        }
    }
}

// ============================================================
// setup() — runs once at boot
//
// Go equivalent: func main() { ... initialization ... }
//
// In Arduino, `setup()` is called once, then `loop()` runs
// forever. This replaces Go's `func main()` + `for {}` pattern.
// ============================================================
void setup() {
    // Debug serial (USB, 115200) — like log.Println in Go
    Serial.begin(115200);
    Serial.println("=== ELRS Relay: PILOT ===");

    // CRSF UART: GPIO16=RX, GPIO17=TX, 420000 baud
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.onReceive(onUartRx);  // attach ISR

    // WiFi Access Point (AP)
    // Go: not applicable — ESP32 WiFi is hardware, not a library
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());  // 192.168.4.1

    // UDP — bind to PILOT_PORT
    // Go: conn, _ := net.ListenUDP("udp", &net.UDPAddr{Port: 14551})
    udp.begin(PILOT_PORT);
    Serial.println("UDP listening on port " + String(PILOT_PORT));
}

// ============================================================
// loop() — runs forever after setup()
//
// Go equivalent:
//   func main() {
//       setup()
//       for {
//           processFrame()
//           receiveTelemetry()
//           time.Sleep(1 * time.Millisecond)
//       }
//   }
// ============================================================
void loop() {
    processFrame();
    receiveTelemetry();
    delay(1);  // yield to WiFi stack (runs in RTOS background task)
}
