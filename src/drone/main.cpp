#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_timer.h>
#include "crsf.h"

// ============================================================
// CONFIGURATION
// ============================================================

#define WIFI_SSID       "ELRS_RELAY"
#define WIFI_PASS       "12345678"
#define PILOT_IP        "192.168.4.1"
#define PILOT_PORT      14551
#define DRONE_PORT      14550

#define UART_TX_PIN     17   // GPIO17 -> Bandit Micro pin 3 (TX = data IN)
#define UART_RX_PIN     16   // GPIO16 -> Bandit Micro pin 4 (RX = telem OUT)
#define UART_BAUD       420000

#define PACKET_RATE_HZ  200          // 200 Hz = 5ms period
#define FAILSAFE_TIMEOUT_MS  500    // 500ms without UDP -> failsafe

// ============================================================
// GLOBAL STATE
// ============================================================

WiFiUDP udp;

// Ring buffer (8 slots) — smooths network jitter
// Go: chan []byte with buffer 8, or ring buffer slice
// C++: fixed 2D array + head/tail indices
#define BUF_SIZE 8
static uint8_t frameBuf[BUF_SIZE][CRSF_RC_FRAME_SIZE];
static volatile int bufHead = 0;  // producer writes here
static volatile int bufTail = 0;  // consumer reads here

// Last good frame (retransmit when buffer empty)
static uint8_t lastFrame[CRSF_RC_FRAME_SIZE];
static bool hasLastFrame = false;

// Failsafe tracking
static uint32_t lastUdpTime = 0;
static bool inFailsafe = false;

// ============================================================
// Hardware Timer — precise 5ms ticks (200Hz)
//
// Go equivalent:
//   ticker := time.NewTicker(5 * time.Millisecond)
//   go func() {
//       for range ticker.C {
//           sendNextFrame()
//       }
//   }()
//
// In C++ on ESP32:
//   esp_timer_create() + esp_timer_start_periodic() = hardware
//   timer that calls a callback every N microseconds.
//   More precise than time.Ticker (which has OS scheduling jitter).
//
// The callback runs in the ESP-IDF timer task — a high-priority
// RTOS task. It's like a goroutine that gets preempted in
// immediately when the timer fires.
// ============================================================

static esp_timer_handle_t txTimer;

// ============================================================
// TX Timer Callback — fires every 5ms (200Hz)
//
// IRAM_ATTR = put in fast RAM (required for timer callbacks)
//
// This is the heart of the drone side:
// 1. If buffer has frames -> send next one
// 2. If buffer empty but we have last frame -> repeat it
// 3. If nothing at all -> send failsafe
//
// Go equivalent:
//   func onTxTimer() {
//       select {
//       case frame := <-frameCh:
//           lastFrame = frame
//           port.Write(frame)
//       default:
//           if hasLastFrame {
//               port.Write(lastFrame)
//           } else {
//               port.Write(failsafeFrame)
//           }
//       }
//   }
// ============================================================
void IRAM_ATTR onTxTimer(void *arg) {
    uint8_t frame[CRSF_RC_FRAME_SIZE];

    if (bufHead != bufTail) {
        // Buffer has frames — send next one
        memcpy(frame, frameBuf[bufTail], CRSF_RC_FRAME_SIZE);
        bufTail = (bufTail + 1) % BUF_SIZE;

        // Save as last good frame
        memcpy(lastFrame, frame, CRSF_RC_FRAME_SIZE);
        hasLastFrame = true;
        inFailsafe = false;
    } else if (hasLastFrame) {
        // Buffer empty — repeat last good frame
        // (holds control surfaces in place during jitter)
        memcpy(frame, lastFrame, CRSF_RC_FRAME_SIZE);
    } else {
        // Nothing received yet — send failsafe
        crsf_build_failsafe_frame(frame);
        inFailsafe = true;
    }

    // Send to JR module via UART
    // Go: port.Write(frame)
    Serial2.write(frame, CRSF_RC_FRAME_SIZE);
}

// ============================================================
// receiveUdp — called from loop()
//
// Go:
//   n, _, _ := conn.ReadFromUDP(buf)
//   var pkt UdpPacket
//   binary.Read(bytes.NewReader(buf[:n]), binary.LittleEndian, &pkt)
// ============================================================
void receiveUdp() {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) return;

    uint8_t buf[UDP_PAYLOAD_SIZE + 16];  // extra room
    if (packetSize > (int)sizeof(buf)) return;

    int len = udp.read(buf, packetSize);
    if (len < UDP_PAYLOAD_SIZE) return;

    // Deserialize UDP packet
    UdpPacket pkt;
    if (!udp_packet_deserialize(buf, len, &pkt)) return;

    // Verify CRSF sync byte + frame type
    if (pkt.crsf[0] != CRSF_SYNC_BYTE_TX || pkt.crsf[2] != CRSF_FRAME_TYPE_RC) return;

    // Verify CRC
    uint8_t crc = crsf_crc8(&pkt.crsf[2], pkt.crsf[1] - 1);
    if (crc != pkt.crsf[pkt.crsf[1]]) return;

    // Push into ring buffer
    memcpy(frameBuf[bufHead], pkt.crsf, CRSF_RC_FRAME_SIZE);
    bufHead = (bufHead + 1) % BUF_SIZE;

    // Reset failsafe timer
    lastUdpTime = millis();
    inFailsafe = false;
}

// ============================================================
// sendTelemetry — read from JR module, forward to pilot via UDP
//
// JR module sends telemetry frames back on UART.
// We collect them and forward via UDP to the pilot side.
// ============================================================
static uint8_t telemBuf[64];
static uint16_t telemLen = 0;

void sendTelemetry() {
    while (Serial2.available()) {
        if (telemLen < sizeof(telemBuf)) {
            telemBuf[telemLen++] = Serial2.read();
        } else {
            // Check for complete frame before discarding
            if (telemBuf[0] == CRSF_SYNC_BYTE_RX) {
                uint8_t expected = telemBuf[1] + 2;
                if (telemLen >= expected) {
                    udp.beginPacket(PILOT_IP, PILOT_PORT);
                    udp.write(telemBuf, expected);
                    udp.endPacket();
                    telemLen = 0;
                    return;
                }
            }
            telemLen = 0;
        }
    }

    // Check if we have a complete frame in buffer
    if (telemLen >= 4 && telemBuf[0] == CRSF_SYNC_BYTE_RX) {
        uint8_t expected = telemBuf[1] + 2;
        if (telemLen >= expected) {
            udp.beginPacket(PILOT_IP, PILOT_PORT);
            udp.write(telemBuf, expected);
            udp.endPacket();
            telemLen = 0;
        }
    }
}

// ============================================================
// checkFailsafe — if no UDP for >500ms, force failsafe
//
// Go:
//   if time.Since(lastUdpTime) > 500*time.Millisecond {
//       inFailsafe = true
//       bufHead = bufTail = 0
//       hasLastFrame = false
//   }
// ============================================================
void checkFailsafe() {
    if (inFailsafe) return;  // already in failsafe

    if (millis() - lastUdpTime > FAILSAFE_TIMEOUT_MS) {
        inFailsafe = true;
        bufHead = bufTail = 0;     // clear buffer
        hasLastFrame = false;       // forget last frame
        Serial.println("FAILSAFE: no UDP for >500ms");
    }
}

// ============================================================
// setup()
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("=== ELRS Relay: DRONE ===");

    // UART to JR module
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // WiFi Station — connect to pilot's AP
    // Go: not applicable (hardware WiFi)
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");

    // Wait up to 10 seconds for connection
    uint32_t startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
        delay(200);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi FAILED - rebooting in 3s");
        delay(3000);
        ESP.restart();  // like os.Exit(1) but hardware reboot
    }

    Serial.print("\nConnected! IP: ");
    Serial.println(WiFi.localIP());

    // UDP — bind to DRONE_PORT
    udp.begin(DRONE_PORT);
    Serial.println("UDP listening on port " + String(DRONE_PORT));

    // ============================================================
    // Hardware timer — 5ms = 200Hz
    //
    // Go:
    //   ticker := time.NewTicker(5 * time.Millisecond)
    //   go func() { for range ticker.C { onTxTimer(nil) } }()
    //
    // C++:
    //   esp_timer_create_args_t = config struct for timer
    //   esp_timer_create() = allocate timer
    //   esp_timer_start_periodic() = start it, fires every N us
    // ============================================================
    esp_timer_create_args_t timerArgs = {};
    timerArgs.callback = onTxTimer;     // function pointer (like Go func)
    timerArgs.name = "tx_timer";        // for debugging

    esp_timer_create(&timerArgs, &txTimer);
    esp_timer_start_periodic(txTimer, 5000);  // 5000 microseconds = 5ms

    lastUdpTime = millis();
    Serial.println("TX timer started at 200Hz");
}

// ============================================================
// loop()
//
// Go:
//   for {
//       receiveUdp()
//       sendTelemetry()
//       checkFailsafe()
//       time.Sleep(1 * time.Millisecond)
//   }
// ============================================================
void loop() {
    receiveUdp();
    sendTelemetry();
    checkFailsafe();
    delay(1);
}
