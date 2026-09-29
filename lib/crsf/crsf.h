#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include <string.h>

// ============================================================
// CRSF Protocol Constants
// ============================================================
// In Go these would be `const` declarations at package level.
// In C++, `#define` is a preprocessor macro — pure text replacement
// before compilation. No type, no scope, just substitution.

#define CRSF_SYNC_BYTE_TX     0xEE   // Address: TX module
#define CRSF_SYNC_BYTE_RX    0xC8   // Address: Flight Controller

#define CRSF_FRAME_TYPE_RC      0x16   // RC Channels Packed (16ch x 11bit)
#define CRSF_FRAME_TYPE_LINK    0x14   // Link Statistics
#define CRSF_FRAME_TYPE_BATTERY 0x08   // Battery Sensor
#define CRSF_FRAME_TYPE_GPS     0x02   // GPS
#define CRSF_FRAME_TYPE_ATTITUDE 0x1E  // Attitude
#define CRSF_FRAME_TYPE_FLIGHT  0x21   // Flight Mode

#define CRSF_RC_PAYLOAD_SIZE   22     // 16 channels x 11 bits = 176 bits = 22 bytes
#define CRSF_RC_FRAME_SIZE     26     // sync(1) + len(1) + type(1) + payload(22) + crc(1)

#define CRSF_CHANNEL_MIN     172      // ~988 us
#define CRSF_CHANNEL_CENTER  992      // ~1500 us
#define CRSF_CHANNEL_MAX     1811     // ~2012 us

// ============================================================
// Function declarations
//
// In Go, you declare function signatures in an interface or just
// define them in the same package. In C++, the .h file declares
// the signature, and the .cpp file provides the implementation.
// This separation is called the "header/source split."
//
// `const uint8_t *data` means "pointer to read-only bytes."
// In Go: `data []byte` (you just don't modify it).
// ============================================================

uint8_t crsf_crc8(const uint8_t *data, uint8_t len);
void crsf_pack_channels(const uint16_t ch[16], uint8_t *out22);
void crsf_unpack_channels(const uint8_t *data22, uint16_t out[16]);
void crsf_build_rc_frame(const uint16_t ch[16], uint8_t *out26);
void crsf_build_failsafe_frame(uint8_t *out26);

// ============================================================
// UDP wrapper struct
//
// Go equivalent:
//   type UdpPacket struct {
//       Seq       uint32
//       Timestamp uint32
//       Crsf      [26]byte
//   }
//
// In C++, `struct` is almost identical — just fields, no methods
// (unless you add them). `uint32_t` is like Go's `uint32`.
// ============================================================

#define UDP_PAYLOAD_SIZE  34   // 4 + 4 + 26

struct UdpPacket {
    uint32_t seq;
    uint32_t timestamp;
    uint8_t  crsf[CRSF_RC_FRAME_SIZE]; // 26 bytes
};

void udp_packet_serialize(const UdpPacket *pkt, uint8_t *out);
bool udp_packet_deserialize(const uint8_t *data, uint16_t len, UdpPacket *pkt);

#endif // CRSF_H
