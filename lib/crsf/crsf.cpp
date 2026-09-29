#include "crsf.h"

// ============================================================
// CRC8 — polynomial 0xD5
//
// Go equivalent:
//   func crc8(data []byte) byte {
//       var crc byte = 0
//       for _, b := range data {
//           crc ^= b
//           for j := 0; j < 8; j++ {
//               if crc&0x80 != 0 { crc = (crc << 1) ^ 0xD5 } else { crc <<= 1 }
//           }
//       }
//       return crc
//   }
//
// In C++:
// - `uint8_t` is exactly Go's `byte` (unsigned 8-bit)
// - `for` loops look the same as Go's C-style for
// - `<<` and `^` (XOR) work identically
// - No range syntax; we use index-based loops
// ============================================================
uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================
// Pack 16 channels (11-bit each) into 22 bytes
//
// Go equivalent:
//   func packChannels(ch [16]uint16) [22]byte { ... }
//
// In C++:
// - `const uint16_t ch[16]` = read-only array of 16 uint16s
//   (Go: `ch [16]uint16` passed by value)
// - `uint8_t *out22` = output pointer (Go: return value)
//   C++ often uses output pointers instead of returning arrays
// ============================================================
void crsf_pack_channels(const uint16_t ch[16], uint8_t *out22) {
    uint32_t bits = 0;
    int bit_count = 0;
    int idx = 0;

    for (int i = 0; i < 16; i++) {
        bits |= (uint32_t)(ch[i] & 0x7FF) << bit_count;
        bit_count += 11;
        while (bit_count >= 8) {
            out22[idx++] = (uint8_t)(bits & 0xFF);
            bits >>= 8;
            bit_count -= 8;
        }
    }
    if (bit_count > 0) {
        out22[idx++] = (uint8_t)(bits & 0xFF);
    }
}

// ============================================================
// Unpack 22 bytes into 16 channels (reverse of pack)
// ============================================================
void crsf_unpack_channels(const uint8_t *data22, uint16_t out[16]) {
    uint32_t bits = 0;
    int bit_count = 0;
    int byte_idx = 0;

    for (int i = 0; i < 16; i++) {
        while (bit_count < 11) {
            bits |= (uint32_t)data22[byte_idx++] << bit_count;
            bit_count += 8;
        }
        out[i] = (uint16_t)(bits & 0x7FF);
        bits >>= 11;
        bit_count -= 11;
    }
}

// ============================================================
// Build a complete RC channels frame (26 bytes)
//
// Frame layout:
//   [0]    = 0xEE (sync)
//   [1]    = 0x18 (length = 24)
//   [2]    = 0x16 (type: RC Channels Packed)
//   [3..24] = 22 bytes of packed channels
//   [25]   = CRC8 of bytes [2..24]
// ============================================================
void crsf_build_rc_frame(const uint16_t ch[16], uint8_t *out26) {
    out26[0] = CRSF_SYNC_BYTE_TX;      // 0xEE
    out26[1] = 0x18;                   // length
    out26[2] = CRSF_FRAME_TYPE_RC;     // 0x16
    crsf_pack_channels(ch, &out26[3]); // 22 bytes
    out26[25] = crsf_crc8(&out26[2], 23); // CRC from type to end of payload
}

// ============================================================
// Failsafe: all channels center (992), throttle 0 (172)
// ============================================================
void crsf_build_failsafe_frame(uint8_t *out26) {
    uint16_t ch[16];
    for (int i = 0; i < 16; i++) {
        ch[i] = CRSF_CHANNEL_CENTER;
    }
    ch[2] = CRSF_CHANNEL_MIN;  // throttle = 0
    crsf_build_rc_frame(ch, out26);
}

// ============================================================
// UDP packet serialization
//
// Go used binary.Write / binary.Read with bytes.Buffer.
// Here we use memcpy — the C equivalent of copy(dst, src).
//
// ESP32 is little-endian by default, so uint32_t in memory
// is already in the right byte order for our protocol.
// ============================================================
void udp_packet_serialize(const UdpPacket *pkt, uint8_t *out) {
    memcpy(out, &pkt->seq, 4);          // bytes 0-3: seq
    memcpy(out + 4, &pkt->timestamp, 4); // bytes 4-7: timestamp
    memcpy(out + 8, pkt->crsf, CRSF_RC_FRAME_SIZE); // bytes 8-33: CRSF frame
}

bool udp_packet_deserialize(const uint8_t *data, uint16_t len, UdpPacket *pkt) {
    if (len < UDP_PAYLOAD_SIZE) {
        return false;
    }
    memcpy(&pkt->seq, data, 4);
    memcpy(&pkt->timestamp, data + 4, 4);
    memcpy(pkt->crsf, data + 8, CRSF_RC_FRAME_SIZE);
    return true;
}
