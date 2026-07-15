#pragma once
#include <Arduino.h>

// This file must be byte-identical to the copy in the sensor-ESP repo -
// both sides pack/unpack the same struct. If you change one, change both.

// ---- UART link config ----
#define UART_BAUD      115200
#define UART_NUM       1        // keep UART0 free for USB debug

// ---- Packet format ----
#define PKT_START_BYTE 0xAA

#pragma pack(push, 1)
struct IRPacket {
    uint8_t  start;          // sync byte, always 0xAA
    uint8_t  seq;            // rolling counter, lets us spot dropped packets
    uint16_t freq1_mag;      // lock-in magnitude, channel 1
    uint16_t freq2_mag;      // lock-in magnitude, channel 2
    uint8_t  detected_mask;  // bit0 = ch1 above threshold, bit1 = ch2 above threshold
    uint8_t  checksum;       // XOR of every byte between start and checksum
};
#pragma pack(pop)

static_assert(sizeof(IRPacket) == 8, "IRPacket size drifted - update both repos together");

inline uint8_t irPacketChecksum(const IRPacket& p) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&p);
    uint8_t c = 0;
    for (size_t i = 1; i < sizeof(IRPacket) - 1; i++) c ^= bytes[i];
    return c;
}
