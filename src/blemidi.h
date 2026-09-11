#pragma once
#include <Arduino.h>

// Minimal BLE-MIDI ("MIDI over Bluetooth Low Energy") protocol support: the
// GATT UUIDs, and a packet walker shared by the transform modules.
namespace blemidi {

inline constexpr char SERVICE_UUID[] = "03b80e5a-ede8-4b33-a751-6ce34ec4c700";
inline constexpr char CHAR_UUID[]    = "7772e5db-3868-4112-a1a9-f2669d106bf3";

// Number of data bytes that follow a channel-voice/mode status byte.
inline int dataBytesFor(uint8_t status) {
  switch (status & 0xF0) {
    case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0: return 2;
    case 0xC0: case 0xD0: return 1;
    case 0xF0:
      switch (status) {
        case 0xF1: case 0xF3: return 1;
        case 0xF2: return 2;
        default:   return 0;   // F0 (SysEx), F4-F7, F8-FF (System Real-Time)
      }
  }
  return 0;
}

// Called once per channel-voice/mode message found while walking a packet.
//   status      the message's status byte (0x80..0xEF)
//   data        its data bytes (1 or 2), mutable in place
//   dataLen     number of data bytes
//   statusByte  pointer to the status byte in the packet if this message
//               carries one explicitly, else nullptr (running status: only
//               `data` can be rewritten in place, not the message type,
//               without shifting every following byte)
//   ctx         opaque pointer passed through from forEachMessage()
using MessageFn = void (*)(uint8_t status, uint8_t *data, uint8_t dataLen,
                            uint8_t *statusByte, void *ctx);

// Walk a BLE-MIDI packet (header, then [timestamp, message]*), calling `fn`
// for each channel-voice/mode message found. Running status is tracked and
// reset at the start of every packet, per the BLE-MIDI spec.
//
// A byte in the *timestamp* position is always a timestamp, even if it's
// 0x80..0xFF (that's what a timestamp-low byte is: 0x80 | (ms & 0x7F), so it
// lands in the System Real-Time range 0xF8..0xFF about 6% of the time).
// Real-Time status is only recognised in the *status* position.
//
// SysEx / System Common ends the walk; anything from there to the end of the
// packet is left untouched.
void forEachMessage(uint8_t *packet, uint16_t len, MessageFn fn, void *ctx = nullptr);

}  // namespace blemidi
