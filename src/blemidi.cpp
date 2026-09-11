#include "blemidi.h"

namespace blemidi {

void forEachMessage(uint8_t *p, uint16_t len, MessageFn fn, void *ctx) {
  if (len < 2) return;
  size_t  i       = 1;   // skip the BLE-MIDI header
  uint8_t running = 0;

  while (i < len) {
    if (p[i] & 0x80) {
      ++i;                     // consume the timestamp-low byte
      if (i >= len) break;
    }
    // else: no timestamp byte here -> running-status continuation

    uint8_t *statusByte = nullptr;
    uint8_t  status;
    if (p[i] & 0x80) {
      if (p[i] >= 0xF8) { ++i; continue; }   // System Real-Time: 1 byte, no data
      if (p[i] >= 0xF0) break;               // SysEx / System Common: leave the rest
      statusByte = &p[i];
      status = p[i];
      running = status;
      ++i;
    } else {
      if (running == 0 || running >= 0xF0) { ++i; continue; }
      status = running;
    }

    const int need = dataBytesFor(status);
    if (need <= 0 || i + need > len) break;   // nothing to do / truncated

    if (fn) fn(status, &p[i], (uint8_t)need, statusByte, ctx);
    if (statusByte) running = *statusByte;     // fn() may have rewritten it

    i += need;
  }
}

}  // namespace blemidi
