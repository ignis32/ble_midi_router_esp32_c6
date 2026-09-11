#include "cc_to_pitchbend.h"

#include "../blemidi.h"
#include "../config.h"

namespace cc_to_pitchbend {
namespace {

// CC 0..127 (centre 64) -> 14-bit pitch bend 0..16383 (centre 8192). A dead
// zone around 64 maps to centre; leaving it, the bend ramps from centre (no
// jump). CC2PB_RANGE_PCT caps how far it reaches at the CC extremes.
uint16_t ccToPitchBend(uint8_t cc) {
  constexpr int   centre = 64;
  constexpr int   dz     = CC2PB_DEADZONE;
  constexpr float scale  = CC2PB_RANGE_PCT / 100.0f;
  const int d = (int)cc - centre;

  if (d >= -dz && d <= dz) return 8192;

  if (d > dz) {
    float f = float(d - dz) / float(127 - centre - dz);   // 0..1
    if (f > 1.0f) f = 1.0f;
    return (uint16_t)(8192.0f + f * scale * 8191.0f + 0.5f);
  }
  float f = float(-d - dz) / float(centre - dz);           // 0..1
  if (f > 1.0f) f = 1.0f;
  const int v = (int)(8192.0f - f * scale * 8192.0f - 0.5f);
  return (uint16_t)(v < 0 ? 0 : v);
}

void visit(uint8_t status, uint8_t *data, uint8_t dataLen, uint8_t *statusByte, void *ctx) {
  int *hits = static_cast<int *>(ctx);
  if (!statusByte) return;                          // rewrites the status byte -> explicit only
  if ((status & 0xF0) != 0xB0 || dataLen != 2) return;
  if (data[0] != CC2PB_CONTROLLER) return;

  const uint16_t pb = ccToPitchBend(data[1]);
  *statusByte = 0xE0 | (status & 0x0F);   // Pitch Bend, same channel
  data[0] = pb & 0x7F;                    // LSB
  data[1] = (pb >> 7) & 0x7F;             // MSB
  ++*hits;
}

}  // namespace

int apply(uint8_t *packet, uint16_t len) {
#if ENABLE_CC_TO_PITCHBEND
  int hits = 0;
  blemidi::forEachMessage(packet, len, visit, &hits);
  return hits;
#else
  (void)packet;
  (void)len;
  return 0;
#endif
}

}  // namespace cc_to_pitchbend
