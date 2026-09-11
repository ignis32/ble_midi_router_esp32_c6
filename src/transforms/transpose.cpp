#include "transpose.h"

#include "../blemidi.h"
#include "../config.h"

namespace transpose {
namespace {

int8_t s_semitones = 0;

uint8_t clamp7(int v) { return v < 0 ? 0 : (v > 127 ? 127 : (uint8_t)v); }

void visit(uint8_t status, uint8_t *data, uint8_t dataLen, uint8_t * /*statusByte*/,
           void * /*ctx*/) {
  const uint8_t hi = status & 0xF0;
  if ((hi == 0x80 || hi == 0x90 || hi == 0xA0) && dataLen >= 1) {
    data[0] = clamp7((int)data[0] + s_semitones);
  }
}

}  // namespace

void apply(uint8_t *packet, uint16_t len) {
#if ENABLE_TRANSPOSE
  if (s_semitones == 0) return;
  blemidi::forEachMessage(packet, len, visit);
#else
  (void)packet;
  (void)len;
#endif
}

void set(int8_t semitones) { s_semitones = semitones; }
int8_t get() { return s_semitones; }

int8_t cycle() {
  static const int8_t steps[] = TRANSPOSE_STEPS;
  constexpr size_t n = sizeof(steps) / sizeof(steps[0]);
  size_t idx = 0;
  for (size_t k = 0; k < n; ++k) {
    if (steps[k] == s_semitones) { idx = k; break; }
  }
  s_semitones = steps[(idx + 1) % n];
  return s_semitones;
}

}  // namespace transpose
