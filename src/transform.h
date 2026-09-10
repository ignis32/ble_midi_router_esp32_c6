// ---------------------------------------------------------------------------
//  In-place BLE-MIDI packet transforms, layered on top of raw passthrough.
//
//  CC -> Pitch Bend: Control Change #CC2PB_CC (e.g. rotation from an Artinoise
//  Re.corder, centred on 64) becomes a Pitch Bend on the same channel.
//
//  Both messages are 3 bytes, so this is a byte-for-byte overwrite inside the
//  existing packet — header, timestamps and every other message are untouched.
//  Running status is reset per packet; anything ambiguous is left as-is.
// ---------------------------------------------------------------------------
#pragma once
#include <Arduino.h>
#include "blemidi.h"   // blemidi::dataBytesFor

// -------------------------------- config --------------------------------
#ifndef CC2PB_ENABLE
#define CC2PB_ENABLE 1
#endif
#ifndef CC2PB_CC
#define CC2PB_CC 52            // Control Change number to convert
#endif
#ifndef CC2PB_DEADZONE
#define CC2PB_DEADZONE 6       // CC units either side of 64 that map to centre
#endif
#ifndef CC2PB_RANGE_PCT
#define CC2PB_RANGE_PCT 10    // % of the full 14-bit bend used at the CC extremes
#endif

namespace xform {

inline uint8_t clamp7(int v) { return v < 0 ? 0 : (v > 127 ? 127 : (uint8_t)v); }

// CC 0..127 (centre 64) -> 14-bit pitch bend 0..16383 (centre 8192).
// Dead zone around 64; leaving it, the bend ramps from centre (no step).
// rangePct (1..100) caps the bend depth at the CC extremes.
inline uint16_t ccToPitchBend(uint8_t cc, uint8_t rangePct) {
  const int   centre = 64;
  const int   dz     = CC2PB_DEADZONE;
  const float scale  = rangePct / 100.0f;
  const int   d      = (int)cc - centre;

  if (d >= -dz && d <= dz) return 8192;

  if (d > dz) {
    float f = float(d - dz) / float(127 - centre - dz);   // 0..1
    if (f > 1.0f) f = 1.0f;
    return (uint16_t)(8192.0f + f * scale * 8191.0f + 0.5f);
  }
  float f = float(-d - dz) / float(centre - dz);          // 0..1
  if (f > 1.0f) f = 1.0f;
  int v = (int)(8192.0f - f * scale * 8192.0f - 0.5f);
  return (uint16_t)(v < 0 ? 0 : v);
}

// Walk a BLE-MIDI packet in place:
//   - each explicit "Bn <CC2PB_CC> vv"     -> Pitch Bend on the same channel
//   - Note Off / Note On / Poly Aftertouch -> note number += `transpose`
//     semitones (clamped to 0..127); no-op when transpose == 0
// `len` never changes. Returns the number of CC->PitchBend rewrites.
inline int apply(uint8_t *p, uint16_t len, uint8_t rangePct, int8_t transpose) {
#if !CC2PB_ENABLE
  (void)p; (void)len; (void)rangePct; (void)transpose;
  return 0;
#else
  if (len < 4) return 0;
  int     hits    = 0;
  size_t  i       = 1;     // skip BLE-MIDI header
  uint8_t running = 0;     // reset every packet (per BLE-MIDI spec)

  while (i < len) {
    const uint8_t b = p[i];
    if (!(b & 0x80)) { ++i; continue; }   // stray data byte
    if (b >= 0xF8)   { ++i; continue; }   // System Real-Time, standalone

    // b is a timestamp-low byte; the next byte is status or running-status data
    ++i;
    if (i >= len) break;
    const uint8_t s = p[i];

    if (!(s & 0x80)) {                    // running-status data — leave untouched
      int n = running ? blemidi::dataBytesFor(running) : 1;
      i += n > 0 ? n : 1;
      continue;
    }
    if (s >= 0xF0) break;                 // SysEx / System Common — leave the rest as-is
    running = s;

    if ((s & 0xF0) == 0xB0 && i + 2 < len &&
        p[i + 1] == CC2PB_CC && !(p[i + 2] & 0x80)) {
      const uint16_t pb = ccToPitchBend(p[i + 2], rangePct);
      p[i]     = 0xE0 | (s & 0x0F);       // Pitch Bend, same channel
      p[i + 1] = pb & 0x7F;              // LSB (7 bits)
      p[i + 2] = (pb >> 7) & 0x7F;       // MSB (7 bits)
      ++hits;
      i += 3;
      continue;
    }

    // transpose Note Off / Note On / Poly Key Pressure (byte 1 = note number)
    if (transpose != 0) {
      const uint8_t hi = s & 0xF0;
      if ((hi == 0x80 || hi == 0x90 || hi == 0xA0) &&
          i + 1 < len && !(p[i + 1] & 0x80)) {
        p[i + 1] = clamp7((int)p[i + 1] + transpose);
      }
    }
    i += 1 + blemidi::dataBytesFor(s);   // advance past this message
  }
  return hits;
#endif
}

}  // namespace xform
