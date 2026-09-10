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

// Walk a BLE-MIDI packet as a stream of MIDI messages, rewriting in place:
//   - Note Off / Note On / Poly Aftertouch -> note number += `transpose`
//     semitones (clamped 0..127); handles running-status messages too
//   - explicit "Bn <CC2PB_CC> vv"          -> Pitch Bend on the same channel
// `len` never changes. Running status is reset per packet (BLE-MIDI spec).
// Returns the number of CC->PitchBend rewrites.
inline int apply(uint8_t *p, uint16_t len, uint8_t rangePct, int8_t transpose) {
#if !CC2PB_ENABLE
  (void)p; (void)len; (void)rangePct; (void)transpose;
  return 0;
#else
  if (len < 2) return 0;
  int     hits    = 0;
  size_t  i       = 1;     // skip BLE-MIDI header
  uint8_t running = 0;

  while (i < len) {
    // BLE-MIDI layout: header, then [timestamp-low, message]+. The byte in the
    // timestamp position is 0x80..0xFF — a value >= 0xF8 there is STILL a
    // timestamp, not a System Real-Time status.
    if (p[i] & 0x80) {
      ++i;                                    // consume timestamp-low
      if (i >= len) break;
    }
    // else: no timestamp -> running-status continuation

    // p[i] is now a status byte, or running-status data
    bool    explicitStatus;
    uint8_t status;
    if (p[i] & 0x80) {
      status = p[i];
      if (status >= 0xF8) { ++i; continue; }  // System Real-Time: 1 byte, no data
      if (status >= 0xF0) break;              // SysEx / System Common: leave the rest
      running = status;
      explicitStatus = true;
      ++i;
    } else {
      if (running == 0 || running >= 0xF0) { ++i; continue; }
      status = running;                       // running status: reuse prior status
      explicitStatus = false;
    }

    const int need = blemidi::dataBytesFor(status);   // data bytes for this msg
    if (need <= 0 || i + need > len) break;           // truncated
    const uint8_t hi = status & 0xF0;

    // CC#CC2PB_CC -> Pitch Bend (rewrites the status byte, so explicit only)
    if (explicitStatus && hi == 0xB0 && need == 2 &&
        p[i] == CC2PB_CC && !(p[i + 1] & 0x80)) {
      const uint16_t pb = ccToPitchBend(p[i + 1], rangePct);
      p[i - 1] = 0xE0 | (status & 0x0F);      // the status byte we just passed
      p[i]     = pb & 0x7F;
      p[i + 1] = (pb >> 7) & 0x7F;
      running  = p[i - 1];                    // keep running status coherent
      ++hits;
    }
    // transpose Note Off / Note On / Poly Key Pressure (data[0] = note number)
    else if (transpose != 0 && (hi == 0x80 || hi == 0x90 || hi == 0xA0) &&
             !(p[i] & 0x80)) {
      p[i] = clamp7((int)p[i] + transpose);
    }

    i += need;
  }
  return hits;
#endif
}

}  // namespace xform
