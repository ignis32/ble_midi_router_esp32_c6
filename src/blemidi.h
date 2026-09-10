// ---------------------------------------------------------------------------
//  Minimal BLE-MIDI (Apple "MIDI over Bluetooth Low Energy") codec.
//
//  parse():  BLE-MIDI notification payload  -> list of raw MIDI messages
//  encode(): one raw MIDI message           -> BLE-MIDI packet(s) <= maxPayload
//
//  Handles: channel-voice messages, running status, System Real-Time,
//  and System Exclusive (including multi-packet, best-effort).
// ---------------------------------------------------------------------------
#pragma once
#include <Arduino.h>
#include <vector>

namespace blemidi {

static constexpr char SERVICE_UUID[] = "03b80e5a-ede8-4b33-a751-6ce34ec4c700";
static constexpr char CHAR_UUID[]    = "7772e5db-3868-4112-a1a9-f2669d106bf3";

using Msg     = std::vector<uint8_t>;
using MsgList = std::vector<Msg>;

// number of data bytes that follow a status byte
inline int dataBytesFor(uint8_t status) {
  switch (status & 0xF0) {
    case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0: return 2;
    case 0xC0: case 0xD0: return 1;
    case 0xF0:
      switch (status) {
        case 0xF1: case 0xF3: return 1;
        case 0xF2: return 2;
        default:   return 0;   // F6, F7, F8..FF
      }
  }
  return 0;
}

// Stateful parser — keep one instance per source stream (running status and
// SysEx assembly persist across notifications).
struct Parser {
  uint8_t runningStatus = 0;
  bool    inSysex = false;
  Msg     sysex;

  void reset() { runningStatus = 0; inSysex = false; sysex.clear(); }

  void parse(const uint8_t* p, size_t len, MsgList& out) {
    if (len == 0) return;
    size_t i = (p[0] & 0x80) ? 1 : 0;   // skip header byte if present

    while (i < len) {
      uint8_t b = p[i];

      // ---- inside a System Exclusive dump ----
      if (inSysex) {
        if (b & 0x80) {
          if (b >= 0xF8) { out.push_back({b}); ++i; continue; }   // realtime
          if (b == 0xF7) {                                        // end
            sysex.push_back(0xF7);
            out.push_back(sysex);
            sysex.clear();
            inSysex = false;
            ++i;
            continue;
          }
          ++i;   // timestamp byte (e.g. right before the closing 0xF7)
          continue;
        }
        sysex.push_back(b);
        ++i;
        continue;
      }

      // ---- status / timestamp byte ----
      if (b & 0x80) {
        if (b >= 0xF8) { out.push_back({b}); ++i; continue; }     // realtime

        // timestamp-low byte: the next byte is a status (or running-status data)
        ++i;
        if (i >= len) break;
        uint8_t s = p[i];

        if (s & 0x80) {
          if (s >= 0xF8) { out.push_back({s}); ++i; continue; }   // ts + realtime
          if (s == 0xF0) { inSysex = true; sysex.assign(1, 0xF0); ++i; continue; }
          runningStatus = s;
          ++i;
        } else {
          s = runningStatus;                 // running status: data right after ts
          if (s == 0) { ++i; continue; }
        }
        emit(s, p, len, i, out);
        continue;
      }

      // ---- bare data byte: running status, timestamp omitted ----
      if (runningStatus) {
        emit(runningStatus, p, len, i, out);
      } else {
        ++i;
      }
    }
  }

 private:
  static void emit(uint8_t status, const uint8_t* p, size_t len, size_t& i, MsgList& out) {
    const int need = dataBytesFor(status);
    Msg m;
    m.reserve(need + 1);
    m.push_back(status);
    while ((int)m.size() <= need && i < len && (p[i] & 0x80) == 0) {
      m.push_back(p[i]);
      ++i;
    }
    if ((int)m.size() == need + 1) out.push_back(std::move(m));
  }
};

// Encode one raw MIDI message into BLE-MIDI packets, each <= maxPayload bytes.
// Channel-voice / realtime -> a single packet. SysEx -> chunked if needed.
inline std::vector<Msg> encode(const Msg& midi, uint16_t maxPayload) {
  std::vector<Msg> packets;
  if (midi.empty()) return packets;
  if (maxPayload < 5) maxPayload = 20;

  const uint16_t ts   = millis() & 0x1FFF;
  const uint8_t  hdr  = 0x80 | ((ts >> 7) & 0x3F);
  const uint8_t  tsLo = 0x80 | (ts & 0x7F);

  const bool isSysex = midi[0] == 0xF0;
  if (!isSysex) {
    Msg pkt{hdr, tsLo};
    pkt.insert(pkt.end(), midi.begin(), midi.end());
    packets.push_back(std::move(pkt));
    return packets;
  }

  // SysEx: first packet  = hdr, tsLo, F0, data...
  //        middle packets = hdr, data...
  //        last packet    = hdr, ... , tsLo, F7
  size_t pos = 0;
  bool first = true;
  while (pos < midi.size()) {
    Msg pkt{hdr};
    if (first) { pkt.push_back(tsLo); first = false; }
    while (pos < midi.size() && pkt.size() < maxPayload) {
      if (midi[pos] == 0xF7) {
        if (pkt.size() + 2 > maxPayload) break;   // need room for tsLo + F7
        pkt.push_back(tsLo);
        pkt.push_back(0xF7);
        pos++;
        break;
      }
      pkt.push_back(midi[pos++]);
    }
    packets.push_back(std::move(pkt));
  }
  return packets;
}

// Pack as many messages as fit into BLE-MIDI packets of <= maxPayload bytes.
// Channel-voice / realtime messages share a packet; SysEx is encoded on its own.
inline std::vector<Msg> encodeBatch(const MsgList& msgs, uint16_t maxPayload) {
  std::vector<Msg> packets;
  if (maxPayload < 5) maxPayload = 20;

  const uint16_t ts   = millis() & 0x1FFF;
  const uint8_t  hdr  = 0x80 | ((ts >> 7) & 0x3F);
  const uint8_t  tsLo = 0x80 | (ts & 0x7F);

  Msg cur;
  for (const auto& m : msgs) {
    if (m.empty()) continue;
    if (m[0] == 0xF0) {                       // SysEx: flush, then standalone
      if (cur.size() > 1) { packets.push_back(cur); }
      cur.clear();
      for (auto& p : encode(m, maxPayload)) packets.push_back(std::move(p));
      continue;
    }
    const size_t add = 1 + m.size();          // tsLo + message bytes
    if (cur.empty()) cur.push_back(hdr);
    if (cur.size() + add > maxPayload) {
      packets.push_back(cur);
      cur.clear();
      cur.push_back(hdr);
    }
    cur.push_back(tsLo);
    cur.insert(cur.end(), m.begin(), m.end());
  }
  if (cur.size() > 1) packets.push_back(std::move(cur));
  return packets;
}

}  // namespace blemidi
