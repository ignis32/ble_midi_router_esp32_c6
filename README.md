# BLE MIDI Router — Waveshare ESP32-C6 1.47" LCD

Scans for BLE-MIDI devices, lets you pick a **Source** and a **Target** with the
BOOT button, then forwards MIDI messages Source → Target.

## Supported boards (auto-detected at boot)

Probes the I2C bus (GPIO18/19) for the touch controller / IMU and picks pins:

| | ESP32-C6-LCD-1.47 | ESP32-C6-Touch-LCD-1.47 |
|---|---|---|
| Touch | none | AXS5106L |
| LCD panel | ST7789 | JD9853 (ST7789-compatible init) |
| LCD pins MOSI/SCLK/CS/DC/RST/BL | 6 / 7 / 14 / 15 / 21 / 22 | 2 / 1 / 14 / 15 / 22 / 23 |
| BOOT button | GPIO9 | GPIO8 |
| Flash | 4 MB | 8 MB |

## How it works

- **Topology:** the board is a BLE **central to both** devices, so Source and
  Target must each be a BLE-MIDI **peripheral** (keyboards, CME WIDI, most
  synths). A host that is itself a central (phone / PC / DAW) will not appear —
  that needs the router to be a peripheral, which is not implemented yet.
- **Scan filter:** only devices advertising the BLE-MIDI service UUID
  `03B80E5A-EDE8-4B33-A751-6CE34EC4C700`.
- **Pairing:** the MIDI characteristic usually needs an encrypted link. The
  router bonds automatically with "Just Works" pairing (no PIN).
- **Forwarding — raw passthrough (`RAW_PASSTHROUGH 1`):** each Source
  notification payload is copied to the Target characteristic **verbatim**
  (write-without-response), one write per packet. No MIDI parsing, so nothing
  can be fabricated, dropped-by-misparse, or reordered. A BLE-MIDI packet is
  self-contained, so for a 1:1 router this is both simplest and safest.
  `blemidi.h` (parser + re-encoder) is retained, unused, for a future
  filter / merge / channel-remap mode.
- **CC -> Pitch Bend (`src/transform.h`, `CC2PB_ENABLE 1`):** Control Change
  `#CC2PB_CC` (default 52 — rotation on an Artinoise Re.corder) is rewritten
  **in place** to a Pitch Bend on the same channel before the write. CC and
  Pitch Bend are both 3-byte messages, so the packet is otherwise untouched.
  Curve: dead zone of `CC2PB_DEADZONE` either side of 64 -> bend centre; past
  it the bend ramps from centre (no jump). Depth is `CC2PB_RANGE_PCT` % of the
  full 14-bit range at the CC extremes (audible width = that x the synth's own
  bend range). Resolution is 7-bit (~62 steps/side) — slow sweeps can step;
  slew/interpolation is a TODO.
- **Transpose (`src/transform.h`):** Note Off / Note On / Poly Aftertouch note
  numbers are shifted by `g_transpose` semitones (clamped 0..127) in the same
  in-place pass. Short BOOT press on the ROUTING screen cycles 0 / +12 / -12,
  persisted to NVS.
- **Pipeline:** the notify callback (BLE host task) only copies the payload
  into a fixed-size FreeRTOS queue; a separate pump task does the writes with
  ENOMEM backoff. Writing unpaced from the callback exhausts the mbuf pool and
  drops the link.
- **Latency:** `setConnectionParams(6, 12, 0, 400)` before connecting asks both
  links for a 7.5-15 ms interval. Keep `LOG_RAW 0` — per-message serial logging
  on the hot path adds perceptible latency.
- **Connect order:** Target first (idle radio), then Source; subscribing to the
  Source starts the notification stream. Each connect retries 3x; the ERROR
  screen auto-retries every 4 s.
- **Persistence:** the chosen pair is stored in NVS and reconnected on boot.

## Controls (BOOT button)

| Screen | Short press | Long press (>0.6 s) |
|---|---|---|
| Pick SOURCE / TARGET | move cursor | select highlighted device |
| ROUTING | cycle transpose 0 / +12 / -12 semitones (saved to NVS) | forget pair + rescan |
| ERROR | retry connect | forget pair + rescan |

Hold BOOT **while powering on** to skip the stored pair and rescan.

## Screens

- **Pick SOURCE / TARGET** — live list of BLE-MIDI devices (name, address, RSSI),
  highlighted cursor row.
- **ROUTING** — Source ▸ Target, per-side connection state, `rx` (packets in) /
  `fwd` (packets out) / `drop` counters, and the last MIDI message in hex.
  Header turns red and says `LINK LOST` if either side drops (auto-reconnects).

## Toolchain

- pioarduino platform pinned to **`53.03.13-1`** (Arduino 3.1.3 / IDF 5.3) — the
  54.x/55.x line corrupts PlatformIO's `penv` on Windows.
- Libraries: `Arduino_GFX`, `NimBLE-Arduino` 2.x.

## Build / flash / monitor

```bash
pio run -t upload
pio device monitor
```

## Known limitations / TODO

- No router-as-peripheral mode (can't forward to a phone / PC / DAW — those act
  as BLE centrals).
- Reconnect is by stored address only. A Target with a rotating random address
  (phone / laptop / privacy-enabled device) won't reconnect after it rotates —
  hold BOOT at power-on to rescan and re-pick. TODO: store the name and
  reconnect by name-match.
- One MIDI stream, one direction. No merge, filtering, or channel remap (that's
  what the retained `blemidi.h` parser is for).
- Teardown vs. in-flight notify has a small race window on disconnect.
- No active-note tracking / panic. If the Source link itself drops a Note Off
  notification (RF), that note can hang until the next Note On for it.
