# Architecture

How the router works internally, the project layout, and its known
limitations. For "what is this and how do I flash it", see the top-level
[README](../README.md).

## Supported boards (auto-detected at boot)

Probes the I2C bus (GPIO18/19) for the touch controller / IMU and picks pins:

| | ESP32-C6-LCD-1.47 | ESP32-C6-Touch-LCD-1.47 |
|---|---|---|
| Touch | none | AXS5106L |
| LCD panel | ST7789 | JD9853 (ST7789-compatible init) |
| LCD pins MOSI/SCLK/CS/DC/RST/BL | 6 / 7 / 14 / 15 / 21 / 22 | 2 / 1 / 14 / 15 / 22 / 23 |
| BOOT button | GPIO9 | GPIO8 |
| Flash | 4 MB | 8 MB |

Only the non-touch board has actually been run. The Touch branch should work —
pin profile and detection are in `board.cpp` — but the JD9853 panel is only
*assumed* ST7789-compatible (see the comment on `Arduino_ST7789` in
`display.cpp`); nobody has confirmed it actually renders correctly. If it
doesn't, switch to LovyanGFX's `Panel_JD9853` for that board. Touch input and
the IMU are never read either way — navigation is BOOT-button-only regardless
of which board you're on.

## Project layout

```
src/
  main.cpp                    app: BLE-central connection management + screen
                               state machine. Doesn't render or know board pins.
  config.h                    every tunable setting — start here
  board.h / board.cpp         the two board pin tables + auto-detect
  display.h / display.cpp     all screens; owns the GFX objects
  blemidi.h / blemidi.cpp     BLE-MIDI GATT UUIDs + a packet walker shared by
                               the transform modules
  transforms/
    transform_chain.h/.cpp    single entry point main.cpp calls; doesn't know
                               which transforms exist
    transpose.h/.cpp          general-purpose note transpose
    cc_to_pitchbend.h/.cpp    personal example — see below
```

Adding your own transform: write a module with the `transpose.*` /
`cc_to_pitchbend.*` pattern (an `apply(uint8_t *packet, uint16_t len)` that
uses `blemidi::forEachMessage()` to visit each message) and call it from
`transform_chain.cpp`. It never needs to know about BLE, the connection state,
or the other transforms.

## How it works

- **Topology:** the board is a BLE **central to both** devices, so Source and
  Target must each be a BLE-MIDI **peripheral** (keyboards, CME WIDI, most
  synths). A host that is itself a central (phone / PC / DAW) will not appear —
  that needs the router to be a peripheral, which is not implemented yet.
- **Scan filter:** only devices advertising the BLE-MIDI service UUID
  `03B80E5A-EDE8-4B33-A751-6CE34EC4C700`.
- **Pairing:** the MIDI characteristic usually needs an encrypted link. The
  router bonds automatically with "Just Works" pairing (no PIN).
- **Forwarding — raw passthrough:** each Source notification payload is copied
  to the Target characteristic **verbatim** (write-without-response), one
  write per packet. No MIDI parsing in the forwarding path itself, so nothing
  can be fabricated, dropped-by-misparse, or reordered — a BLE-MIDI packet is
  self-contained, so for a 1:1 router this is both simplest and safest.
- **Pipeline:** the notify callback (BLE host task) only copies the payload
  into a fixed-size FreeRTOS queue; a separate pump task runs the transform
  chain and does the writes with ENOMEM backoff. Writing unpaced from the
  callback exhausts the mbuf pool and drops the link.
- **Reconnect:** on either side dropping, only that side is torn down (with a
  wait for the client object to actually free — `deleteClient()` on a live
  client is async) and reconnected; the healthy link is left alone. After
  `BLE_RECONNECT_FAIL_STREAK_FOR_RESTART` consecutive failures the whole BLE
  stack restarts, which clears any leaked connection slots and lets a peer
  that still thinks it's connected time out and drop the stale link.
- **Latency:** a fast connection interval (7.5–15 ms) is requested on both
  links — two default-interval hops would otherwise stack to 60–100 ms of
  round-trip. Keep `DEBUG_LOG_TRANSFORMS` off — per-packet serial logging on
  the hot path adds perceptible latency; it's for diagnosing a transform.
- **Connect order:** Target first (idle radio), then Source; subscribing to
  the Source starts the notification stream.
- **Persistence:** the chosen pair (and transpose) is stored in NVS and
  reconnected on boot.

### MIDI transforms (`config.h`, `src/transforms/`)

Both run as an in-place rewrite in the pump task, just before the write —
`len` never changes, so this is layered cleanly on top of raw passthrough.

- **Transpose** (`ENABLE_TRANSPOSE`, on by default) — Note Off / Note On /
  Poly Aftertouch note numbers shifted by a configurable number of semitones
  (clamped 0..127). General-purpose, safe with any controller/synth pair.
  Short BOOT press on the ROUTING screen cycles through `TRANSPOSE_STEPS`
  (default −12 / −7 / 0 / +7 / +12, ascending, wraps), persisted to NVS, and
  sends an All Notes Off + All Sound Off burst so a note held across the
  switch can't hang.
- **CC → Pitch Bend** (`ENABLE_CC_TO_PITCHBEND`, **off by default**) —
  converts one Control Change controller to Pitch Bend on the same channel,
  with a dead zone around its centre and a configurable depth. This is a
  **personal mapping** written for the Artinoise Re.corder BLE flute's
  "rotation" CC (centred on 64) — the CC number and curve are specific to
  that controller. Treat `cc_to_pitchbend.*` as a template for your own
  controller, not something to enable blind.

## Controls (BOOT button)

| Screen | Short press | Long press (>0.6 s) |
|---|---|---|
| Pick SOURCE / TARGET | move cursor | select highlighted device |
| CONNECTING | — | cancel and rescan (reacts within one connect timeout, not instantly — see below) |
| ROUTING | cycle transpose (if enabled), else reset counters | forget pair + rescan |
| ERROR | retry connect | forget pair + rescan |

**At boot**, if a pair is stored, the screen shows "Hold BOOT now to forget
saved pair" for ~1.5 s; pressing BOOT during that window forgets it and goes
to Pick SOURCE instead of reconnecting.

**Do not hold BOOT in while power-cycling or resetting the board.** On
ESP32-C6 (and C3), the BOOT-button pin doubles as the chip's UART-download
strapping pin — the ROM samples it at reset and, if it's low, boots straight
into the flashing bootloader instead of running this firmware at all (black
screen, no serial output, `setup()` never runs). This is exactly why a
"hold BOOT at power-on" gesture *can't* be implemented by reading the pin
early in `setup()` — if that condition had actually been true, the app
wouldn't be executing to check it. The 1.5 s window above only works because
it happens once the app is already running, well past the point the ROM
makes that decision.

## Screens

- **Pick SOURCE / TARGET** — live list of BLE-MIDI devices (name, address, RSSI),
  highlighted cursor row.
- **ROUTING** — Source ▸ Target, per-side connection state, `rx` (packets in) /
  `fwd` (packets out) / `drop` counters, transform stats, and the last MIDI
  message in hex. Header turns red and says `LINK LOST` if either side drops
  (auto-reconnects).

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
  use the "hold BOOT now to forget" boot-time window to rescan and re-pick.
  TODO: store the name and reconnect by name-match.
- One MIDI stream, one direction. No merge, filtering, or channel remap beyond
  the two transforms in `src/transforms/` — add more following the same
  pattern.
- No active-note tracking / panic against a dropped notification. If the
  Source *link itself* loses a Note Off to RF (not a transpose change, which
  is already covered), that note can hang until the next Note On for it.
