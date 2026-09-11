# BLE MIDI Router

Firmware for a Waveshare ESP32-C6 1.47" LCD board that bridges two BLE-MIDI
devices: pick a **Source** and a **Target** on the little screen, and it
forwards MIDI between them over Bluetooth — no phone or laptop needed in
between.

Built to connect an [Artinoise Re.corder](https://artinoise.com/) BLE flute
straight into a synth.

## Features

- Scans for BLE-MIDI peripherals, pick Source & Target with one button
- Low-latency, byte-for-byte forwarding — no MIDI re-parsing in the hot path
- Auto-reconnects and remembers your pair across reboots
- Optional built-in transforms: note transpose, CC → Pitch Bend

## Hardware

- [Waveshare ESP32-C6-LCD-1.47](https://www.waveshare.com/esp32-c6-lcd-1.47.htm)
  (the Touch variant works too — auto-detected)

## Build & flash

```bash
pio run -t upload
pio device monitor
```

## Controls

One button (BOOT) does everything: **short press** moves the cursor / cycles
a setting, **long press** selects / resets. Hold it while powering on to
forget the saved pair and scan again.

## More

Internals, project layout, and known limitations are in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).
