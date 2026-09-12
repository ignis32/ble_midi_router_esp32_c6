# BLE MIDI Router

Firmware for a Waveshare ESP32-C6 1.47" LCD board that bridges two BLE-MIDI
peripheral devices: pick a **Source** and a **Target** on the little screen, and
it forwards MIDI between them over Bluetooth — no phone or laptop needed in
between.

I made it for myself, to connect an [Artinoise Re.corder](https://artinoise.com/)
BLE flute straight into an M-Vave FM1 synth.

For the Re.corder specifically, it also maps CC 52 into Pitch Bend with a dead
zone (`ENABLE_CC_TO_PITCHBEND 1` in `config.h`).

Latency is surprisingly okay.

## Features

- Scans for BLE-MIDI peripherals, pick Source & Target with one button
- Low-latency, byte-for-byte forwarding — no MIDI re-parsing in the hot path
- Auto-reconnects and remembers your pair across reboots
- Optional built-in transforms: note transpose, CC → Pitch Bend

## Hardware

- [Waveshare ESP32-C6-LCD-1.47](https://www.waveshare.com/esp32-c6-lcd-1.47.htm)
  (auto-detects the Touch variant too and should work — but only the
  non-touch board has actually been tested)

## Build & flash

```bash
pio run -t upload
pio device monitor
```

## Controls

One button (BOOT) does everything: **short press** moves the cursor / cycles
a setting, **long press** selects / resets. If a pair is already saved, press
BOOT within about 1.5s of power-on (once the screen is showing something) to
forget it and scan again — **don't** hold it in while actually powering on or
resetting, that's the chip's own bootloader-select pin and it'll boot into the
flashing bootloader instead of the app.

## More

Internals, project layout, and known limitations are in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).
