#pragma once
#include <Arduino.h>

// Shifts Note Off / Note On / Poly Key Pressure note numbers by a number of
// semitones, in place. General-purpose -- safe for any BLE-MIDI controller /
// synth pair. Enable and configure the BOOT-button step list in config.h
// (ENABLE_TRANSPOSE, TRANSPOSE_STEPS).
namespace transpose {

// Rewrite note numbers in `packet` by the current transpose. No-op at 0
// semitones or when ENABLE_TRANSPOSE is 0.
void apply(uint8_t *packet, uint16_t len);

void   set(int8_t semitones);
int8_t get();

// Advance to the next value in TRANSPOSE_STEPS (wrapping) and return it.
int8_t cycle();

}  // namespace transpose
