#pragma once
#include <Arduino.h>

// EXAMPLE / personal transform, disabled by default (ENABLE_CC_TO_PITCHBEND
// in config.h). Converts one Control Change controller into Pitch Bend, with
// a dead zone around its centre value and a configurable depth.
//
// Written for the Artinoise Re.corder BLE flute's "rotation" CC (centred on
// 64, tilt-left/right = bend down/up) -- the CC number and curve are specific
// to that controller. Use this file as a template for mapping a different
// controller's continuous CC to Pitch Bend, or copy the pattern for an
// entirely different transform.
namespace cc_to_pitchbend {

// Rewrite matching CC messages to Pitch Bend, in place (no-op if disabled).
// Returns the number of messages rewritten (for UI/stats).
int apply(uint8_t *packet, uint16_t len);

}  // namespace cc_to_pitchbend
