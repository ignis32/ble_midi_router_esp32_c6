#pragma once
#include <Arduino.h>

// Runs every enabled MIDI transform over one BLE-MIDI packet, in place. The
// MIDI pipe calls this without needing to know which transforms exist or
// which are enabled -- add a new transform by writing one module (see
// transpose.* / cc_to_pitchbend.* for the pattern) and registering it here.
namespace transforms {

// Returns the number of "notable" rewrites (currently: CC->PitchBend
// conversions) for the UI/stats counter. `len` never changes.
int apply(uint8_t *packet, uint16_t len);

}  // namespace transforms
