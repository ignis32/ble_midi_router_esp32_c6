#pragma once
// ============================================================================
//  BLE MIDI Router — user-tunable settings.
//
//  Board pin tables live in board.h/.cpp (hardware definitions, not settings).
//  Display colours live in display.cpp (implementation detail, not settings).
// ============================================================================

// --------------------------------------------------------------------- BLE
#define BLE_DEVICE_NAME         "MIDI-Router"
#define BLE_TX_POWER_DBM        3        // NimBLEDevice::setPower()
#define BLE_PREFERRED_MTU       128

// The BLE-MIDI characteristic requires an encrypted link on many devices.
// Bond, no MITM, "Just Works" pairing so it happens without user input.
#define BLE_BOND                 true
#define BLE_MITM                 false
#define BLE_SECURE_CONNECTIONS   false

// Connection interval requested on both links, in 1.25 ms units (6 = 7.5 ms,
// 12 = 15 ms). Two hops at NimBLE's ~30-50 ms default would stack to 60-100 ms
// of round-trip latency; this keeps each hop close to one interval.
#define BLE_CONN_INTERVAL_MIN    6
#define BLE_CONN_INTERVAL_MAX    12
#define BLE_CONN_LATENCY         0
#define BLE_CONN_TIMEOUT_10MS    400      // 4 s supervision timeout

#define BLE_CONNECT_TIMEOUT_MS         8000
#define BLE_CONNECT_RETRIES            4
// Consecutive connect failures after which the router does a full BLE-stack
// restart instead of just retrying (clears leaked connection slots and lets
// a peer that still thinks it's connected time out and drop the stale link).
#define BLE_RECONNECT_FAIL_STREAK_FOR_RESTART  3

#define BLE_SCAN_INTERVAL        80       // x0.625 ms
#define BLE_SCAN_WINDOW          40       // x0.625 ms

// --------------------------------------------------------------------- NVS
// Stored Source/Target pair + transpose, survives reboot.
#define NVS_NAMESPACE            "midirt"
#define NVS_KEY_SRC_ADDR         "src"
#define NVS_KEY_TGT_ADDR         "tgt"
#define NVS_KEY_SRC_TYPE         "srcT"
#define NVS_KEY_TGT_TYPE         "tgtT"
#define NVS_KEY_TRANSPOSE        "xpose"

// ---------------------------------------------------------------- MIDI pipe
#define MIDI_QUEUE_DEPTH           64    // queued source packets awaiting forward
#define MIDI_RAW_MAX_BYTES        160    // >= any MTU-3 we negotiate
#define MIDI_WRITE_RETRIES          8    // per packet, on a congested BLE stack
#define MIDI_WRITE_RETRY_DELAY_MS   3

// ------------------------------------------------------------------- Display
#define DISPLAY_WIDTH             172
#define DISPLAY_HEIGHT            320
#define DISPLAY_ROTATION            0    // 0 = portrait
#define DISPLAY_USE_CANVAS          1    // full-frame buffer, no flicker (~110 KB RAM)
#define DISPLAY_REDRAW_MS        1000    // ROUTING screen refresh (status-only, keep it slow)

// --------------------------------------------------------------------- Debug
// Dump every MIDI packet before/after the transform chain as a diffable pair
// on Serial. Costs a copy + snprintf + USB-CDC write per packet on the pump
// task — DEBUG ONLY, adds perceptible latency. Keep 0 for normal use.
#define DEBUG_LOG_TRANSFORMS        0

// ============================================================================
//  MIDI transforms — optional in-place packet rewrites, applied in the pump
//  task just before each packet is forwarded. See src/transforms/.
// ============================================================================

// ---- Transpose ------------------------------------------------------------
// Shifts Note Off / Note On / Poly Key Pressure note numbers by a number of
// semitones. General-purpose: safe with any BLE-MIDI controller/synth pair.
#define ENABLE_TRANSPOSE            1
// Semitone values the BOOT button cycles through on the ROUTING screen, in
// ascending pitch order (wraps from the top back to the bottom): an octave
// down, a fifth down, unison, a fifth up, an octave up.
#define TRANSPOSE_STEPS             {-12, -7, 0, 7, 12}

// ---- CC -> Pitch Bend ------------------------------------------------------
// Converts one Control Change controller to Pitch Bend, with a dead zone and
// a configurable depth. Written for the Artinoise Re.corder BLE flute's
// "rotation" CC (centred on 64) — this is a personal, controller-specific
// mapping. Enabled here because this build actually drives a Re.corder;
// a fresh clone for a different controller should flip this to 0 and use
// src/transforms/cc_to_pitchbend.* as a template for its own mapping.
#define ENABLE_CC_TO_PITCHBEND      1
#define CC2PB_CONTROLLER           52    // CC number to convert
#define CC2PB_DEADZONE              6    // CC units either side of 64 -> centre
#define CC2PB_RANGE_PCT            10    // % of the full 14-bit bend at the CC extremes
