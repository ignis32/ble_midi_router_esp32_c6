#pragma once

// The two near-identical Waveshare ESP32-C6 1.47" LCD boards this firmware
// supports, auto-detected at boot by probing I2C for the touch controller /
// IMU that only the Touch variant has.
struct BoardProfile {
  const char *name;
  int  mosi, sck, cs, dc, rst, backlight, button;
  bool jd9853Panel;   // informational only -- both boards use the ST7789 init sequence
};

// Probe the I2C bus and return the matching board profile.
const BoardProfile &detectBoard();
