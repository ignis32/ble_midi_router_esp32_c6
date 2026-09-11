#include "board.h"

#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr int I2C_SDA = 18, I2C_SCL = 19;
constexpr uint8_t ADDR_AXS5106L = 0x63;   // touch controller (Touch variant only)
constexpr uint8_t ADDR_QMI8658  = 0x6B;   // IMU (Touch variant only)

const BoardProfile kNonTouch = {
    "ESP32-C6-LCD-1.47", 6, 7, 14, 15, 21, 22, 9, false};
const BoardProfile kTouch = {
    "ESP32-C6-Touch-LCD-1.47", 2, 1, 14, 15, 22, 23, 8, true};

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

}  // namespace

const BoardProfile &detectBoard() {
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  delay(20);
  return (i2cPresent(ADDR_AXS5106L) || i2cPresent(ADDR_QMI8658)) ? kTouch : kNonTouch;
}
