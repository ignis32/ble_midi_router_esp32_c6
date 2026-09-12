#pragma once
#include <Arduino.h>
#include <string>
#include <vector>

#include "board.h"

// All screens the router shows on the 172x320 ST7789 panel. Owns the GFX
// objects itself -- callers pass plain data, never touch the display library.
namespace display {

struct DeviceRow {
  std::string name;
  std::string address;
  int         rssi;
};

struct RoutingView {
  bool srcConnected, tgtConnected;
  std::string srcAddress, tgtAddress;
  uint32_t rx, fwd, drop;
  int8_t   transpose;
  uint32_t cc2pbHits;          // ignored when ENABLE_CC_TO_PITCHBEND is 0
  const uint8_t *lastMessage;
  uint8_t  lastMessageLen;
  uint32_t lastMessageAgeMs;
};

void init(const BoardProfile &board);

void showStatus(const char *line1, const char *line2, const char *hint = nullptr);
void showScanning(const char *title);
// `confirming` frames the cursor row and swaps the footer hint once BOOT has
// been held past the long-press threshold, so it's visible before release
// whether holding long enough has registered.
void showScanList(const char *title, const std::vector<DeviceRow> &rows, int cursor,
                  bool confirming);
void showRouting(const RoutingView &view);
void showError(const char *message);

}  // namespace display
