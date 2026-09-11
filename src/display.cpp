#include "display.h"

#include <Arduino_GFX_Library.h>

#include "config.h"

namespace display {
namespace {

Arduino_DataBus *bus_   = nullptr;
Arduino_GFX     *panel_ = nullptr;
Arduino_GFX     *gfx_   = nullptr;

constexpr uint16_t COL_BG     = 0x0000;
constexpr uint16_t COL_HDR    = 0x001F;   // blue  (selection screens)
constexpr uint16_t COL_HDR_OK = 0x05E0;   // green (routing screen)
constexpr uint16_t COL_TEXT   = 0xFFFF;
constexpr uint16_t COL_DIM    = 0xC618;
constexpr uint16_t COL_SEP    = 0x2104;
constexpr uint16_t COL_CURSOR = 0x033F;
constexpr uint16_t COL_GOOD   = 0x07E0;
constexpr uint16_t COL_BAD    = 0xF800;

void titleBar(const char *txt, uint16_t bg) {
  gfx_->fillRect(0, 0, DISPLAY_WIDTH, 18, bg);
  gfx_->setTextSize(1);
  gfx_->setTextColor(COL_TEXT);
  gfx_->setCursor(3, 6);
  gfx_->print(txt);
}

}  // namespace

void init(const BoardProfile &board) {
  pinMode(board.backlight, OUTPUT);
  digitalWrite(board.backlight, HIGH);

  bus_ = new Arduino_ESP32SPI(board.dc, board.cs, board.sck, board.mosi, GFX_NOT_DEFINED);
  // The Touch board's JD9853 panel accepts the ST7789 init sequence, so one
  // driver covers both boards (Arduino_GFX has no JD9853 driver).
  panel_ = new Arduino_ST7789(bus_, board.rst, DISPLAY_ROTATION, true /* IPS */,
                              DISPLAY_WIDTH, DISPLAY_HEIGHT, 34, 0, 34, 0);
#if DISPLAY_USE_CANVAS
  gfx_ = new Arduino_Canvas(DISPLAY_WIDTH, DISPLAY_HEIGHT, panel_);
#else
  gfx_ = panel_;
#endif
  if (!gfx_->begin()) Serial.println("[display] gfx->begin() failed");
  gfx_->fillScreen(COL_BG);
}

void showStatus(const char *line1, const char *line2) {
  gfx_->fillScreen(COL_BG);
  titleBar("BLE MIDI ROUTER", COL_HDR);
  gfx_->setTextColor(COL_TEXT);
  gfx_->setTextSize(1);
  gfx_->setCursor(6, 60);
  gfx_->print(line1);
  gfx_->setCursor(6, 74);
  gfx_->print(line2);
  gfx_->flush();
}

void showScanning(const char *title) {
  gfx_->fillScreen(COL_BG);
  titleBar(title, COL_HDR);
  gfx_->setTextSize(1);
  gfx_->setTextColor(COL_DIM);
  gfx_->setCursor(6, 40);
  gfx_->print("scanning for BLE-MIDI...");
  gfx_->flush();
}

void showScanList(const char *title, const std::vector<DeviceRow> &rows, int cursor) {
  gfx_->fillScreen(COL_BG);
  titleBar(title, COL_HDR);
  gfx_->setTextSize(1);

  const int rowH = 28;
  const int visible = (DISPLAY_HEIGHT - 22) / rowH;
  int top = 0;
  if (cursor >= visible) top = cursor - visible + 1;

  int y = 22;
  for (int i = top; i < (int)rows.size() && i < top + visible; ++i) {
    const DeviceRow &r = rows[i];
    if (i == cursor) gfx_->fillRect(0, y - 2, DISPLAY_WIDTH, rowH, COL_CURSOR);

    gfx_->setTextColor(COL_TEXT);
    gfx_->setCursor(4, y + 2);
    std::string nm = r.name.empty() ? std::string("(unnamed)") : r.name;
    if (nm.size() > 27) nm.resize(27);
    gfx_->print(nm.c_str());

    gfx_->setTextColor(i == cursor ? COL_TEXT : COL_DIM);
    gfx_->setCursor(4, y + 14);
    gfx_->printf("%s %ddBm", r.address.c_str(), r.rssi);

    gfx_->drawFastHLine(0, y + rowH - 3, DISPLAY_WIDTH, COL_SEP);
    y += rowH;
  }

  gfx_->setTextColor(COL_DIM);
  gfx_->setCursor(4, DISPLAY_HEIGHT - 10);
  gfx_->printf("%d found  short:next long:pick", (int)rows.size());
  gfx_->flush();
}

void showRouting(const RoutingView &v) {
  gfx_->fillScreen(COL_BG);
  const bool ok = v.srcConnected && v.tgtConnected;
  titleBar(ok ? "ROUTING" : "LINK LOST", ok ? COL_HDR_OK : COL_BAD);
  gfx_->setTextSize(1);

  gfx_->setTextColor(COL_TEXT);
  gfx_->setCursor(4, 26);
  gfx_->print("SRC");
  gfx_->setTextColor(v.srcConnected ? COL_GOOD : COL_BAD);
  gfx_->setCursor(34, 26);
  gfx_->print(v.srcConnected ? "connected" : "...");
  gfx_->setTextColor(COL_DIM);
  gfx_->setCursor(4, 40);
  gfx_->print(v.srcAddress.c_str());

  gfx_->setTextColor(COL_TEXT);
  gfx_->setCursor(80, 58);
  gfx_->print("|");
  gfx_->setCursor(80, 68);
  gfx_->print("v");

  gfx_->setTextColor(COL_TEXT);
  gfx_->setCursor(4, 84);
  gfx_->print("TGT");
  gfx_->setTextColor(v.tgtConnected ? COL_GOOD : COL_BAD);
  gfx_->setCursor(34, 84);
  gfx_->print(v.tgtConnected ? "connected" : "...");
  gfx_->setTextColor(COL_DIM);
  gfx_->setCursor(4, 98);
  gfx_->print(v.tgtAddress.c_str());

  gfx_->drawFastHLine(0, 116, DISPLAY_WIDTH, COL_SEP);

  gfx_->setTextColor(COL_TEXT);
  gfx_->setCursor(4, 126);
  gfx_->printf("rx  : %lu", (unsigned long)v.rx);
  gfx_->setCursor(4, 140);
  gfx_->printf("fwd : %lu", (unsigned long)v.fwd);
#if ENABLE_CC_TO_PITCHBEND
  gfx_->setCursor(90, 126);
  gfx_->printf("cc%d>pb", CC2PB_CONTROLLER);
  gfx_->setCursor(90, 140);
  gfx_->printf("%lu @%u%%", (unsigned long)v.cc2pbHits, (unsigned)CC2PB_RANGE_PCT);
#endif
#if ENABLE_TRANSPOSE
  gfx_->setTextColor(COL_GOOD);
  gfx_->setCursor(90, 154);
  gfx_->printf("xpose %+d", v.transpose);
#endif
  if (v.drop) {
    gfx_->setTextColor(COL_BAD);
    gfx_->setCursor(4, 154);
    gfx_->printf("drop: %lu", (unsigned long)v.drop);
  }

  gfx_->setTextColor(COL_DIM);
  gfx_->setCursor(4, 172);
  gfx_->print("last:");
  gfx_->setTextColor(COL_TEXT);
  gfx_->setCursor(34, 172);
  if (v.lastMessageLen) {
    char hex[3 * 8 + 1];
    int o = 0;
    for (int i = 0; i < v.lastMessageLen && i < 8; ++i)
      o += snprintf(hex + o, sizeof(hex) - o, "%02X ", v.lastMessage[i]);
    gfx_->print(hex);
    gfx_->setTextColor(COL_DIM);
    gfx_->setCursor(34, 186);
    gfx_->printf("%lus ago", (unsigned long)(v.lastMessageAgeMs / 1000));
  } else {
    gfx_->print("--");
  }

  gfx_->setTextColor(COL_DIM);
  gfx_->setCursor(4, DISPLAY_HEIGHT - 22);
#if ENABLE_TRANSPOSE
  gfx_->print("short: cycle transpose");
#else
  gfx_->print("short: reset counters");
#endif
  gfx_->setCursor(4, DISPLAY_HEIGHT - 10);
  gfx_->print("long : forget + rescan");
  gfx_->flush();
}

void showError(const char *message) {
  gfx_->fillScreen(COL_BG);
  titleBar("ERROR", COL_BAD);
  gfx_->setTextSize(1);
  gfx_->setTextColor(COL_TEXT);
  gfx_->setCursor(6, 50);
  gfx_->print(message);
  gfx_->setTextColor(COL_DIM);
  gfx_->setCursor(6, 80);
  gfx_->print("short: retry");
  gfx_->setCursor(6, 94);
  gfx_->print("long : forget + rescan");
  gfx_->flush();
}

}  // namespace display
