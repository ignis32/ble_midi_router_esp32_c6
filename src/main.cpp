// ---------------------------------------------------------------------------
//  BLE MIDI Router  —  Waveshare ESP32-C6 1.47" LCD boards
//
//  - scans for BLE-MIDI peripherals (devices advertising the MIDI service)
//  - pick a SOURCE, then a TARGET with the BOOT button
//        short press = move cursor        long press (>600 ms) = select
//  - connects to both as a BLE central and forwards MIDI SOURCE -> TARGET
//  - the pair is stored in NVS and reconnected automatically on boot;
//    long-press on the routing screen forgets it and rescans
//
//  Topology: this board is central to BOTH devices, so both must be BLE-MIDI
//  peripherals (keyboards, WIDI, most synths). Forwarding to a host that is
//  itself a BLE central (phone / PC) is out of scope here.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include <NimBLEDevice.h>

#include <algorithm>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "blemidi.h"
#include "transform.h"

// --------------------------- board definitions ---------------------------
struct BoardCfg {
  const char *name;
  int mosi, sck, cs, dc, rst, bl, btn;
  bool jd9853;
};
static const BoardCfg BOARD_NONTOUCH = {
    "ESP32-C6-LCD-1.47", 6, 7, 14, 15, 21, 22, 9, false};
static const BoardCfg BOARD_TOUCH = {
    "ESP32-C6-Touch-LCD-1.47", 2, 1, 14, 15, 22, 23, 8, true};

static constexpr int I2C_SDA = 18, I2C_SCL = 19;
static constexpr uint8_t ADDR_AXS5106L = 0x63, ADDR_QMI8658 = 0x6B;
static const BoardCfg *BOARD = &BOARD_NONTOUCH;

// Forwarding mode:
//   RAW_PASSTHROUGH 1 -> copy the source notification payload to the target
//                        verbatim. No MIDI parsing, so nothing can be
//                        fabricated / mis-parsed. Correct for a 1:1 router.
//   (set to 0 only when a future filter/merge/remap feature needs blemidi.h)
#define RAW_PASSTHROUGH 1
// LOG_RAW 1 -> hex-dump every inbound "<" / outbound ">" payload on serial.
// Costs an snprintf + USB-CDC write per MIDI message on the BLE hot path —
// adds noticeable latency. Debug only; keep 0 for normal use.
#define LOG_RAW 0

// ------------------------------- display --------------------------------
static constexpr int16_t SCREEN_W = 172, SCREEN_H = 320;
static constexpr uint8_t ROTATION = 0;
#define USE_CANVAS 1

Arduino_DataBus *bus   = nullptr;
Arduino_GFX     *panel = nullptr;
Arduino_GFX     *gfx   = nullptr;

static constexpr uint16_t COL_BG     = 0x0000;
static constexpr uint16_t COL_HDR    = 0x001F;   // blue  (selection screens)
static constexpr uint16_t COL_HDR_OK = 0x05E0;   // green (routing screen)
static constexpr uint16_t COL_TEXT   = 0xFFFF;
static constexpr uint16_t COL_DIM    = 0xC618;
static constexpr uint16_t COL_SEP    = 0x2104;
static constexpr uint16_t COL_CURSOR = 0x033F;
static constexpr uint16_t COL_GOOD   = 0x07E0;
static constexpr uint16_t COL_BAD    = 0xF800;

// ------------------------------- state ----------------------------------
enum class State { ScanSource, ScanTarget, Connecting, Routing, Failed };
static State g_state = State::ScanSource;

struct Found {
  uint64_t    addr;      // NimBLEAddress as uint64
  uint8_t     type;
  std::string name;
  int         rssi;
  uint32_t    lastSeen;
};
static std::vector<Found> g_found;
static portMUX_TYPE       g_foundMux = portMUX_INITIALIZER_UNLOCKED;
static int                g_cursor = 0;

static uint64_t g_srcAddr = 0, g_tgtAddr = 0;
static uint8_t  g_srcType = 0, g_tgtType = 0;
static bool     g_srcSet  = false, g_tgtSet = false;

static NimBLEClient             *g_srcCli  = nullptr;
static NimBLEClient             *g_tgtCli  = nullptr;
static NimBLERemoteCharacteristic *g_tgtChar = nullptr;
static volatile bool  g_srcConn = false, g_tgtConn = false;
static volatile bool  g_linkLost = false;

static volatile uint32_t g_rxCount = 0, g_fwdCount = 0, g_dropCount = 0;
static volatile uint32_t g_xformCount = 0;   // CC->PitchBend rewrites
static uint8_t g_pbRangePct = CC2PB_RANGE_PCT;   // live-adjustable bend depth
static const uint8_t PB_RANGE_STEPS[] = {100, 75, 50, 33, 25, 15, 10, 5};
static uint8_t  g_lastMsg[12];
static uint8_t  g_lastMsgLen = 0;
static uint32_t g_lastMsgAt  = 0;

// source notify (producer, BLE host task) -> pump task (consumer, writes target).
// One queue entry = one whole source notification payload, forwarded verbatim.
static constexpr uint16_t RAW_MAX = 160;   // >= any MTU-3 we negotiate (128)
struct RawFrame {
  uint16_t len;
  uint8_t  data[RAW_MAX];
};
static QueueHandle_t g_midiQ    = nullptr;
static TaskHandle_t  g_pumpTask = nullptr;

#if LOG_RAW
static void logHex(char dir, const uint8_t *p, size_t n) {
  char line[3 * RAW_MAX + 8];
  int o = snprintf(line, sizeof(line), "%c ", dir);
  for (size_t i = 0; i < n && o < (int)sizeof(line) - 4; ++i)
    o += snprintf(line + o, sizeof(line) - o, "%02X ", p[i]);
  Serial.println(line);
}
#else
static inline void logHex(char, const uint8_t *, size_t) {}
#endif

static char g_failMsg[48] = "";

static Preferences g_prefs;

// ------------------------------- helpers --------------------------------
static NimBLEUUID midiSvcUuid() { return NimBLEUUID(blemidi::SERVICE_UUID); }

static void savePair() {
  g_prefs.begin("midirt", false);
  g_prefs.putULong64("src", g_srcAddr);
  g_prefs.putULong64("tgt", g_tgtAddr);
  g_prefs.putUChar("srcT", g_srcType);
  g_prefs.putUChar("tgtT", g_tgtType);
  g_prefs.end();
}
static void forgetPair() {
  g_prefs.begin("midirt", false);
  g_prefs.clear();
  g_prefs.end();
  g_srcSet = g_tgtSet = false;
}
static bool loadPair() {
  g_prefs.begin("midirt", true);
  g_srcAddr = g_prefs.getULong64("src", 0);
  g_tgtAddr = g_prefs.getULong64("tgt", 0);
  g_srcType = g_prefs.getUChar("srcT", 0);
  g_tgtType = g_prefs.getUChar("tgtT", 0);
  g_pbRangePct = g_prefs.getUChar("pbrng", CC2PB_RANGE_PCT);
  g_prefs.end();
  g_srcSet = g_srcAddr != 0;
  g_tgtSet = g_tgtAddr != 0;
  return g_srcSet && g_tgtSet;
}
static void savePbRange() {
  g_prefs.begin("midirt", false);
  g_prefs.putUChar("pbrng", g_pbRangePct);
  g_prefs.end();
}

// --------------------------- board detection ---------------------------
static bool i2cPresent(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}
static const BoardCfg *detectBoard() {
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  delay(20);
  return (i2cPresent(ADDR_AXS5106L) || i2cPresent(ADDR_QMI8658)) ? &BOARD_TOUCH
                                                                 : &BOARD_NONTOUCH;
}

// ----------------------------- BLE scanning ---------------------------
class ScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    if (!dev->isAdvertisingService(midiSvcUuid())) return;
    const uint64_t a = (uint64_t)dev->getAddress();
    const uint8_t  t = dev->getAddress().getType();
    std::string nm = dev->haveName() ? dev->getName() : std::string();
    const int r = dev->getRSSI();
    const uint32_t now = millis();

    portENTER_CRITICAL(&g_foundMux);
    bool hit = false;
    for (auto &f : g_found) {
      if (f.addr == a) {
        f.rssi = r; f.lastSeen = now;
        if (!nm.empty()) f.name = nm;
        hit = true;
        break;
      }
    }
    if (!hit) g_found.push_back({a, t, nm, r, now});
    portEXIT_CRITICAL(&g_foundMux);
  }
};
static ScanCB g_scanCB;

static void startScan() {
  NimBLEScan *s = NimBLEDevice::getScan();
  s->setScanCallbacks(&g_scanCB, false);
  s->setActiveScan(true);
  s->setInterval(80);
  s->setWindow(40);
  s->setMaxResults(0);
  s->start(0, false, true);
}
static void stopScan() { NimBLEDevice::getScan()->stop(); }

// ---------------------------- MIDI forwarding -------------------------
// Producer: runs in the BLE host task. Copy the payload and enqueue — NO BLE
// writes here (writing unpaced from this context exhausts the mbuf pool ->
// ENOMEM -> the link drops) and NO parsing (raw passthrough can't fabricate
// or mis-parse anything). The pump task does the writing.
static void onSrcNotify(NimBLERemoteCharacteristic *c, uint8_t *data, size_t len,
                        bool isNotify) {
  g_rxCount++;
  if (!g_midiQ || len == 0) return;
  logHex('<', data, len);

  RawFrame f;
  f.len = len > RAW_MAX ? RAW_MAX : (uint16_t)len;
  memcpy(f.data, data, f.len);
  if (xQueueSend(g_midiQ, &f, 0) != pdTRUE) g_dropCount++;   // queue full
}

// Consumer: forwards each source notification payload to the target verbatim,
// one write per packet, with backoff on a congested stack.
static void midiPumpTask(void *) {
  RawFrame f;
  for (;;) {
    if (xQueueReceive(g_midiQ, &f, pdMS_TO_TICKS(50)) != pdTRUE) continue;

    // in-place packet transforms (CC#52 -> Pitch Bend); length unchanged
    g_xformCount += xform::apply(f.data, f.len, g_pbRangePct);

    NimBLERemoteCharacteristic *ch = g_tgtChar;
    if (!ch || !g_tgtConn) { g_dropCount++; continue; }

    int tries = 0;
    for (;;) {
      ch = g_tgtChar;
      if (!ch || !g_tgtConn) { g_dropCount++; break; }
      if (ch->writeValue(f.data, f.len, false)) {
        g_fwdCount++;
        logHex('>', f.data, f.len);
        break;
      }
      if (++tries >= 8) { g_dropCount++; break; }
      vTaskDelay(pdMS_TO_TICKS(3));   // let the stack drain, then retry
    }

    const uint8_t n = f.len < sizeof(g_lastMsg) ? f.len : sizeof(g_lastMsg);
    memcpy(g_lastMsg, f.data, n);
    g_lastMsgLen = n;
    g_lastMsgAt  = millis();
  }
}

class SrcCliCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *c) override {
    Serial.println("[MIDI-RT] SRC onConnect");
  }
  void onDisconnect(NimBLEClient *c, int reason) override {
    Serial.printf("[MIDI-RT] SRC onDisconnect reason=%d (0x%02X)\n", reason, reason & 0xFF);
    g_srcConn = false;
    g_linkLost = true;
  }
};
class TgtCliCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *c) override {
    Serial.println("[MIDI-RT] TGT onConnect");
  }
  void onDisconnect(NimBLEClient *c, int reason) override {
    Serial.printf("[MIDI-RT] TGT onDisconnect reason=%d (0x%02X)\n", reason, reason & 0xFF);
    g_tgtConn = false;
    g_tgtChar = nullptr;
    g_linkLost = true;
  }
};
static SrcCliCB g_srcCliCB;
static TgtCliCB g_tgtCliCB;

static void fail(const char *msg) {
  strncpy(g_failMsg, msg, sizeof(g_failMsg) - 1);
  Serial.printf("[MIDI-RT] FAIL: %s\n", msg);
  g_state = State::Failed;
}

static void teardownLinks() {
  if (g_srcCli) { NimBLEDevice::deleteClient(g_srcCli); g_srcCli = nullptr; }
  if (g_tgtCli) { NimBLEDevice::deleteClient(g_tgtCli); g_tgtCli = nullptr; }
  g_tgtChar = nullptr;
  g_srcConn = g_tgtConn = false;
}

static NimBLERemoteCharacteristic *g_srcChar = nullptr;   // held between connect & subscribe

// connect + discover the MIDI characteristic. Does NOT subscribe (source).
static bool connectOne(bool source) {
  const uint64_t a = source ? g_srcAddr : g_tgtAddr;
  const uint8_t  t = source ? g_srcType : g_tgtType;
  NimBLEAddress addr(a, t);

  NimBLEClient *cli = NimBLEDevice::createClient(addr);
  cli->setClientCallbacks(source ? (NimBLEClientCallbacks *)&g_srcCliCB
                                 : (NimBLEClientCallbacks *)&g_tgtCliCB,
                          false);
  cli->setConnectTimeout(8000);
  // Fast interval from the first connection event: 7.5-15 ms, no slave latency,
  // 4 s supervision timeout. Two hops at the ~30-50 ms default would stack to
  // 60-100 ms; this keeps a hop near one interval.
  cli->setConnectionParams(6, 12, 0, 400);
  if (!cli->connect(addr)) {
    NimBLEDevice::deleteClient(cli);
    return false;
  }
  NimBLERemoteService *svc = cli->getService(midiSvcUuid());
  NimBLERemoteCharacteristic *ch =
      svc ? svc->getCharacteristic(NimBLEUUID(blemidi::CHAR_UUID)) : nullptr;
  if (!ch) {
    cli->disconnect();
    NimBLEDevice::deleteClient(cli);
    return false;
  }

  if (source) {
    g_srcCli  = cli;
    g_srcChar = ch;
    g_srcConn = true;
  } else {
    g_tgtCli  = cli;
    g_tgtChar = ch;
    g_tgtConn = true;
  }
  return true;
}

static bool connectWithRetry(bool source, int tries) {
  for (int i = 1; i <= tries; ++i) {
    if (connectOne(source)) return true;
    Serial.printf("[MIDI-RT] %s connect attempt %d/%d failed\n",
                  source ? "SRC" : "TGT", i, tries);
    vTaskDelay(pdMS_TO_TICKS(800));
  }
  return false;
}

static void drawStatus(const char *line1, const char *line2);

static void doConnect() {
  teardownLinks();
  g_srcChar = nullptr;
  g_linkLost = false;
  if (g_midiQ) xQueueReset(g_midiQ);

  // Target first (idle radio), then source. Subscribing to the source starts
  // the notification flood, so do that last once both links are up.
  drawStatus("Connecting to", "TARGET ...");
  Serial.println("[MIDI-RT] connecting TARGET...");
  if (!connectWithRetry(false, 3)) { fail("TARGET connect failed"); return; }
  Serial.printf("[MIDI-RT] TARGET connected, MTU=%u\n",
                g_tgtCli ? g_tgtCli->getMTU() : 0);

  drawStatus("Connecting to", "SOURCE ...");
  Serial.println("[MIDI-RT] connecting SOURCE...");
  if (!connectWithRetry(true, 3)) { fail("SOURCE connect failed"); return; }

  if (!g_srcChar || !g_srcChar->canNotify() ||
      !g_srcChar->subscribe(true, onSrcNotify)) {
    fail("SOURCE subscribe failed");
    return;
  }
  Serial.println("[MIDI-RT] SOURCE subscribed -> ROUTING");

  g_rxCount = g_fwdCount = g_dropCount = g_xformCount = 0;
  g_lastMsgLen = 0;
  g_state = State::Routing;
}

// ------------------------------ rendering ----------------------------
static void titleBar(const char *txt, uint16_t bg) {
  gfx->fillRect(0, 0, SCREEN_W, 18, bg);
  gfx->setTextSize(1);
  gfx->setTextColor(COL_TEXT);
  gfx->setCursor(3, 6);
  gfx->print(txt);
}

static void drawStatus(const char *line1, const char *line2) {
  gfx->fillScreen(COL_BG);
  titleBar("BLE MIDI ROUTER", COL_HDR);
  gfx->setTextColor(COL_TEXT);
  gfx->setTextSize(1);
  gfx->setCursor(6, 60);
  gfx->print(line1);
  gfx->setCursor(6, 74);
  gfx->print(line2);
  gfx->flush();
}

static void drawScanList(bool pickingSource) {
  gfx->fillScreen(COL_BG);
  titleBar(pickingSource ? "Pick SOURCE" : "Pick TARGET", COL_HDR);

  std::vector<Found> list;
  portENTER_CRITICAL(&g_foundMux);
  list = g_found;
  portEXIT_CRITICAL(&g_foundMux);

  // when picking target, don't offer the chosen source
  if (!pickingSource && g_srcSet) {
    list.erase(std::remove_if(list.begin(), list.end(),
                              [](const Found &f) { return f.addr == g_srcAddr; }),
               list.end());
  }

  gfx->setTextSize(1);
  if (list.empty()) {
    gfx->setTextColor(COL_DIM);
    gfx->setCursor(6, 40);
    gfx->print("scanning for BLE-MIDI...");
    gfx->flush();
    return;
  }

  if (g_cursor >= (int)list.size()) g_cursor = 0;

  const int rowH = 28;
  const int visible = (SCREEN_H - 22) / rowH;
  int top = 0;
  if (g_cursor >= visible) top = g_cursor - visible + 1;

  int y = 22;
  for (int i = top; i < (int)list.size() && i < top + visible; ++i) {
    const Found &f = list[i];
    if (i == g_cursor) gfx->fillRect(0, y - 2, SCREEN_W, rowH, COL_CURSOR);

    NimBLEAddress a(f.addr, f.type);
    gfx->setTextColor(COL_TEXT);
    gfx->setCursor(4, y + 2);
    std::string nm = f.name.empty() ? std::string("(unnamed)") : f.name;
    if (nm.size() > 27) nm.resize(27);
    gfx->print(nm.c_str());

    gfx->setTextColor(i == g_cursor ? COL_TEXT : COL_DIM);
    gfx->setCursor(4, y + 14);
    gfx->printf("%s %ddBm", a.toString().c_str(), f.rssi);

    gfx->drawFastHLine(0, y + rowH - 3, SCREEN_W, COL_SEP);
    y += rowH;
  }

  gfx->setTextColor(COL_DIM);
  gfx->setCursor(4, SCREEN_H - 10);
  gfx->printf("%d found  short:next long:pick", (int)list.size());
  gfx->flush();
}

static void drawRouting() {
  gfx->fillScreen(COL_BG);
  const bool ok = g_srcConn && g_tgtConn;
  titleBar(ok ? "ROUTING" : "LINK LOST", ok ? COL_HDR_OK : COL_BAD);

  NimBLEAddress sa(g_srcAddr, g_srcType), ta(g_tgtAddr, g_tgtType);
  gfx->setTextSize(1);

  gfx->setTextColor(COL_TEXT);
  gfx->setCursor(4, 26);
  gfx->print("SRC");
  gfx->setTextColor(g_srcConn ? COL_GOOD : COL_BAD);
  gfx->setCursor(34, 26);
  gfx->print(g_srcConn ? "connected" : "...");
  gfx->setTextColor(COL_DIM);
  gfx->setCursor(4, 40);
  gfx->print(sa.toString().c_str());

  gfx->setTextColor(COL_TEXT);
  gfx->setCursor(80, 58);
  gfx->print("|");
  gfx->setCursor(80, 68);
  gfx->print("v");

  gfx->setTextColor(COL_TEXT);
  gfx->setCursor(4, 84);
  gfx->print("TGT");
  gfx->setTextColor(g_tgtConn ? COL_GOOD : COL_BAD);
  gfx->setCursor(34, 84);
  gfx->print(g_tgtConn ? "connected" : "...");
  gfx->setTextColor(COL_DIM);
  gfx->setCursor(4, 98);
  gfx->print(ta.toString().c_str());

  gfx->drawFastHLine(0, 116, SCREEN_W, COL_SEP);

  gfx->setTextColor(COL_TEXT);
  gfx->setCursor(4, 126);
  gfx->printf("rx  : %lu", (unsigned long)g_rxCount);
  gfx->setCursor(4, 140);
  gfx->printf("fwd : %lu", (unsigned long)g_fwdCount);
#if CC2PB_ENABLE
  gfx->setCursor(90, 126);
  gfx->printf("cc%d>pb", CC2PB_CC);
  gfx->setCursor(90, 140);
  gfx->printf("%lu", (unsigned long)g_xformCount);
  gfx->setTextColor(COL_GOOD);
  gfx->setCursor(90, 154);
  gfx->printf("bend %u%%", g_pbRangePct);
#endif
  if (g_dropCount) {
    gfx->setTextColor(COL_BAD);
    gfx->setCursor(4, 154);
    gfx->printf("drop: %lu", (unsigned long)g_dropCount);
  }

  gfx->setTextColor(COL_DIM);
  gfx->setCursor(4, 172);
  gfx->print("last:");
  gfx->setTextColor(COL_TEXT);
  gfx->setCursor(34, 172);
  if (g_lastMsgLen) {
    char hex[3 * 8 + 1];
    int o = 0;
    for (int i = 0; i < g_lastMsgLen; ++i)
      o += snprintf(hex + o, sizeof(hex) - o, "%02X ", g_lastMsg[i]);
    gfx->print(hex);
    gfx->setTextColor(COL_DIM);
    gfx->setCursor(34, 186);
    gfx->printf("%lus ago", (unsigned long)((millis() - g_lastMsgAt) / 1000));
  } else {
    gfx->print("--");
  }

  gfx->setTextColor(COL_DIM);
  gfx->setCursor(4, SCREEN_H - 22);
  gfx->print("short: bend depth");
  gfx->setCursor(4, SCREEN_H - 10);
  gfx->print("long : forget + rescan");
  gfx->flush();
}

static void drawFailed() {
  gfx->fillScreen(COL_BG);
  titleBar("ERROR", COL_BAD);
  gfx->setTextSize(1);
  gfx->setTextColor(COL_TEXT);
  gfx->setCursor(6, 50);
  gfx->print(g_failMsg);
  gfx->setTextColor(COL_DIM);
  gfx->setCursor(6, 80);
  gfx->print("short: retry");
  gfx->setCursor(6, 94);
  gfx->print("long : forget + rescan");
  gfx->flush();
}

// ------------------------------- button ------------------------------
enum class Press { None, Short, Long };
static Press pollButton() {
  static bool wasDown = false;
  static uint32_t downAt = 0;
  const bool down = digitalRead(BOARD->btn) == LOW;
  const uint32_t now = millis();
  Press p = Press::None;
  if (down && !wasDown) {
    wasDown = true;
    downAt = now;
  } else if (!down && wasDown) {
    wasDown = false;
    const uint32_t held = now - downAt;
    if (held >= 30) p = held > 600 ? Press::Long : Press::Short;
  }
  return p;
}

static int scanListSize(bool pickingSource) {
  portENTER_CRITICAL(&g_foundMux);
  int n = g_found.size();
  if (!pickingSource && g_srcSet) {
    for (auto &f : g_found)
      if (f.addr == g_srcAddr) { n--; break; }
  }
  portEXIT_CRITICAL(&g_foundMux);
  return n;
}

// pick the cursor-th entry from the same filtered/ordered list drawScanList uses
static bool pickAt(bool pickingSource, int idx, uint64_t &addr, uint8_t &type) {
  portENTER_CRITICAL(&g_foundMux);
  std::vector<Found> list = g_found;
  portEXIT_CRITICAL(&g_foundMux);
  if (!pickingSource && g_srcSet) {
    list.erase(std::remove_if(list.begin(), list.end(),
                              [](const Found &f) { return f.addr == g_srcAddr; }),
               list.end());
  }
  if (idx < 0 || idx >= (int)list.size()) return false;
  addr = list[idx].addr;
  type = list[idx].type;
  return true;
}

// -------------------------------- setup ------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[MIDI-RT] boot");

  BOARD = detectBoard();
  Serial.printf("[MIDI-RT] board: %s\n", BOARD->name);

  pinMode(BOARD->btn, INPUT_PULLUP);
  pinMode(BOARD->bl, OUTPUT);
  digitalWrite(BOARD->bl, HIGH);

  bus = new Arduino_ESP32SPI(BOARD->dc, BOARD->cs, BOARD->sck, BOARD->mosi,
                             GFX_NOT_DEFINED);
  panel = new Arduino_ST7789(bus, BOARD->rst, ROTATION, true, SCREEN_W, SCREEN_H,
                             34, 0, 34, 0);
#if USE_CANVAS
  gfx = new Arduino_Canvas(SCREEN_W, SCREEN_H, panel);
#else
  gfx = panel;
#endif
  if (!gfx->begin()) Serial.println("[MIDI-RT] gfx->begin() failed");
  gfx->fillScreen(COL_BG);

  NimBLEDevice::init("MIDI-Router");
  NimBLEDevice::setPower(3 /* dBm */);
  NimBLEDevice::setMTU(128);
  // The BLE-MIDI characteristic requires an encrypted link on many devices.
  // Bond, no MITM, "Just Works" pairing so it happens without user input.
  NimBLEDevice::setSecurityAuth(true, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  g_midiQ = xQueueCreate(64, sizeof(RawFrame));
  xTaskCreate(midiPumpTask, "midiPump", 8192, nullptr, 6, &g_pumpTask);

  const bool held = digitalRead(BOARD->btn) == LOW;
  if (loadPair() && !held) {
    Serial.println("[MIDI-RT] stored pair found -> connecting");
    g_state = State::Connecting;
  } else {
    if (held) Serial.println("[MIDI-RT] BOOT held -> forced rescan");
    forgetPair();
    startScan();
    g_state = State::ScanSource;
  }
}

// -------------------------------- loop -------------------------------
void loop() {
  const Press btn = pollButton();

  switch (g_state) {
    case State::ScanSource:
    case State::ScanTarget: {
      const bool src = g_state == State::ScanSource;
      const int n = scanListSize(src);
      if (btn == Press::Short && n > 0) {
        g_cursor = (g_cursor + 1) % n;
      } else if (btn == Press::Long && n > 0) {
        uint64_t a; uint8_t t;
        if (pickAt(src, g_cursor, a, t)) {
          if (src) {
            g_srcAddr = a; g_srcType = t; g_srcSet = true;
            g_cursor = 0;
            g_state = State::ScanTarget;
            Serial.printf("[MIDI-RT] source = %s\n",
                          NimBLEAddress(a, t).toString().c_str());
          } else {
            g_tgtAddr = a; g_tgtType = t; g_tgtSet = true;
            Serial.printf("[MIDI-RT] target = %s\n",
                          NimBLEAddress(a, t).toString().c_str());
            stopScan();
            savePair();
            g_state = State::Connecting;
          }
        }
      }
      drawScanList(src);
      delay(40);
      break;
    }

    case State::Connecting:
      doConnect();
      break;

    case State::Routing: {
      if (g_linkLost) {
        g_linkLost = false;
        Serial.println("[MIDI-RT] link lost -> reconnecting");
        g_state = State::Connecting;
        break;
      }
      static uint32_t lastDraw = 0;
      if (btn == Press::Short) {
        // cycle the CC->Pitch Bend depth and persist it
        size_t n = sizeof(PB_RANGE_STEPS) / sizeof(PB_RANGE_STEPS[0]);
        size_t idx = 0;
        for (size_t k = 0; k < n; ++k)
          if (PB_RANGE_STEPS[k] == g_pbRangePct) { idx = k; break; }
        g_pbRangePct = PB_RANGE_STEPS[(idx + 1) % n];
        savePbRange();
        Serial.printf("[MIDI-RT] pb range = %u%%\n", g_pbRangePct);
        lastDraw = 0;   // redraw immediately
      } else if (btn == Press::Long) {
        teardownLinks();
        forgetPair();
        g_found.clear();
        g_cursor = 0;
        startScan();
        g_state = State::ScanSource;
        break;
      }
      static uint32_t lastHb = 0;
      static uint32_t lastFwd = 0;
      if (millis() - lastHb > 2000) {
        lastHb = millis();
        if (g_fwdCount != lastFwd) {
          Serial.printf("[MIDI-RT] rx=%lu fwd=%lu drop=%lu cc2pb=%lu(%u%%) last=",
                        (unsigned long)g_rxCount, (unsigned long)g_fwdCount,
                        (unsigned long)g_dropCount, (unsigned long)g_xformCount,
                        g_pbRangePct);
          for (int i = 0; i < g_lastMsgLen; ++i) Serial.printf("%02X ", g_lastMsg[i]);
          Serial.println();
          lastFwd = g_fwdCount;
        }
      }
      // routing screen is status-only; 1 Hz redraw keeps the SPI bus (and the
      // radio) free for MIDI forwarding
      if (millis() - lastDraw >= 1000) {
        drawRouting();
        lastDraw = millis();
      }
      delay(30);
      break;
    }

    case State::Failed: {
      static uint32_t failedAt = 0;
      if (failedAt == 0) failedAt = millis();

      if (btn == Press::Short) {
        failedAt = 0;
        g_state = State::Connecting;
      } else if (btn == Press::Long) {
        failedAt = 0;
        teardownLinks();
        forgetPair();
        g_found.clear();
        g_cursor = 0;
        startScan();
        g_state = State::ScanSource;
      } else if (millis() - failedAt > 4000) {
        failedAt = 0;
        Serial.println("[MIDI-RT] auto-retry after failure");
        g_state = State::Connecting;
      } else {
        drawFailed();
        delay(80);
      }
      break;
    }
  }
}
