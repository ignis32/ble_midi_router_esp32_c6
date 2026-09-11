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
//
//  Forwarding is raw passthrough: each Source notification payload is copied
//  to the Target characteristic verbatim, with no MIDI parsing, so nothing
//  can be fabricated or mis-parsed. Optional in-place packet transforms run
//  just before the write -- see config.h and src/transforms/.
//
//  This file is the app: board/BLE bring-up, the BLE-central connection
//  management, and the screen state machine. Display rendering is in
//  display.*, board detection in board.*, and MIDI transforms in
//  transforms/*, all driven from config.h.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Preferences.h>
#include <NimBLEDevice.h>

#include <algorithm>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "config.h"
#include "board.h"
#include "display.h"
#include "blemidi.h"
#include "transforms/transform_chain.h"
#include "transforms/transpose.h"

// ------------------------------- state ----------------------------------
enum class State { ScanSource, ScanTarget, Connecting, Routing, Failed };
static State g_state = State::ScanSource;
static const BoardProfile *g_board = nullptr;

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

static NimBLEClient               *g_srcCli  = nullptr;
static NimBLEClient               *g_tgtCli  = nullptr;
static NimBLERemoteCharacteristic *g_srcChar = nullptr;   // held between connect & subscribe
static NimBLERemoteCharacteristic *g_tgtChar = nullptr;
static volatile bool  g_srcConn = false, g_tgtConn = false;
static volatile bool  g_linkLost = false;

static volatile uint32_t g_rxCount = 0, g_fwdCount = 0, g_dropCount = 0;
static volatile uint32_t g_xformCount = 0;   // notable transform rewrites (e.g. CC->PitchBend)
static uint8_t  g_lastMsg[12];
static uint8_t  g_lastMsgLen = 0;
static uint32_t g_lastMsgAt  = 0;

static int  g_failStreak = 0;
static char g_failMsg[48] = "";

static Preferences g_prefs;

// Source notify (producer, BLE host task) -> pump task (consumer, writes
// target). One queue entry = one whole source notification payload.
struct RawFrame {
  uint16_t len;
  uint8_t  data[MIDI_RAW_MAX_BYTES];
};
static QueueHandle_t g_midiQ    = nullptr;
static TaskHandle_t  g_pumpTask = nullptr;

#if DEBUG_LOG_TRANSFORMS
static void hexInto(char *dst, size_t cap, const uint8_t *p, size_t n) {
  int o = 0;
  for (size_t i = 0; i < n && o < (int)cap - 4; ++i)
    o += snprintf(dst + o, cap - o, "%02X ", p[i]);
  if (o) dst[o - 1] = '\0';
}
// Pair-dump one packet before and after the transform chain.
static void logTransform(const uint8_t *pre, size_t preLen, const uint8_t *post, size_t postLen) {
  char a[3 * MIDI_RAW_MAX_BYTES], b[3 * MIDI_RAW_MAX_BYTES];
  hexInto(a, sizeof(a), pre, preLen);
  hexInto(b, sizeof(b), post, postLen);
  const bool changed = preLen != postLen || memcmp(pre, post, preLen) != 0;
  Serial.printf("%s xf  in : %s\n        out: %s\n", changed ? "*" : " ", a, b);
}
#endif

// ------------------------------- helpers --------------------------------
static NimBLEUUID midiSvcUuid() { return NimBLEUUID(blemidi::SERVICE_UUID); }

static void savePair() {
  g_prefs.begin(NVS_NAMESPACE, false);
  g_prefs.putULong64(NVS_KEY_SRC_ADDR, g_srcAddr);
  g_prefs.putULong64(NVS_KEY_TGT_ADDR, g_tgtAddr);
  g_prefs.putUChar(NVS_KEY_SRC_TYPE, g_srcType);
  g_prefs.putUChar(NVS_KEY_TGT_TYPE, g_tgtType);
  g_prefs.end();
}
static void forgetPair() {
  g_prefs.begin(NVS_NAMESPACE, false);
  g_prefs.clear();
  g_prefs.end();
  g_srcSet = g_tgtSet = false;
}
static bool loadPair() {
  g_prefs.begin(NVS_NAMESPACE, true);
  g_srcAddr = g_prefs.getULong64(NVS_KEY_SRC_ADDR, 0);
  g_tgtAddr = g_prefs.getULong64(NVS_KEY_TGT_ADDR, 0);
  g_srcType = g_prefs.getUChar(NVS_KEY_SRC_TYPE, 0);
  g_tgtType = g_prefs.getUChar(NVS_KEY_TGT_TYPE, 0);
  transpose::set(g_prefs.getChar(NVS_KEY_TRANSPOSE, 0));
  g_prefs.end();
  g_srcSet = g_srcAddr != 0;
  g_tgtSet = g_tgtAddr != 0;
  return g_srcSet && g_tgtSet;
}
static void saveTranspose() {
  g_prefs.begin(NVS_NAMESPACE, false);
  g_prefs.putChar(NVS_KEY_TRANSPOSE, transpose::get());
  g_prefs.end();
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
  s->setInterval(BLE_SCAN_INTERVAL);
  s->setWindow(BLE_SCAN_WINDOW);
  s->setMaxResults(0);
  s->start(0, false, true);
}
static void stopScan() { NimBLEDevice::getScan()->stop(); }

// ---------------------------- MIDI forwarding -------------------------
// Producer: runs in the BLE host task. Copy the payload and enqueue — NO BLE
// writes here (writing unpaced from this context exhausts the mbuf pool ->
// ENOMEM -> the link drops) and NO parsing (raw passthrough can't fabricate
// or mis-parse anything). The pump task does the writing.
static void onSrcNotify(NimBLERemoteCharacteristic *c, uint8_t *data, size_t len, bool isNotify) {
  g_rxCount++;
  if (!g_midiQ || len == 0) return;

  RawFrame f;
  f.len = len > MIDI_RAW_MAX_BYTES ? MIDI_RAW_MAX_BYTES : (uint16_t)len;
  memcpy(f.data, data, f.len);
  if (xQueueSend(g_midiQ, &f, 0) != pdTRUE) g_dropCount++;   // queue full
}

// Queue All Notes Off (CC123) + All Sound Off (CC120) on every channel.
// Used when transpose changes so a note held across the switch can't hang.
static void enqueuePanic() {
  if (!g_midiQ) return;
  const uint16_t ts   = millis() & 0x1FFF;
  const uint8_t  hdr  = 0x80 | ((ts >> 7) & 0x3F);
  const uint8_t  tsLo = 0x80 | (ts & 0x7F);
  for (uint8_t cc : {(uint8_t)123, (uint8_t)120}) {
    for (uint8_t ch = 0; ch < 16; ++ch) {
      RawFrame f;
      f.len = 5;
      f.data[0] = hdr;
      f.data[1] = tsLo;
      f.data[2] = 0xB0 | ch;
      f.data[3] = cc;
      f.data[4] = 0x00;
      xQueueSend(g_midiQ, &f, 0);
    }
  }
}

// Consumer: runs the transform chain, then forwards each packet to the
// target verbatim, one write per packet, with backoff on a congested stack.
static void midiPumpTask(void *) {
  RawFrame f;
  for (;;) {
    if (xQueueReceive(g_midiQ, &f, pdMS_TO_TICKS(50)) != pdTRUE) continue;

#if DEBUG_LOG_TRANSFORMS
    uint8_t  preBuf[MIDI_RAW_MAX_BYTES];
    uint16_t preLen = f.len;
    memcpy(preBuf, f.data, f.len);
#endif
    g_xformCount += transforms::apply(f.data, f.len);   // in place; len unchanged
#if DEBUG_LOG_TRANSFORMS
    logTransform(preBuf, preLen, f.data, f.len);
#endif

    NimBLERemoteCharacteristic *ch = g_tgtChar;
    if (!ch || !g_tgtConn) { g_dropCount++; continue; }

    int tries = 0;
    for (;;) {
      ch = g_tgtChar;
      if (!ch || !g_tgtConn) { g_dropCount++; break; }
      if (ch->writeValue(f.data, f.len, false)) {
        g_fwdCount++;
        break;
      }
      if (++tries >= MIDI_WRITE_RETRIES) { g_dropCount++; break; }
      vTaskDelay(pdMS_TO_TICKS(MIDI_WRITE_RETRY_DELAY_MS));   // let the stack drain
    }

    const uint8_t n = f.len < sizeof(g_lastMsg) ? f.len : sizeof(g_lastMsg);
    memcpy(g_lastMsg, f.data, n);
    g_lastMsgLen = n;
    g_lastMsgAt  = millis();
  }
}

class SrcCliCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *) override { Serial.println("[MIDI-RT] SRC onConnect"); }
  void onDisconnect(NimBLEClient *, int reason) override {
    Serial.printf("[MIDI-RT] SRC onDisconnect reason=%d (0x%02X)\n", reason, reason & 0xFF);
    g_srcConn = false;
    g_linkLost = true;
  }
};
class TgtCliCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *) override { Serial.println("[MIDI-RT] TGT onConnect"); }
  void onDisconnect(NimBLEClient *, int reason) override {
    Serial.printf("[MIDI-RT] TGT onDisconnect reason=%d (0x%02X)\n", reason, reason & 0xFF);
    g_tgtConn = false;
    g_tgtChar = nullptr;
    g_linkLost = true;
  }
};
static SrcCliCB g_srcCliCB;
static TgtCliCB g_tgtCliCB;

static void fail(const char *msg) {
  ++g_failStreak;
  strncpy(g_failMsg, msg, sizeof(g_failMsg) - 1);
  Serial.printf("[MIDI-RT] FAIL: %s (streak %d)\n", msg, g_failStreak);
  g_state = State::Failed;
}

// Tear down ONE link and wait for it to actually go. deleteClient() on a
// still-connected client only *starts* an async disconnect; reconnecting
// before the client object is freed leaks a controller connection slot (only
// 3 exist) and wedges the stack until a full power cycle.
static void teardownLink(bool source) {
  if (source) { g_srcChar = nullptr; g_srcConn = false; }
  else        { g_tgtChar = nullptr; g_tgtConn = false; }

  NimBLEClient *&cli = source ? g_srcCli : g_tgtCli;
  if (!cli) return;

  const size_t before = NimBLEDevice::getCreatedClientCount();
  NimBLEDevice::deleteClient(cli);
  cli = nullptr;

  const uint32_t t0 = millis();
  while (NimBLEDevice::getCreatedClientCount() >= before && millis() - t0 < 2500)
    delay(20);
}
static void teardownLinks() {
  teardownLink(true);
  teardownLink(false);
}

static void configureRadio() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(BLE_TX_POWER_DBM);
  NimBLEDevice::setMTU(BLE_PREFERRED_MTU);
  NimBLEDevice::setSecurityAuth(BLE_BOND, BLE_MITM, BLE_SECURE_CONNECTIONS);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
}

// Nuclear recovery: fully restart the BLE stack. Frees any leaked connection
// slots on our side, and our disappearance lets the peers drop their stale
// links via supervision timeout.
static void bleRestart() {
  Serial.println("[MIDI-RT] BLE stack restart");
  teardownLinks();
  NimBLEDevice::deinit(true);
  delay(400);
  configureRadio();
  g_srcConn = g_tgtConn = false;
  g_srcChar = g_tgtChar = nullptr;
  g_failStreak = 0;
}

// Connect + discover the MIDI characteristic. Does NOT subscribe (source).
static bool connectOne(bool source) {
  const uint64_t a = source ? g_srcAddr : g_tgtAddr;
  const uint8_t  t = source ? g_srcType : g_tgtType;
  NimBLEAddress addr(a, t);

  NimBLEClient *cli = NimBLEDevice::createClient(addr);
  cli->setClientCallbacks(source ? (NimBLEClientCallbacks *)&g_srcCliCB
                                 : (NimBLEClientCallbacks *)&g_tgtCliCB,
                          false);
  cli->setConnectTimeout(BLE_CONNECT_TIMEOUT_MS);
  // Fast interval from the first connection event: two hops at NimBLE's
  // ~30-50 ms default would stack to 60-100 ms of round-trip latency.
  cli->setConnectionParams(BLE_CONN_INTERVAL_MIN, BLE_CONN_INTERVAL_MAX,
                           BLE_CONN_LATENCY, BLE_CONN_TIMEOUT_10MS);
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
  const size_t keep = source && g_tgtConn ? 1 : (!source && g_srcConn ? 1 : 0);
  for (int i = 1; i <= tries; ++i) {
    if (connectOne(source)) return true;
    Serial.printf("[MIDI-RT] %s connect attempt %d/%d failed\n",
                  source ? "SRC" : "TGT", i, tries);
    // let the failed/cancelled client object be freed before the next try
    const uint32_t t0 = millis();
    while (NimBLEDevice::getCreatedClientCount() > keep && millis() - t0 < 1500)
      delay(20);
    vTaskDelay(pdMS_TO_TICKS(800));
  }
  return false;
}

static void doConnect() {
  g_linkLost = false;
  if (g_midiQ) xQueueReset(g_midiQ);

  // Drop only the dead side(s); a healthy link stays up and is left alone.
  if (!g_tgtConn) teardownLink(false);
  if (!g_srcConn) teardownLink(true);

  // Target first (idle radio); subscribing to Source starts the notify flood.
  if (!g_tgtConn) {
    display::showStatus("Connecting to", "TARGET ...");
    Serial.println("[MIDI-RT] connecting TARGET...");
    if (!connectWithRetry(false, BLE_CONNECT_RETRIES)) { fail("TARGET connect failed"); return; }
    Serial.printf("[MIDI-RT] TARGET connected, MTU=%u\n", g_tgtCli ? g_tgtCli->getMTU() : 0);
  }

  if (!g_srcConn) {
    display::showStatus("Connecting to", "SOURCE ...");
    Serial.println("[MIDI-RT] connecting SOURCE...");
    if (!connectWithRetry(true, BLE_CONNECT_RETRIES)) { fail("SOURCE connect failed"); return; }
    if (!g_srcChar || !g_srcChar->canNotify() || !g_srcChar->subscribe(true, onSrcNotify)) {
      fail("SOURCE subscribe failed");
      return;
    }
    Serial.println("[MIDI-RT] SOURCE subscribed");
  }

  g_failStreak = 0;
  Serial.println("[MIDI-RT] -> ROUTING");
  g_rxCount = g_fwdCount = g_dropCount = g_xformCount = 0;
  g_lastMsgLen = 0;
  g_state = State::Routing;
}

// ------------------------------- scan list -----------------------------
// Snapshot g_found, filtering out the already-picked source when choosing
// the target. Shared by drawing and picking so both see the same ordering.
static std::vector<Found> snapshotFound(bool pickingSource) {
  portENTER_CRITICAL(&g_foundMux);
  std::vector<Found> list = g_found;
  portEXIT_CRITICAL(&g_foundMux);
  if (!pickingSource && g_srcSet) {
    list.erase(std::remove_if(list.begin(), list.end(),
                              [](const Found &f) { return f.addr == g_srcAddr; }),
               list.end());
  }
  return list;
}

static void drawScanList(bool pickingSource) {
  std::vector<Found> list = snapshotFound(pickingSource);
  const char *title = pickingSource ? "Pick SOURCE" : "Pick TARGET";
  if (list.empty()) {
    display::showScanning(title);
    return;
  }
  if (g_cursor >= (int)list.size()) g_cursor = 0;

  std::vector<display::DeviceRow> rows;
  rows.reserve(list.size());
  for (auto &f : list)
    rows.push_back({f.name, NimBLEAddress(f.addr, f.type).toString(), f.rssi});
  display::showScanList(title, rows, g_cursor);
}

static void drawRouting() {
  display::RoutingView v;
  v.srcConnected = g_srcConn;
  v.tgtConnected = g_tgtConn;
  v.srcAddress   = NimBLEAddress(g_srcAddr, g_srcType).toString();
  v.tgtAddress   = NimBLEAddress(g_tgtAddr, g_tgtType).toString();
  v.rx = g_rxCount;
  v.fwd = g_fwdCount;
  v.drop = g_dropCount;
  v.transpose = transpose::get();
  v.cc2pbHits = g_xformCount;
  v.lastMessage = g_lastMsg;
  v.lastMessageLen = g_lastMsgLen;
  v.lastMessageAgeMs = millis() - g_lastMsgAt;
  display::showRouting(v);
}

// ------------------------------- button ------------------------------
enum class Press { None, Short, Long };
static Press pollButton() {
  static bool wasDown = false;
  static uint32_t downAt = 0;
  const bool down = digitalRead(g_board->button) == LOW;
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

// Pick the cursor-th entry from the same filtered/ordered list drawScanList uses.
static bool pickAt(bool pickingSource, int idx, uint64_t &addr, uint8_t &type) {
  std::vector<Found> list = snapshotFound(pickingSource);
  if (idx < 0 || idx >= (int)list.size()) return false;
  addr = list[idx].addr;
  type = list[idx].type;
  return true;
}

static void resetForRescan() {
  teardownLinks();
  forgetPair();
  g_found.clear();
  g_cursor = 0;
  startScan();
  g_state = State::ScanSource;
}

// -------------------------------- setup ------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[MIDI-RT] boot");

  g_board = &detectBoard();
  Serial.printf("[MIDI-RT] board: %s\n", g_board->name);
  pinMode(g_board->button, INPUT_PULLUP);

  display::init(*g_board);
  configureRadio();

  g_midiQ = xQueueCreate(MIDI_QUEUE_DEPTH, sizeof(RawFrame));
  xTaskCreate(midiPumpTask, "midiPump", 8192, nullptr, 6, &g_pumpTask);

  const bool held = digitalRead(g_board->button) == LOW;
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
            Serial.printf("[MIDI-RT] source = %s\n", NimBLEAddress(a, t).toString().c_str());
          } else {
            g_tgtAddr = a; g_tgtType = t; g_tgtSet = true;
            Serial.printf("[MIDI-RT] target = %s\n", NimBLEAddress(a, t).toString().c_str());
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
#if ENABLE_TRANSPOSE
        transpose::cycle();
        saveTranspose();
        enqueuePanic();   // clear notes held across the change
        Serial.printf("[MIDI-RT] transpose = %+d (panic sent)\n", transpose::get());
#else
        g_rxCount = g_fwdCount = g_dropCount = g_xformCount = 0;
#endif
        lastDraw = 0;   // redraw immediately
      } else if (btn == Press::Long) {
        resetForRescan();
        break;
      }
      static uint32_t lastHb = 0;
      static uint32_t lastFwd = 0;
      if (millis() - lastHb > 2000) {
        lastHb = millis();
        if (g_fwdCount != lastFwd) {
          Serial.printf("[MIDI-RT] rx=%lu fwd=%lu drop=%lu xform=%lu xpose=%+d last=",
                        (unsigned long)g_rxCount, (unsigned long)g_fwdCount,
                        (unsigned long)g_dropCount, (unsigned long)g_xformCount,
                        transpose::get());
          for (int i = 0; i < g_lastMsgLen; ++i) Serial.printf("%02X ", g_lastMsg[i]);
          Serial.println();
          lastFwd = g_fwdCount;
        }
      }
      // routing screen is status-only; a slow redraw keeps the SPI bus (and
      // the radio) free for MIDI forwarding
      if (millis() - lastDraw >= DISPLAY_REDRAW_MS) {
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
        resetForRescan();
      } else if (millis() - failedAt > 4000) {
        failedAt = 0;
        if (g_failStreak >= BLE_RECONNECT_FAIL_STREAK_FOR_RESTART) bleRestart();
        Serial.println("[MIDI-RT] auto-retry after failure");
        g_state = State::Connecting;
      } else {
        display::showError(g_failMsg);
        delay(80);
      }
      break;
    }
  }
}
