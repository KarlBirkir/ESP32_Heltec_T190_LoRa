/*
 * ============================================================
 *  Heltec Vision Master T190 — LoRa RSSI Propagation Tester
 *  Same firmware on both boards. Each board is both TX and RX.
 *
 *  BUTTON MAPPING (portrait, USB at bottom):
 *    PRG = GPIO 0   — top-right corner
 *    USR = GPIO 21  — top-left corner
 *
 *  MAIN SCREEN BUTTONS:
 *    PRG short  — TX (or sync+TX if settings dirty)
 *    PRG long   — Toggle beacon mode
 *    USR short  — Enter settings
 *    USR long   — Force-apply pending settings locally (no sync)
 *
 *  SETTINGS SCREEN BUTTONS:
 *    PRG short  — Cycle selected value
 *    USR short  — Move highlight / exit on wrap
 *    USR long   — Roll back pending changes to current active settings
 *
 *  DISPLAY PINS (internal, default SPI):
 *    MOSI=48  SCLK=38  CS=39  DC=47  RST=40
 *    BL=17 (backlight, HIGH=on)
 *    GPIO 7 (TFT_VCTRL) must be LOW to power the display rail
 *
 *  LORA PINS (internal, HSPI):
 *    NSS=8  SCK=9  MOSI=10  MISO=11  RST=12  BUSY=13  DIO1=14
 *    Do NOT call pinMode on GPIO14 (JTAG MTCK)
 *
 *  NOTE on RTT:
 *    radio->transmit() blocks for the full on-air time, so
 *    txTimestamp is set after TX completes. RTT = inter-packet
 *    gap only, not including TX airtimes.
 *
 *  PROTOCOL:
 *    PKT_REQUEST      0x52  TX→RX  ping request
 *    PKT_RESPONSE     0x41  RX→TX  RSSI/SNR report
 *    PKT_SETTINGS     0x53  TX→RX  settings update
 *    PKT_SETTINGS_ACK 0x4B  RX→TX  settings acknowledged
 *
 *  RADIO STATE MACHINE:
 *    RADIO_IDLE
 *    RADIO_WAITING_SETTINGS_ACK  — PKT_SETTINGS sent, awaiting ACK
 *    RADIO_WAITING_RESPONSE      — PKT_REQUEST sent, awaiting response
 *    RADIO_BEACON_WAITING        — beacon mode, PKT_REQUEST in flight
 *    RADIO_BEACON_IDLE           — beacon mode, waiting for next interval
 * ============================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <RadioLib.h>
#include <Preferences.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// ── Display pins ─────────────────────────────────────────────
#define TFT_MOSI    48
#define TFT_SCLK    38
#define TFT_CS      39
#define TFT_DC      47
#define TFT_RST     40
#define TFT_BL      17
#define TFT_VCTRL    7

// ── LoRa pins (HSPI) ─────────────────────────────────────────
#define LORA_NSS     8
#define LORA_SCK     9
#define LORA_MOSI   10
#define LORA_MISO   11
#define LORA_RST    12
#define LORA_BUSY   13
#define LORA_DIO1   14   // do NOT call pinMode on this pin (JTAG MTCK)

// ── Buttons & LED ────────────────────────────────────────────
#define BTN_PRG      0   // top-right in portrait
#define BTN_USR     21   // top-left  in portrait
#define LED_PIN     35

// ── Display geometry ─────────────────────────────────────────
#define TFT_W      170
#define TFT_H      320

// ── Layout Y anchors ─────────────────────────────────────────
#define Y_TOPBAR        0   // top bar 14px
#define Y_CFGLINE      16   // config line — size-2 text, 22px tall
#define Y_DIV1         38   // subtle separator
#define Y_REM_ROW      41   // REMOTE bar row
#define Y_LOC_ROW      57   // LOCAL  bar row
#define Y_DIV2         73   // subtle separator
#define Y_HIST_HDR     76   // HISTORY header strip (teal bg)
#define Y_HIST_COL     89   // column labels
#define Y_HIST_ROWS   100   // 6 rows × 14px = 84px → ends at 184
#define Y_DIV3        185   // subtle separator
#define Y_WAITING     188   // status / waiting line
#define Y_HINT1       202   // bottom hint line 1
#define Y_HINT2       214   // bottom hint line 2
#define BAR_W          82   // width of RSSI bar
#define BAR_H          11   // height of RSSI bar
#define BAR_X           2   // left edge of labels
#define NUM_X         (BAR_X + BAR_W + 4 + 20)  // x for RSSI number after bar

// ── Colour palette (RGB565) ───────────────────────────────────
#define C_BG        0x0008   // near-black
#define C_PANEL     0x0810   // top bar background
#define C_WHITE     0xFFFF
#define C_GRAY      0x7BEF
#define C_DGRAY     0x39E7
#define C_CYAN      0x07FF   // SF value / static labels
#define C_GREEN     0x07E0   // freq value / good RSSI
#define C_ORANGE    0xFD20   // BW value / [TX] hint
#define C_RED       0xF800   // bad RSSI / [SYN] dirty / errors
#define C_YELLOW    0xFFE0   // medium RSSI
#define C_TEAL      0x0455   // history header strip
#define C_SELECT    0x2945   // settings highlight
#define C_MUTED     0x4A69   // history tag — mid blue-grey
#define C_SNR       0x867D   // SNR numbers — muted teal
#define C_SEP       0x2104   // subtle separator line colour
#define C_BARHINT   0x1082   // bar background

// ── Packet type bytes ─────────────────────────────────────────
#define PKT_REQUEST      0x52   // 'R'
#define PKT_RESPONSE     0x41   // 'A'
#define PKT_SETTINGS     0x53   // 'S'
#define PKT_SETTINGS_ACK 0x4B   // 'K'

// ── LoRa constants ────────────────────────────────────────────
#define TCXO_VOLTAGE   1.8f
#define TX_POWER_DBM   22
#define LORA_CR         8
#define LORA_PREAMBLE  12

// ── Selectable radio parameters ──────────────────────────────
const uint8_t SF_OPTIONS[]   = {10, 11, 12};
const float   FREQ_OPTIONS[] = {863.0f, 868.0f, 903.0f, 915.0f, 920.0f,
                                 928.0f, 940.0f, 950.0f, 960.0f};
const float   BW_OPTIONS[]   = {62.5f, 125.0f, 250.0f};

constexpr uint8_t SF_COUNT   = sizeof(SF_OPTIONS)   / sizeof(SF_OPTIONS[0]);
constexpr uint8_t FREQ_COUNT = sizeof(FREQ_OPTIONS) / sizeof(FREQ_OPTIONS[0]);
constexpr uint8_t BW_COUNT   = sizeof(BW_OPTIONS)   / sizeof(BW_OPTIONS[0]);

// ── History ───────────────────────────────────────────────────
#define HISTORY_SIZE 6

struct PingRecord {
    uint16_t seq;
    int16_t  remoteRSSI;
    int8_t   remoteSNR;
    int16_t  localRSSI;
    int8_t   localSNR;
    uint32_t roundTripMs;
    uint8_t  sfIdx;
    uint8_t  freqIdx;
    uint8_t  bwIdx;
    bool     valid;
    bool     lost;
};

// ── OTA packet structs (packed — no alignment padding) ────────
#pragma pack(push, 1)
struct RequestPacket {
    uint8_t  type;
    uint16_t seq;
    uint8_t  txPower;
    uint8_t  sf;
    uint8_t  bwIdx;
    uint8_t  freqIdx;
};

struct ResponsePacket {
    uint8_t  type;
    uint16_t seq;
    int16_t  rssiOfRequest;
    int8_t   snrOfRequest;
    uint32_t airTimeMs;
};

struct SettingsPacket {
    uint8_t  type;
    uint16_t seq;
    uint8_t  sfIdx;
    uint8_t  freqIdx;
    uint8_t  bwIdx;
};

struct SettingsAckPacket {
    uint8_t  type;
    uint16_t seq;
};
#pragma pack(pop)

// ── Radio state machine ───────────────────────────────────────
enum RadioState : uint8_t {
    RADIO_IDLE,
    RADIO_WAITING_SETTINGS_ACK,
    RADIO_WAITING_RESPONSE,
    RADIO_BEACON_WAITING,
    RADIO_BEACON_IDLE
};

// ── Hardware objects ──────────────────────────────────────────
Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
SPIClass        loraSPI(HSPI);
SX1262*         radio = nullptr;
Preferences     prefs;

// ── Active settings (what the radio is tuned to, remote knows) ─
uint8_t sfIdx   = 2;
uint8_t freqIdx = 1;
uint8_t bwIdx   = 1;

// ── Pending settings (user has dialled in, may differ) ────────
uint8_t pendingSfIdx   = 2;
uint8_t pendingFreqIdx = 1;
uint8_t pendingBwIdx   = 1;

// settingsDirty  — pending differs from active
// settingsForced — force-applied locally, remote may not match
bool settingsDirty  = false;
bool settingsForced = false;

// ── TX / RX state ─────────────────────────────────────────────
uint16_t   txSeq         = 0;
uint32_t   txTimestamp   = 0;
RadioState radioState    = RADIO_IDLE;
uint32_t   replyTimeout  = 15000;
uint8_t    settingsRetry = 0;
uint16_t   settingsSeq   = 0;

// ── Beacon ────────────────────────────────────────────────────
bool     beaconMode     = false;
uint32_t nextBeaconTime = 0;
uint32_t lastRTT        = 0;

// ── Long-press detection ──────────────────────────────────────
uint32_t prgDownAt    = 0;
bool     prgLongFired = false;
uint32_t usrDownAt    = 0;
bool     usrLongFired = false;

// ── History ring buffer ───────────────────────────────────────
PingRecord history[HISTORY_SIZE];
uint8_t    histHead = 0;

// ── UI state ──────────────────────────────────────────────────
uint8_t  currentScreen = 0;
uint8_t  settingsRow   = 0;

// ── Notification banner ───────────────────────────────────────
bool     notifActive = false;
uint32_t notifExpiry = 0;
char     notifText[28] = {};
uint16_t notifColor  = C_WHITE;

// ── Button edge detection ─────────────────────────────────────
uint8_t btnPrgPrev = HIGH;
uint8_t btnUsrPrev = HIGH;

bool radioReady = false;

// ── RadioLib DIO1 interrupt flag ──────────────────────────────
volatile bool rxFlag = false;
void IRAM_ATTR onDio1Rise() { rxFlag = true; }

// ============================================================
//  WDT feed
// ============================================================

void feedWDT() {
    TIMERG0.wdtwprotect.val = TIMG_WDT_WKEY_V;
    TIMERG0.wdtfeed.val     = 1;
    TIMERG0.wdtwprotect.val = 0;
    TIMERG1.wdtwprotect.val = TIMG_WDT_WKEY_V;
    TIMERG1.wdtfeed.val     = 1;
    TIMERG1.wdtwprotect.val = 0;
}

// ============================================================
//  NVS helpers
// ============================================================

void loadSettings() {
    prefs.begin("lora_cfg", true);
    sfIdx   = prefs.getUChar("sf",   2);
    freqIdx = prefs.getUChar("freq", 1);
    bwIdx   = prefs.getUChar("bw",   1);
    prefs.end();
    sfIdx   = min(sfIdx,   (uint8_t)(SF_COUNT   - 1));
    freqIdx = min(freqIdx, (uint8_t)(FREQ_COUNT - 1));
    bwIdx   = min(bwIdx,   (uint8_t)(BW_COUNT   - 1));
    pendingSfIdx   = sfIdx;
    pendingFreqIdx = freqIdx;
    pendingBwIdx   = bwIdx;
}

void saveSettings() {
    prefs.begin("lora_cfg", false);
    prefs.putUChar("sf",   sfIdx);
    prefs.putUChar("freq", freqIdx);
    prefs.putUChar("bw",   bwIdx);
    prefs.end();
}

// ============================================================
//  Accessors
// ============================================================

float   currentFreq() { return FREQ_OPTIONS[freqIdx]; }
float   currentBW()   { return BW_OPTIONS[bwIdx]; }
uint8_t currentSF()   { return SF_OPTIONS[sfIdx]; }

float   pendingFreq() { return FREQ_OPTIONS[pendingFreqIdx]; }
float   pendingBW()   { return BW_OPTIONS[pendingBwIdx]; }
uint8_t pendingSF()   { return SF_OPTIONS[pendingSfIdx]; }

void updateDirtyFlag() {
    settingsDirty = (pendingSfIdx   != sfIdx  ||
                     pendingFreqIdx != freqIdx ||
                     pendingBwIdx   != bwIdx);
}

// ============================================================
//  Air-time estimate (Semtech AN1200.13)
// ============================================================

uint32_t calcAirTimeMs(uint8_t payloadBytes) {
    float sf   = (float)currentSF();
    float bw   = currentBW() * 1000.0f;
    float ts   = (float)(1 << (int)sf) / bw;
    bool  ldro = (currentSF() >= 11 && currentBW() <= 125.0f)
              || (currentSF() >= 12 && currentBW() <= 250.0f);
    float de   = ldro ? 1.0f : 0.0f;
    float npre = (float)LORA_PREAMBLE + 4.25f;
    float arg  = (8.0f * (float)payloadBytes - 4.0f * sf + 28.0f)
                 / (4.0f * (sf - 2.0f * de));
    float npay = 8.0f + fmaxf(0.0f, ceilf(arg) * (float)(LORA_CR + 1));
    return (uint32_t)((npre + npay) * ts * 1000.0f);
}

// ============================================================
//  Radio control
// ============================================================

bool applyRadioSettings() {
    if (!radio) return false;
    int s = radio->setFrequency(currentFreq());
    if (s != RADIOLIB_ERR_NONE) return false;
    s = radio->setSpreadingFactor(currentSF());
    if (s != RADIOLIB_ERR_NONE) return false;
    s = radio->setBandwidth(currentBW());
    if (s != RADIOLIB_ERR_NONE) return false;
    return true;
}

bool initRadio() {
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    feedWDT();
    delay(50);
    radio = new SX1262(new Module(LORA_NSS, LORA_DIO1, LORA_RST,
                                  LORA_BUSY, loraSPI));
    feedWDT();
    int s = radio->begin(
        currentFreq(), currentBW(), currentSF(),
        LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
        TX_POWER_DBM, LORA_PREAMBLE, TCXO_VOLTAGE
    );
    if (s != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] begin() failed: %d\n", s);
        return false;
    }
    radio->setCRC(2);
    radio->setDio1Action(onDio1Rise);
    s = radio->startReceive();
    if (s != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] startReceive() failed: %d\n", s);
        return false;
    }
    Serial.printf("[LoRa] Ready — %.1f MHz  SF%u  BW%.0f kHz\n",
                  currentFreq(), currentSF(), currentBW());
    return true;
}

// FIX: standby() resets FIFO pointers, clearing stale TX data that
// could otherwise be read as a spurious RX event. rxFlag is also
// cleared to discard any interrupt that fired during transmit().
void startReceive() {
    if (!radio) return;
    radio->standby();
    rxFlag = false;
    int s = radio->startReceive();
    if (s != RADIOLIB_ERR_NONE)
        Serial.printf("[LoRa] startReceive error: %d\n", s);
}

// ============================================================
//  Drawing utilities
// ============================================================

void drawHBar(int16_t x, int16_t y, int16_t w, int16_t h,
              int pct, uint16_t fillCol, uint16_t bgCol) {
    tft.fillRect(x, y, w, h, bgCol);
    tft.drawRect(x, y, w, h, C_DGRAY);
    if (pct > 0) {
        int16_t fw = (int32_t)(w - 2) * pct / 100;
        tft.fillRect(x + 1, y + 1, fw, h - 2, fillCol);
    }
}

int rssiPct(int16_t rssiDbm) {
    return (int)constrain(map(rssiDbm, -140, -40, 0, 100), 0, 100);
}

uint16_t rssiColor(int16_t rssiDbm) {
    if (rssiDbm >= -90)  return C_GREEN;
    if (rssiDbm >= -110) return C_YELLOW;
    return C_RED;
}

void drawCentred(const char* txt, int16_t y, uint16_t col, uint8_t sz = 1) {
    tft.setTextSize(sz);
    tft.setTextColor(col, C_BG);
    int16_t x1, y1; uint16_t tw, th;
    tft.getTextBounds(txt, 0, y, &x1, &y1, &tw, &th);
    tft.setCursor((TFT_W - tw) / 2, y);
    tft.print(txt);
}

void drawBtnHint(uint8_t pos, const char* label, uint16_t col) {
    tft.setTextSize(1);
    tft.setTextColor(col, C_PANEL);
    if (pos == 0) {
        tft.setCursor(2, 2);
    } else {
        int16_t x1, y1; uint16_t tw, th;
        tft.getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
        tft.setCursor(TFT_W - tw - 2, 2);
    }
    tft.print(label);
}

// ============================================================
//  Notification banner — overlays top bar, blocks buttons
// ============================================================

void showNotif(const char* msg, uint16_t col, uint32_t durationMs = 3000) {
    strncpy(notifText, msg, sizeof(notifText) - 1);
    notifText[sizeof(notifText) - 1] = '\0';
    notifColor  = col;
    notifActive = true;
    notifExpiry = millis() + durationMs;
    tft.fillRect(0, Y_TOPBAR, TFT_W, 14, C_PANEL);
    tft.setTextSize(1);
    tft.setTextColor(col, C_PANEL);
    int16_t x1, y1; uint16_t tw, th;
    tft.getTextBounds(msg, 0, 2, &x1, &y1, &tw, &th);
    tft.setCursor((TFT_W - tw) / 2, 2);
    tft.print(msg);
}

void clearNotif() {
    notifActive = false;
    tft.fillRect(0, Y_TOPBAR, TFT_W, 14, C_PANEL);
    if (currentScreen == 0) {
        drawBtnHint(0, "[SET]", C_CYAN);
        if (beaconMode) {
            // drawn by updateBeaconHint()
        } else if (settingsDirty) {
            drawBtnHint(1, "[SYN]", C_RED);
        } else {
            drawBtnHint(1, "[TX]", C_ORANGE);
        }
    } else {
        drawBtnHint(0, "[BACK]", C_CYAN);
        drawBtnHint(1, "[CHG]",  C_ORANGE);
        tft.setTextColor(C_WHITE, C_PANEL);
        int16_t x1, y1; uint16_t tw, th;
        const char* s = "SETTINGS";
        tft.getTextBounds(s, 0, 2, &x1, &y1, &tw, &th);
        tft.setCursor((TFT_W - tw) / 2, 2);
        tft.print(s);
    }
}

void checkNotifExpiry() {
    if (notifActive && millis() >= notifExpiry) clearNotif();
}

// ============================================================
//  Top bar
// ============================================================

void drawMainTopBar() {
    tft.fillRect(0, Y_TOPBAR, TFT_W, 14, C_PANEL);
    if (notifActive) {
        tft.setTextColor(notifColor, C_PANEL);
        int16_t x1, y1; uint16_t tw, th;
        tft.getTextBounds(notifText, 0, 2, &x1, &y1, &tw, &th);
        tft.setCursor((TFT_W - tw) / 2, 2);
        tft.setTextSize(1);
        tft.print(notifText);
        return;
    }
    drawBtnHint(0, "[SET]", C_CYAN);
    if (beaconMode) {
        // drawn by updateBeaconHint()
    } else if (settingsDirty) {
        drawBtnHint(1, "[SYN]", C_RED);
    } else {
        drawBtnHint(1, "[TX]", C_ORANGE);
    }
}

void updateBeaconHint() {
    if (!beaconMode || notifActive) return;
    static uint32_t lastPulse = 0;
    static bool     pulseOn   = true;
    if (millis() - lastPulse < 500) return;
    lastPulse = millis();
    pulseOn   = !pulseOn;
    tft.fillRect(TFT_W / 2, 0, TFT_W / 2, 14, C_PANEL);
    drawBtnHint(1, "BEACON", pulseOn ? C_ORANGE : C_DGRAY);
}

// ============================================================
//  Config line — size-2, three coloured fields
// ============================================================

void drawCfgLine() {
    tft.fillRect(0, Y_CFGLINE, TFT_W, 22, C_BG);
    tft.setTextSize(2);

    // SF — left
    {
        char buf[10];
        uint16_t col;
        if (settingsDirty && pendingSfIdx != sfIdx) {
            snprintf(buf, sizeof(buf), "S%u>%u", currentSF(), pendingSF());
            col = C_RED;
        } else {
            snprintf(buf, sizeof(buf), "SF%u", currentSF());
            col = C_CYAN;
        }
        tft.setTextColor(col, C_BG);
        tft.setCursor(2, Y_CFGLINE);
        tft.print(buf);
    }

    // BW — centre
    {
        char buf[12];
        uint16_t col;
        if (settingsDirty && pendingBwIdx != bwIdx) {
            snprintf(buf, sizeof(buf), "%.0f>%.0f", currentBW(), pendingBW());
            col = C_RED;
        } else {
            snprintf(buf, sizeof(buf), "%.0fk", currentBW());
            col = C_ORANGE;
        }
        tft.setTextColor(col, C_BG);
        int16_t x1, y1; uint16_t tw, th;
        tft.getTextBounds(buf, 0, Y_CFGLINE, &x1, &y1, &tw, &th);
        tft.setCursor((TFT_W - tw) / 2, Y_CFGLINE);
        tft.print(buf);
    }

    // Freq — right
    {
        char buf[12];
        uint16_t col;
        if (settingsDirty && pendingFreqIdx != freqIdx) {
            snprintf(buf, sizeof(buf), "%.0f>%.0f", currentFreq(), pendingFreq());
            col = C_RED;
        } else {
            snprintf(buf, sizeof(buf), "%.0fM", currentFreq());
            col = C_GREEN;
        }
        tft.setTextColor(col, C_BG);
        int16_t x1, y1; uint16_t tw, th;
        tft.getTextBounds(buf, 0, Y_CFGLINE, &x1, &y1, &tw, &th);
        tft.setCursor(TFT_W - tw - 2, Y_CFGLINE);
        tft.print(buf);
    }
}

// ============================================================
//  Bottom hints — persistent info lines below history
// ============================================================

void drawBottomHints() {
    tft.fillRect(0, Y_HINT1, TFT_W, 24, C_BG);
    tft.setTextSize(1);

    // Line 1: always shown
    tft.setTextColor(C_DGRAY, C_BG);
    drawCentred("Long press [TX] to Beacon", Y_HINT1, C_DGRAY, 1);

    // Line 2: only when settings have been force-applied locally
    if (settingsForced) {
        drawCentred("Settings unsynced!", Y_HINT2, C_RED, 1);
    }
}

// ============================================================
//  Main screen static chrome
// ============================================================

void drawMainStatic() {
    tft.fillScreen(C_BG);
    drawMainTopBar();
    drawCfgLine();

    tft.drawFastHLine(0, Y_DIV1, TFT_W, C_SEP);

    tft.setTextSize(1);
    tft.setTextColor(C_GRAY, C_BG);
    tft.setCursor(BAR_X, Y_REM_ROW + 2); tft.print("REM");
    tft.setCursor(BAR_X, Y_LOC_ROW + 2); tft.print("LOC");

    tft.drawFastHLine(0, Y_DIV2, TFT_W, C_SEP);

    tft.fillRect(0, Y_HIST_HDR, TFT_W, 12, C_TEAL);
    tft.setTextColor(C_WHITE, C_TEAL); tft.setTextSize(1);
    tft.setCursor(2, Y_HIST_HDR + 2); tft.print("HISTORY");

    tft.setTextColor(C_DGRAY, C_BG); tft.setTextSize(1);
    tft.setCursor(2, Y_HIST_COL); tft.print("# RmRX LoRX  RTT  Config");

    tft.drawFastHLine(0, Y_DIV3, TFT_W, C_SEP);

    drawBottomHints();
}

// ============================================================
//  Live LAST PING section
// ============================================================

void updateMainLive(const PingRecord& r) {
    tft.fillRect(BAR_X + 20, Y_REM_ROW, TFT_W - BAR_X - 20, BAR_H, C_BG);
    tft.fillRect(BAR_X + 20, Y_LOC_ROW, TFT_W - BAR_X - 20, BAR_H, C_BG);

    if (!r.valid) {
        tft.setTextColor(C_DGRAY, C_BG); tft.setTextSize(1);
        tft.setCursor(BAR_X + 22, Y_REM_ROW + 2);
        tft.print("-- no data --");
        return;
    }

    char buf[16];

    // REMOTE row
    drawHBar(BAR_X + 20, Y_REM_ROW, BAR_W, BAR_H,
             rssiPct(r.remoteRSSI), rssiColor(r.remoteRSSI), C_BARHINT);
    tft.setTextSize(1);
    snprintf(buf, sizeof(buf), "%4d", r.remoteRSSI);
    tft.setTextColor(rssiColor(r.remoteRSSI), C_BG);
    tft.setCursor(NUM_X, Y_REM_ROW + 2); tft.print(buf);
    snprintf(buf, sizeof(buf), "%+d", r.remoteSNR);
    tft.setTextColor(C_SNR, C_BG);
    tft.setCursor(NUM_X + 30, Y_REM_ROW + 2); tft.print(buf);

    // LOCAL row
    drawHBar(BAR_X + 20, Y_LOC_ROW, BAR_W, BAR_H,
             rssiPct(r.localRSSI), rssiColor(r.localRSSI), C_BARHINT);
    snprintf(buf, sizeof(buf), "%4d", r.localRSSI);
    tft.setTextColor(rssiColor(r.localRSSI), C_BG);
    tft.setCursor(NUM_X, Y_LOC_ROW + 2); tft.print(buf);
    snprintf(buf, sizeof(buf), "%+d", r.localSNR);
    tft.setTextColor(C_SNR, C_BG);
    tft.setCursor(NUM_X + 30, Y_LOC_ROW + 2); tft.print(buf);
}

// ============================================================
//  History — 6 rows, newest first
// ============================================================

void fmtTag(char* buf, uint8_t sz, uint8_t si, uint8_t fi, uint8_t bi) {
    snprintf(buf, sz, "%u/%.0f/%.0f",
             SF_OPTIONS[si], BW_OPTIONS[bi], FREQ_OPTIONS[fi]);
}

void updateMainHistory() {
    tft.fillRect(0, Y_HIST_ROWS, TFT_W, HISTORY_SIZE * 14, C_BG);

    for (uint8_t i = 0; i < HISTORY_SIZE; i++) {
        uint8_t idx = (histHead + HISTORY_SIZE - 1 - i) % HISTORY_SIZE;
        const PingRecord& r = history[idx];
        int16_t y = Y_HIST_ROWS + (int16_t)i * 14;

        tft.setTextSize(1);
        tft.setCursor(2, y);

        if (!r.valid) {
            tft.setTextColor(C_DGRAY, C_BG);
            tft.print("---");
            continue;
        }

        char tag[14];
        fmtTag(tag, sizeof(tag), r.sfIdx, r.freqIdx, r.bwIdx);

        if (r.lost) {
            tft.setTextColor(C_RED, C_BG);
            char row[32];
            snprintf(row, sizeof(row), "%-2u LOST        %s",
                     r.seq % 100, tag);
            tft.print(row);
        } else {
            tft.setTextColor(C_WHITE, C_BG);
            char row[24];
            snprintf(row, sizeof(row), "%-2u %4d %4d %4u",
                     r.seq % 100, r.remoteRSSI, r.localRSSI,
                     (unsigned)min(r.roundTripMs, (uint32_t)9999));
            tft.print(row);
            int16_t x1, y1; uint16_t tw, th;
            tft.getTextBounds(tag, 0, y, &x1, &y1, &tw, &th);
            tft.setTextColor(C_MUTED, C_BG);
            tft.setCursor(TFT_W - tw - 2, y);
            tft.print(tag);
        }
    }
}

// ============================================================
//  Status line
// ============================================================

void updateStatusLine() {
    static uint32_t lastToggle = 0;
    static bool     visible    = false;
    if (millis() - lastToggle >= 500) {
        lastToggle = millis();
        visible    = !visible;
    }
    tft.fillRect(0, Y_WAITING, TFT_W, 10, C_BG);
    tft.setTextSize(1);
    switch (radioState) {
        case RADIO_WAITING_SETTINGS_ACK:
            if (visible) drawCentred("syncing settings...", Y_WAITING, C_CYAN, 1);
            break;
        case RADIO_WAITING_RESPONSE:
        case RADIO_BEACON_WAITING:
            if (visible) drawCentred("waiting for reply...", Y_WAITING, C_YELLOW, 1);
            break;
        case RADIO_BEACON_IDLE: {
            uint32_t now = millis();
            if (nextBeaconTime > now) {
                uint32_t rem = (nextBeaconTime - now) / 1000;
                char buf[28];
                snprintf(buf, sizeof(buf), "beacon in %us", (unsigned)rem);
                drawCentred(buf, Y_WAITING, C_DGRAY, 1);
            }
            break;
        }
        default: break;
    }
}

void clearHistory() {
    for (uint8_t i = 0; i < HISTORY_SIZE; i++) history[i] = {};
    histHead = 0;
}

// ============================================================
//  Settings screen
// ============================================================

// Draws current active settings as info line below top bar,
// matching the style of the main screen config line.
void drawSettingsInfoLine() {
    tft.fillRect(0, 18, TFT_W, 10, C_BG);
    tft.setTextSize(1);

    // SF — cyan, left
    char sfBuf[6];
    snprintf(sfBuf, sizeof(sfBuf), "SF%u", currentSF());
    tft.setTextColor(C_CYAN, C_BG);
    tft.setCursor(2, 18);
    tft.print(sfBuf);

    // BW — orange, centre
    char bwBuf[8];
    snprintf(bwBuf, sizeof(bwBuf), "%.0fk", currentBW());
    tft.setTextColor(C_ORANGE, C_BG);
    int16_t x1, y1; uint16_t tw, th;
    tft.getTextBounds(bwBuf, 0, 18, &x1, &y1, &tw, &th);
    tft.setCursor((TFT_W - tw) / 2, 18);
    tft.print(bwBuf);

    // Freq — green, right
    char frBuf[8];
    snprintf(frBuf, sizeof(frBuf), "%.0fM", currentFreq());
    tft.setTextColor(C_GREEN, C_BG);
    tft.getTextBounds(frBuf, 0, 18, &x1, &y1, &tw, &th);
    tft.setCursor(TFT_W - tw - 2, 18);
    tft.print(frBuf);
}

void drawSettingsValues() {
    static const int16_t rowY[3] = {44, 90, 136};

    for (uint8_t row = 0; row < 3; row++) {
        int16_t  y   = rowY[row];
        bool     sel = (row == settingsRow);
        uint16_t bg  = sel ? C_SELECT : C_BG;

        tft.fillRect(0, y, TFT_W, 34, bg);
        tft.drawRect(0, y, TFT_W, 34, sel ? C_CYAN : C_DGRAY);

        char val[16];
        if (row == 0)      snprintf(val, sizeof(val), "SF%u",     pendingSF());
        else if (row == 1) snprintf(val, sizeof(val), "%.1f MHz", pendingFreq());
        else               snprintf(val, sizeof(val), "%.0f kHz", pendingBW());

        tft.setTextSize(2);
        tft.setTextColor(sel ? C_CYAN : C_WHITE, bg);
        int16_t x1, y1; uint16_t tw, th;
        tft.getTextBounds(val, 0, y + 8, &x1, &y1, &tw, &th);
        tft.setCursor((TFT_W - tw) / 2, y + 8);
        tft.print(val);

        tft.setTextSize(1);
        tft.setTextColor(sel ? C_ORANGE : C_DGRAY, bg);
        tft.setCursor(4,           y + 12); tft.print("<");
        tft.setCursor(TFT_W - 10, y + 12); tft.print(">");
    }
}

void drawSettingsScreen() {
    tft.fillScreen(C_BG);
    tft.fillRect(0, 0, TFT_W, 14, C_PANEL);
    drawBtnHint(0, "[BACK]", C_CYAN);
    drawBtnHint(1, "[CHG]",  C_ORANGE);
    tft.setTextSize(1);
    tft.setTextColor(C_WHITE, C_PANEL);
    int16_t x1, y1; uint16_t tw, th;
    tft.getTextBounds("SETTINGS", 0, 2, &x1, &y1, &tw, &th);
    tft.setCursor((TFT_W - tw) / 2, 2);
    tft.print("SETTINGS");

    tft.drawFastHLine(0, 14, TFT_W, C_TEAL);

    // Info line: current active settings in matching colour style
    drawSettingsInfoLine();

    tft.drawFastHLine(0, 28, TFT_W, C_SEP);

    static const int16_t labelY[3] = {34, 80, 126};
    static const char*   labels[3] = {"Spreading Factor", "Frequency", "Bandwidth"};
    tft.setTextColor(C_GRAY, C_BG);
    for (uint8_t i = 0; i < 3; i++) {
        tft.setCursor(2, labelY[i]);
        tft.print(labels[i]);
    }

    // Bottom hints
    tft.drawFastHLine(0, TFT_H - 28, TFT_W, C_SEP);
    tft.setTextColor(C_DGRAY, C_BG); tft.setTextSize(1);
    tft.setCursor(2, TFT_H - 24);
    tft.print("Long press [BACK] to reset.");
    tft.setCursor(2, TFT_H - 12);
    tft.print("Settings update on next TX.");

    drawSettingsValues();
}

// ============================================================
//  Transmit helpers
// ============================================================

void ledFlash() {
    digitalWrite(LED_PIN, HIGH); delay(30); digitalWrite(LED_PIN, LOW);
}

bool sendSettingsPacket() {
    if (!radio) return false;
    SettingsPacket pkt;
    pkt.type    = PKT_SETTINGS;
    pkt.seq     = ++settingsSeq;
    pkt.sfIdx   = pendingSfIdx;
    pkt.freqIdx = pendingFreqIdx;
    pkt.bwIdx   = pendingBwIdx;

    applyRadioSettings();
    int s = radio->transmit((uint8_t*)&pkt, sizeof(pkt));
    txTimestamp = millis();

    if (s == RADIOLIB_ERR_NONE) {
        Serial.printf("[TX] SETTINGS seq=%u  SF%u  %.1fMHz  %.0fkHz\n",
                      pkt.seq, pendingSF(), pendingFreq(), pendingBW());
        ledFlash();
        startReceive();
        return true;
    }
    Serial.printf("[TX] SETTINGS failed: %d\n", s);
    startReceive();
    return false;
}

bool sendRequest() {
    if (!radio) return false;
    RequestPacket pkt;
    pkt.type    = PKT_REQUEST;
    pkt.seq     = ++txSeq;
    pkt.txPower = TX_POWER_DBM;
    pkt.sf      = sfIdx;
    pkt.bwIdx   = bwIdx;
    pkt.freqIdx = freqIdx;

    applyRadioSettings();
    int s = radio->transmit((uint8_t*)&pkt, sizeof(pkt));
    txTimestamp = millis();

    if (s == RADIOLIB_ERR_NONE) {
        Serial.printf("[TX] REQUEST seq=%u  SF%u  %.1fMHz  %.0fkHz\n",
                      pkt.seq, currentSF(), currentFreq(), currentBW());
        ledFlash();
        startReceive();
        return true;
    }
    Serial.printf("[TX] REQUEST failed: %d\n", s);
    startReceive();
    return false;
}

void doTxAction() {
    if (!radioReady) return;
    if (radioState != RADIO_IDLE && radioState != RADIO_BEACON_IDLE) return;

    if (settingsDirty) {
        settingsRetry = 0;
        if (sendSettingsPacket())
            radioState = RADIO_WAITING_SETTINGS_ACK;
    } else {
        if (sendRequest())
            radioState = beaconMode ? RADIO_BEACON_WAITING : RADIO_WAITING_RESPONSE;
    }
}

// Apply pending settings locally without syncing to remote.
// settingsForced stays true until a successful sync clears it.
void forceLocalSettings() {
    if (!settingsDirty) {
        showNotif("No pending changes", C_DGRAY);
        return;
    }
    sfIdx   = pendingSfIdx;
    freqIdx = pendingFreqIdx;
    bwIdx   = pendingBwIdx;
    settingsDirty  = false;
    settingsForced = true;
    replyTimeout   = calcAirTimeMs(sizeof(RequestPacket)) * 3 + 3000;
    saveSettings();
    applyRadioSettings();
    Serial.printf("[LOCAL] Settings forced: SF%u  %.1fMHz  %.0fkHz\n",
                  currentSF(), currentFreq(), currentBW());
    if (currentScreen == 0) {
        drawCfgLine();
        drawMainTopBar();
        drawBottomHints();
        showNotif("Applied locally only", C_YELLOW);
    }
}

// ============================================================
//  Handle incoming packet
// ============================================================

void handleRx() {
    uint8_t buf[64];
    size_t  len = sizeof(buf);
    int     s   = radio->readData(buf, len);

    if (s != RADIOLIB_ERR_NONE) {
        Serial.printf("[RX] readData error: %d\n", s);
        startReceive();
        return;
    }

    float rxRSSI = radio->getRSSI();
    float rxSNR  = radio->getSNR();
    Serial.printf("[RX] type=0x%02X  %u bytes  RSSI=%.0f  SNR=%.1f\n",
                  len > 0 ? buf[0] : 0, (unsigned)len, rxRSSI, rxSNR);

    if (len < 1) { startReceive(); return; }

    // ── PKT_SETTINGS ─────────────────────────────────────────
    if (buf[0] == PKT_SETTINGS && len >= sizeof(SettingsPacket)) {
        SettingsPacket pkt;
        memcpy(&pkt, buf, sizeof(pkt));
        Serial.printf("[RX] SETTINGS seq=%u  SF%u  freq%u  bw%u\n",
                      pkt.seq, SF_OPTIONS[pkt.sfIdx], pkt.freqIdx, pkt.bwIdx);

        delay(20);
        SettingsAckPacket ack;
        ack.type = PKT_SETTINGS_ACK;
        ack.seq  = pkt.seq;
        applyRadioSettings();
        s = radio->transmit((uint8_t*)&ack, sizeof(ack));
        if (s == RADIOLIB_ERR_NONE) {
            Serial.printf("[TX] SETTINGS_ACK seq=%u\n", ack.seq);
            ledFlash();
        } else {
            Serial.printf("[TX] SETTINGS_ACK failed: %d\n", s);
        }

        sfIdx   = min(pkt.sfIdx,   (uint8_t)(SF_COUNT   - 1));
        freqIdx = min(pkt.freqIdx, (uint8_t)(FREQ_COUNT - 1));
        bwIdx   = min(pkt.bwIdx,   (uint8_t)(BW_COUNT   - 1));
        pendingSfIdx   = sfIdx;
        pendingFreqIdx = freqIdx;
        pendingBwIdx   = bwIdx;
        settingsDirty  = false;
        settingsForced = false;
        replyTimeout   = calcAirTimeMs(sizeof(RequestPacket)) * 3 + 3000;
        saveSettings();
        applyRadioSettings();

        if (radioState == RADIO_WAITING_RESPONSE ||
            radioState == RADIO_BEACON_WAITING) {
            radioState = RADIO_IDLE;
            if (currentScreen == 0) {
                showNotif("Remote changed settings", C_CYAN);
                drawCfgLine();
                drawMainTopBar();
                drawBottomHints();
            }
        } else {
            if (currentScreen == 0) {
                drawCfgLine();
                drawMainTopBar();
                drawBottomHints();
            } else {
                drawSettingsInfoLine();
                drawSettingsValues();
            }
        }
        startReceive();
        return;
    }

    // ── PKT_SETTINGS_ACK ─────────────────────────────────────
    if (buf[0] == PKT_SETTINGS_ACK && len >= sizeof(SettingsAckPacket)) {
        if (radioState != RADIO_WAITING_SETTINGS_ACK) {
            startReceive(); return;
        }
        SettingsAckPacket ack;
        memcpy(&ack, buf, sizeof(ack));
        if (ack.seq != settingsSeq) {
            Serial.printf("[RX] Stale SETTINGS_ACK seq=%u expected=%u\n",
                          ack.seq, settingsSeq);
            startReceive(); return;
        }
        Serial.printf("[RX] SETTINGS_ACK seq=%u\n", ack.seq);

        sfIdx   = pendingSfIdx;
        freqIdx = pendingFreqIdx;
        bwIdx   = pendingBwIdx;
        settingsDirty  = false;
        settingsForced = false;
        replyTimeout   = calcAirTimeMs(sizeof(RequestPacket)) * 3 + 3000;
        saveSettings();
        applyRadioSettings();

        if (currentScreen == 0) {
            showNotif("Settings synced!", C_GREEN);
            drawCfgLine();
            drawMainTopBar();
            drawBottomHints();
            updateMainHistory();
        }

        radioState = RADIO_IDLE;
        if (sendRequest())
            radioState = beaconMode ? RADIO_BEACON_WAITING : RADIO_WAITING_RESPONSE;
        return;
    }

    // ── PKT_REQUEST ───────────────────────────────────────────
    if (buf[0] == PKT_REQUEST && len >= sizeof(RequestPacket)) {
        RequestPacket req;
        memcpy(&req, buf, sizeof(req));
        Serial.printf("[RX] REQUEST seq=%u\n", req.seq);

        delay(20);
        ResponsePacket resp;
        resp.type          = PKT_RESPONSE;
        resp.seq           = req.seq;
        resp.rssiOfRequest = (int16_t)rxRSSI;
        resp.snrOfRequest  = (int8_t)rxSNR;
        resp.airTimeMs     = calcAirTimeMs(sizeof(RequestPacket));

        applyRadioSettings();
        s = radio->transmit((uint8_t*)&resp, sizeof(resp));
        if (s == RADIOLIB_ERR_NONE) {
            Serial.printf("[TX] RESPONSE seq=%u\n", resp.seq);
            ledFlash();
        } else {
            Serial.printf("[TX] RESPONSE failed: %d\n", s);
        }
        startReceive();
        return;
    }

    // ── PKT_RESPONSE ──────────────────────────────────────────
    if (buf[0] == PKT_RESPONSE && len >= sizeof(ResponsePacket)) {
        if (radioState != RADIO_WAITING_RESPONSE &&
            radioState != RADIO_BEACON_WAITING) {
            startReceive(); return;
        }
        ResponsePacket resp;
        memcpy(&resp, buf, sizeof(resp));
        if (resp.seq != txSeq) {
            Serial.printf("[RX] Stale RESPONSE seq=%u expected=%u\n",
                          resp.seq, txSeq);
            startReceive(); return;
        }

        uint32_t rtt = millis() - txTimestamp;
        lastRTT = rtt;

        PingRecord r  = {};
        r.seq         = resp.seq;
        r.remoteRSSI  = resp.rssiOfRequest;
        r.remoteSNR   = resp.snrOfRequest;
        r.localRSSI   = (int16_t)rxRSSI;
        r.localSNR    = (int8_t)rxSNR;
        r.roundTripMs = rtt;
        r.sfIdx       = sfIdx;
        r.freqIdx     = freqIdx;
        r.bwIdx       = bwIdx;
        r.valid       = true;

        history[histHead] = r;
        histHead = (histHead + 1) % HISTORY_SIZE;

        Serial.printf("[PING] seq=%u  remRSSI=%d  locRSSI=%d  RTT=%u ms\n",
                      r.seq, r.remoteRSSI, r.localRSSI, (unsigned)rtt);

        if (currentScreen == 0) {
            updateMainLive(r);
            updateMainHistory();
        }

        if (beaconMode) {
            radioState     = RADIO_BEACON_IDLE;
            nextBeaconTime = millis() + 2 * rtt;
        } else {
            radioState = RADIO_IDLE;
        }
        startReceive();
        return;
    }

    Serial.printf("[RX] Unknown type 0x%02X\n", buf[0]);
    startReceive();
}

// ============================================================
//  Timeout handler
// ============================================================

void handleTimeouts() {
    if (radioState == RADIO_IDLE || radioState == RADIO_BEACON_IDLE) return;
    if (millis() - txTimestamp <= replyTimeout) return;

    if (radioState == RADIO_WAITING_SETTINGS_ACK) {
        settingsRetry++;
        if (settingsRetry < 3) {
            Serial.printf("[TIMEOUT] SETTINGS retry %u/3\n", settingsRetry);
            if (!sendSettingsPacket()) radioState = RADIO_IDLE;
        } else {
            Serial.println("[TIMEOUT] SETTINGS failed after 3 retries");
            radioState = RADIO_IDLE;
            if (currentScreen == 0) showNotif("Sync failed!", C_RED);
        }
        return;
    }

    // RADIO_WAITING_RESPONSE or RADIO_BEACON_WAITING
    Serial.printf("[TIMEOUT] REQUEST seq=%u\n", txSeq);

    PingRecord lost = {};
    lost.seq     = txSeq;
    lost.sfIdx   = sfIdx;
    lost.freqIdx = freqIdx;
    lost.bwIdx   = bwIdx;
    lost.valid   = true;
    lost.lost    = true;
    history[histHead] = lost;
    histHead = (histHead + 1) % HISTORY_SIZE;

    if (currentScreen == 0) {
        // FIX: clear live section so stale RSSI values don't persist
        PingRecord empty = {};
        updateMainLive(empty);
        updateMainHistory();
        tft.fillRect(0, Y_WAITING, TFT_W, 10, C_BG);
        drawCentred("TIMEOUT — no reply", Y_WAITING, C_RED, 1);
    }

    if (beaconMode) {
        radioState     = RADIO_BEACON_IDLE;
        nextBeaconTime = millis() + replyTimeout;
    } else {
        radioState = RADIO_IDLE;
    }
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
    feedWDT();
    Serial.begin(115200);
    while (!Serial && millis() < 2000) yield();
    Serial.println("\nHeltec T190 — LoRa RSSI Propagation Tester");

    loadSettings();
    replyTimeout = calcAirTimeMs(sizeof(RequestPacket)) * 3 + 3000;

    Serial.println("[SETUP] GPIO...");
    pinMode(TFT_VCTRL, OUTPUT);
    pinMode(TFT_BL,    OUTPUT);
    pinMode(LED_PIN,   OUTPUT);
    pinMode(BTN_PRG,   INPUT_PULLUP);
    pinMode(BTN_USR,   INPUT_PULLUP);
    digitalWrite(TFT_VCTRL, LOW);
    digitalWrite(TFT_BL,    HIGH);
    digitalWrite(LED_PIN,   LOW);

    feedWDT();
    Serial.println("[SETUP] Display...");
    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
    tft.init(TFT_W, TFT_H);
    tft.setRotation(0);
    tft.fillScreen(C_BG);

    drawCentred("LoRa RSSI Tester", 60, C_CYAN,  2);
    drawCentred("Heltec T190",       90, C_WHITE, 1);
    drawCentred("Initialising...",  120, C_DGRAY, 1);

    clearHistory();

    feedWDT();
    Serial.println("[SETUP] Radio...");
    radioReady = initRadio();

    if (radioReady) {
        drawCentred("Radio OK!", 140, C_GREEN, 1);
        char info[32];
        snprintf(info, sizeof(info), "SF%u  %.0fkHz  %.1fMHz",
                 currentSF(), currentBW(), currentFreq());
        drawCentred(info, 156, C_CYAN, 1);
    } else {
        drawCentred("Radio FAILED!",         140, C_RED,    1);
        drawCentred("Try TCXO 1.6 or 2.4 V", 156, C_ORANGE, 1);
    }

    uint32_t splashEnd = millis() + 1500;
    while (millis() < splashEnd) { feedWDT(); yield(); }

    drawMainStatic();
    updateMainLive(history[0]);
    updateMainHistory();
    Serial.println("[SETUP] Done.");
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
    // ── Radio RX ─────────────────────────────────────────────
    if (rxFlag) {
        rxFlag = false;
        if (radio) handleRx();
    }

    handleTimeouts();

    // ── Beacon ───────────────────────────────────────────────
    if (beaconMode && radioState == RADIO_BEACON_IDLE &&
        millis() >= nextBeaconTime) {
        doTxAction();
    }

    checkNotifExpiry();

    // ── Button reads ─────────────────────────────────────────
    uint8_t prgNow = digitalRead(BTN_PRG);
    uint8_t usrNow = digitalRead(BTN_USR);

    // ── PRG falling edge ─────────────────────────────────────
    if (prgNow == LOW && btnPrgPrev == HIGH) {
        prgDownAt    = millis();
        prgLongFired = false;
        if (notifActive) {
            clearNotif();
        } else if (currentScreen == 1) {
            // Settings: cycle value immediately on press
            switch (settingsRow) {
                case 0: pendingSfIdx   = (pendingSfIdx   + 1) % SF_COUNT;   break;
                case 1: pendingFreqIdx = (pendingFreqIdx + 1) % FREQ_COUNT; break;
                case 2: pendingBwIdx   = (pendingBwIdx   + 1) % BW_COUNT;   break;
            }
            updateDirtyFlag();
            drawSettingsValues();
        }
        // Main screen short press handled on release (to distinguish from long)
    }

    // ── PRG long press ────────────────────────────────────────
    if (prgNow == LOW && !prgLongFired &&
        millis() - prgDownAt >= 1500) {
        prgLongFired = true;
        if (currentScreen == 0 && !notifActive) {
            if (beaconMode) {
                beaconMode = false;
                radioState = RADIO_IDLE;
                showNotif("Beacon stopped", C_DGRAY);
                drawMainTopBar();
            } else if (radioState == RADIO_IDLE) {
                beaconMode = true;
                drawMainTopBar();
                doTxAction();
            }
        }
    }

    // ── PRG rising edge ───────────────────────────────────────
    if (prgNow == HIGH && btnPrgPrev == LOW) {
        if (!prgLongFired && !notifActive && currentScreen == 0) {
            if (radioState == RADIO_IDLE && radioReady && !beaconMode) {
                doTxAction();
            }
        }
    }

    // ── USR falling edge ─────────────────────────────────────
    if (usrNow == LOW && btnUsrPrev == HIGH) {
        usrDownAt    = millis();
        usrLongFired = false;
        if (notifActive) {
            clearNotif();
        } else if (currentScreen == 1) {
            // Settings: advance row or exit on wrap (short press on release)
        } else {
            // Main: enter settings (short press on release)
        }
    }

    // ── USR long press ────────────────────────────────────────
    if (usrNow == LOW && !usrLongFired &&
        millis() - usrDownAt >= 1500) {
        usrLongFired = true;
        if (!notifActive) {
            if (currentScreen == 0) {
                // Force-apply pending settings locally
                forceLocalSettings();
            } else if (currentScreen == 1) {
                // Roll back pending to current active
                pendingSfIdx   = sfIdx;
                pendingFreqIdx = freqIdx;
                pendingBwIdx   = bwIdx;
                updateDirtyFlag();
                drawSettingsValues();
                showNotif("Changes rolled back", C_CYAN);
            }
        }
    }

    // ── USR rising edge ───────────────────────────────────────
    if (usrNow == HIGH && btnUsrPrev == LOW) {
        if (!usrLongFired && !notifActive) {
            if (currentScreen == 0) {
                if (radioState == RADIO_IDLE && !beaconMode) {
                    currentScreen = 1;
                    drawSettingsScreen();
                }
            } else {
                // Settings: advance row or exit
                settingsRow++;
                if (settingsRow >= 3) {
                    settingsRow   = 0;
                    currentScreen = 0;
                    drawMainStatic();
                    uint8_t lastIdx = (histHead + HISTORY_SIZE - 1) % HISTORY_SIZE;
                    updateMainLive(history[lastIdx]);
                    updateMainHistory();
                } else {
                    drawSettingsValues();
                }
            }
        }
    }

    btnPrgPrev = prgNow;
    btnUsrPrev = usrNow;

    if (currentScreen == 0) {
        updateStatusLine();
        updateBeaconHint();
    }

    delay(15);
}
