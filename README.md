# Heltec T190 — LoRa RSSI Propagation Tester

Same firmware on both boards. Each acts as both transmitter and receiver.

---

## How it works

```
Board A presses PRG  →  sends REQUEST  (seq#, settings)
Board B receives     →  auto-sends RESPONSE (RSSI + SNR of request)
Board A receives     →  displays both RSSIs, SNR, round-trip time
```

**Packet types:**
- `0x52` ('R') — REQUEST: triggers a ping
- `0x41` ('A') — RESPONSE: carries the remote board's RSSI/SNR back

---

## Controls

| Button | Screen 0 (Main) | Screen 1 (Settings) |
|--------|----------------|---------------------|
| **PRG** (top-right) | Send REQUEST | Cycle selected value |
| **USR** (top-left)  | Go to Settings | Move highlight / exit on wrap |

In Settings: USR moves the highlight through SF → Freq → BW → back to Main.
PRG cycles the highlighted value forward through its options.

---

## Screen 0 — Main

```
┌──────────────────────────┐
│ [SET]      LoRa Tester [TX] │  ← button hints
│  SF12  125kHz  868.0MHz    │  ← live settings summary
├──────────────────────────┤
│ LAST PING                  │
│ Remote RX  [████░░] -95dBm │  ← their RSSI of our TX
│ Local  RX  [██░░░░] -110dBm│  ← our RSSI of their reply
│ Rem SNR: +8 dB             │
│ Loc SNR: +2 dB    450ms    │  ← round-trip time
├──────────────────────────┤
│ HISTORY                    │
│ #    RmRX  LcRX  RTT       │
│ #12   -95  -110  450ms     │
│ #11   -97  -108  448ms     │
│ #10   -98  -112  451ms     │
├──────────────────────────┤
│ AirTime: ~1647 ms/pkt      │
└──────────────────────────┘
```

Lost packets show as `#xx  LOST` in red in the history.

---

## Screen 1 — Settings

USR moves highlight, PRG cycles value.
Exiting (USR past BW row) returns to Main and applies settings.

| Parameter | Options |
|-----------|---------|
| Spreading Factor | SF10, SF11, SF12 |
| Frequency | 863, 868, 903, 915, 920, 928, 940, 950, 960 MHz |
| Bandwidth | 62.5, 125, 250 kHz |

**Both boards must be on identical settings to communicate.**

---

## LoRa Settings Reference

| Setting | Value | Notes |
|---------|-------|-------|
| Coding Rate | 4/8 | Max error correction |
| Preamble | 12 symbols | Slightly above default 8 |
| TX Power | +22 dBm | SX1262 maximum |
| CRC | 2 bytes | Enabled |
| TCXO | 1.8V | HT-RA62 spec |
| Sync word | Private (0x12) | Avoids public LoRaWAN collisions |

**Approximate air times (REQUEST packet):**

| SF | BW | Air time |
|----|----|----------|
| SF10 | 125 kHz | ~370 ms |
| SF11 | 125 kHz | ~700 ms |
| SF12 | 125 kHz | ~1400 ms |
| SF12 | 62.5 kHz | ~2800 ms |

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| "Radio FAILED!" on boot | Try changing `TCXO_VOLTAGE` to `1.6f` or `2.4f` |
| Display blank | Check `digitalWrite(TFT_VCTRL, LOW)` in setup() |
| No reply received | Confirm both boards on same SF / Freq / BW |
| USR button unresponsive | Confirm GPIO 27; try GPIO 26 if still broken |
| Seq numbers jumping | Normal — one board is receiving the other's reply-triggers |

---

## Quick-start context for new Claude conversations

```
Board: Heltec Vision Master T190
MCU: ESP32-S3R8 (8MB SiP flash, 8MB OPI PSRAM)

Working platformio.ini:
  board = heltec_wifi_lora_32_V3
  flash_size = 8MB, partitions = default_8MB.csv
  memory_type = qio_opi, psram_type = opi
  build_flags: -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1
               -DHELTEC_NO_DISPLAY=1 -DBOARD_HAS_PSRAM

Display: ST7789 170x320 — MOSI=48 SCLK=38 CS=39 DC=47 RST=40 BL=17
  GPIO 7 (VTFT Ctrl) LOW = display power on
  SPI.begin(38, -1, 48, 39)

LoRa: HT-RA62 (SX1262 HF) on separate HSPI
  NSS=8 SCK=9 MOSI=10 MISO=11 RST=12 BUSY=13 DIO1=14
  TCXO=1.8V, TX=+22dBm, lib=RadioLib

Buttons: PRG=GPIO0 (top-right), USR=GPIO27 (top-left) — active LOW
LED: GPIO35 active HIGH
```
