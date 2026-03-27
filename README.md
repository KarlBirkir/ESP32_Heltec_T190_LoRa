# Heltec T190 — LoRa RSSI Propagation Tester

Same firmware on both boards. Each acts as both transmitter and receiver.
Press the TX button on either board to ping the other and measure signal strength.

---

## How it works

```
Board A presses PRG  →  sends PKT_REQUEST  (seq#, current settings)
Board B receives     →  auto-sends PKT_RESPONSE (its RSSI + SNR of that request)
Board A receives     →  displays both RSSIs, SNRs, round-trip time
```

When settings differ from the remote, a sync handshake runs automatically
before the ping:

```
Board A presses PRG (settings dirty)
  →  sends PKT_SETTINGS on old settings
  →  Board B ACKs on old settings, then switches
  →  Board A switches, then immediately sends PKT_REQUEST on new settings
```

**Packet types:**
| Byte | Name | Direction | Purpose |
|------|------|-----------|---------|
| `0x52` | PKT_REQUEST | TX → RX | Ping request |
| `0x41` | PKT_RESPONSE | RX → TX | RSSI/SNR report |
| `0x53` | PKT_SETTINGS | TX → RX | Settings update |
| `0x4B` | PKT_SETTINGS_ACK | RX → TX | Settings acknowledged |

---

## Button reference

### Main screen

| Button | Press | Action |
|--------|-------|--------|
| **PRG** `[TX]` / `[SYN]` (top-right) | Short | Ping (or sync + ping if settings dirty) |
| **PRG** (top-right) | Long 1.5s | Toggle beacon mode |
| **USR** `[SET]` (top-left) | Short | Enter settings screen |
| **USR** (top-left) | Long 1.5s | Force-apply pending settings locally (no sync) |

`[TX]` shows in orange when settings match remote.
`[SYN]` shows in red when settings are dirty and a sync will run before the ping.
`BEACON` pulses orange/grey on the right when beacon mode is active.

### Settings screen

| Button | Press | Action |
|--------|-------|--------|
| **PRG** `[CHG]` (top-right) | Short | Cycle highlighted value forward |
| **USR** `[BACK]` (top-left) | Short | Move highlight down / exit on last row |
| **USR** (top-left) | Long 1.5s | Roll back pending changes to current active settings |

The info line below the top bar shows the current **active** settings
in the same colour scheme as the main screen (SF=cyan, BW=orange, Freq=green),
so you always know what's live while browsing pending changes.

---

## Main screen layout

```
┌──────────────────────────────┐
│ [SET]                   [TX] │  ← button hints / notification banner
│ SF12         125k       868M │  ← active settings (cyan / orange / green)
├──────────────────────────────┤
│ REM [████████░░] -95  +8     │  ← remote RSSI bar + dBm + SNR
│ LOC [██████░░░░] -108 +3     │  ← local  RSSI bar + dBm + SNR
├──────────────────────────────┤
│ HISTORY                      │
│ # RmRX LoRX  RTT  Config     │
│ 5  -95  -108 420  12/125/868 │  ← newest first
│ 4  -97  -110 418  10/125/868 │  ← config tag shows settings at time of ping
│ 3 LOST              12/125/868│  ← lost packets in red
│ 2  -91  -105 395  12/125/868 │
│ 1  -93  -107 412  12/125/868 │
│ ---                          │
├──────────────────────────────┤
│        syncing settings...   │  ← pulsing status line
│  Long press [TX] to Beacon.  │  ← persistent hint
│      Settings unsynced!      │  ← shown when force-applied locally
└──────────────────────────────┘
```

RSSI bars are colour-coded: green ≥ −90 dBm, yellow ≥ −110 dBm, red < −110 dBm.
History shows the settings active at the time of each measurement — useful for
comparing RSSI across different SF/BW/frequency combinations.

---

## Settings screen layout

```
┌──────────────────────────────┐
│ [BACK]    SETTINGS    [CHG]  │
│ SF12         125k       868M │  ← current active settings (read-only display)
├──────────────────────────────┤
│ Spreading Factor             │
│ ┌──────────────────────────┐ │
│ │ <        SF12           >│ │  ← highlighted row
│ └──────────────────────────┘ │
│ Frequency                    │
│ ┌──────────────────────────┐ │
│ │ <      868.0 MHz        >│ │
│ └──────────────────────────┘ │
│ Bandwidth                    │
│ ┌──────────────────────────┐ │
│ │ <       125 kHz         >│ │
│ └──────────────────────────┘ │
├──────────────────────────────┤
│ Long press [BACK] to reset.  │
│ Settings update on next TX.  │
└──────────────────────────────┘
```

Changes are pending until PRG is pressed on the main screen.
The sync handshake ensures both boards switch simultaneously.

---

## Beacon mode

Long press PRG on the main screen to start beacon mode. The board will
automatically ping repeatedly, waiting 2× the last RTT between each ping.
This lets you walk around and map coverage without pressing a button each time.

Long press PRG again to stop. Beacon stops automatically if you enter settings.

---

## Settings sync and force-local

**Normal sync (PRG short press with dirty settings):**
Sends PKT_SETTINGS to remote → waits for ACK → both boards switch → ping fires.
Retries up to 3 times (7s each) before showing "Sync failed!".

**Force-local (USR long press on main screen):**
Applies pending settings to this board only, without syncing.
"Settings unsynced!" appears at the bottom until a successful sync clears it.
Useful when the remote is out of range but you want to switch frequency to find it.

**Roll back (USR long press on settings screen):**
Resets pending values to match current active settings, discarding unsaved changes.

---

## Persistent settings

Settings (SF, Freq, BW) are saved to flash (NVS) after every successful sync.
Loaded at boot — the board wakes up on the last confirmed settings.

---

## LoRa radio reference

| Parameter | Value | Notes |
|-----------|-------|-------|
| Chip | SX1262 (HT-RA62 HF module) | 150–960 MHz capable |
| TX power | +22 dBm | SX1262 maximum |
| Coding rate | 4/8 | Maximum error correction |
| Preamble | 12 symbols | Slightly above default 8 |
| CRC | 2 bytes | Always enabled |
| TCXO voltage | 1.8 V | HT-RA62 spec |
| Sync word | 0x12 (private) | Avoids public LoRaWAN traffic |

**Approximate air times (per packet):**

| SF | BW | Air time |
|----|----|----------|
| SF10 | 125 kHz | ~370 ms |
| SF11 | 125 kHz | ~700 ms |
| SF12 | 125 kHz | ~1400 ms |
| SF12 | 62.5 kHz | ~2800 ms |

Note: the antenna on the T190 is tuned closer to 1020 MHz than 868 MHz.
Signal quality improves noticeably at higher frequencies (928–960 MHz).

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| "Radio FAILED!" on boot | Try `TCXO_VOLTAGE 1.6f` or `2.4f` in main.cpp |
| Display blank / no backlight | Check `digitalWrite(TFT_VCTRL, LOW)` in setup() |
| Board boot-loops | Hold PRG, plug USB, release PRG to enter bootloader, re-flash |
| No reply received | Confirm both boards on same SF / Freq / BW |
| Sync keeps failing | Try force-local on both boards to manually align settings |
| RSSI constant after link loss | Fixed — was stale FIFO data; standby() clears it now |

---

## GPIO notes (ESP32-S3R8 specific)

Some pins cannot be used freely due to internal connections:

| GPIO | Restriction | Reason |
|------|-------------|--------|
| 14 | No pinMode | JTAG MTCK — crashes on pulldown |
| 19, 20 | No general use | USB D+/D- |
| 21 | Use with care | USB JTAG TX — works as input when debugger not connected |
| 26–32 | Do not use | Internal flash/PSRAM — any pinMode crashes firmware |

---

## Quick-start context for new Claude conversations

```
Board: Heltec Vision Master T190 (HT-VMT190)
MCU: ESP32-S3R8 — 8MB SiP flash, 8MB OPI PSRAM

platformio.ini essentials:
  board = heltec_wifi_lora_32_V3
  flash_size = 8MB, partitions = default_8MB.csv
  memory_type = qio_opi, psram_type = opi
  build_flags: -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1
               -DHELTEC_NO_DISPLAY=1 -DBOARD_HAS_PSRAM

Display: ST7789 170x320 on default SPI
  MOSI=48 SCLK=38 CS=39 DC=47 RST=40 BL=17
  GPIO 7 (TFT_VCTRL) must be LOW to power display rail
  SPI.begin(38, -1, 48, 39) then tft.init(170, 320)

LoRa: HT-RA62 (SX1262 HF, 150–960 MHz) on HSPI
  NSS=8 SCK=9 MOSI=10 MISO=11 RST=12 BUSY=13 DIO1=14
  TCXO=1.8V, TX=+22dBm, CRC=2, lib=RadioLib
  Construct radio object INSIDE initRadio() after loraSPI.begin()
  Call radio->standby() before startReceive() to clear FIFO

Buttons: PRG=GPIO0 (top-right), USR=GPIO21 (top-left)
  Active LOW, INPUT_PULLUP. Do NOT pinMode GPIO14.
LED: GPIO35, active HIGH

WDT: feedWDT() via timer group registers (disableCore0/1WDT unreliable)
```
