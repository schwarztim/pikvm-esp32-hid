# ESP32-S3 PiKVM HID Firmware — Verification Report

**Reviewed:** `/tmp/esp32-hid-keyboard/platformio.ini` and `/tmp/esp32-hid-keyboard/src/main.cpp`
**Spec:** `/Users/timothy.schwarz/Projects/kvm/PICO_HID_PROTOCOL.md` (kvmd master @ `6a5cbe7`, post-`v4.99`)
**Method:** Cross-referenced against Espressif `arduino-esp32` core (master), `hathach/tinyusb` upstream (master), Espressif ESP-IDF S3 GPIO docs, and PlatformIO N16R8 community configs.

---

## VERIFIED — Things the firmware does correctly

| # | Item | Evidence |
|---|------|----------|
| V1 | **VID/PID** `0x1209/0xEDA2` matches PiKVM HID spec §6 | `main.cpp:218-219` vs spec §6 line "VID `0x1209` (pid.codes), PID `0xEDA2` for HID" |
| V2 | **Composite descriptor strategy** (3 separate HID interfaces, IADs absent because composite via `bDeviceClass=0`) — matches Pico firmware which advertises 3 HID interfaces under `bDeviceClass=0` | `main.cpp:214` (`bDeviceClass=0`) vs `ph_usb.c` from PiKVM source (also `bDeviceClass=0`) |
| V3 | **Per-interface report descriptors omit Report-ID prefix** — required for boot protocol compliance | `main.cpp:233-268` (no `0x85, ID` byte). `tud_hid_n_report(ITF_KBD, 0, ...)` correctly passes `report_id=0` for non-ID descriptors |
| V4 | **Keyboard report layout (8 bytes)**: mods, reserved, 6 keys | `main.cpp:503-510` matches spec §6.1 |
| V5 | **Absolute mouse report layout (6 bytes)**: 1 button + LE u16 X + LE u16 Y + s8 wheel | `main.cpp:512-525` matches spec §6.2; coordinate mapping `(x + 32768) / 2` correctly squashes signed-int16 input range to descriptor's [0..32767] range, identical to Pico's `_mouse_abs_send_report` |
| V6 | **Relative mouse report layout (4 bytes)**: button + dx + dy + wheel | `main.cpp:527-535` matches spec §6.3 |
| V7 | **CRC-16/MODBUS** algorithm with poly 0xA001, init 0xFFFF, no final XOR | `main.cpp:486-499` byte-for-byte identical to spec §3.4 reference impl |
| V8 | **CRC wire layout big-endian** (high byte first) | `main.cpp:669-670` matches spec §3.4 `ph_split16` |
| V9 | **Boot protocol selection on each interface** — `HID_ITF_PROTOCOL_KEYBOARD` for keyboard, `HID_ITF_PROTOCOL_NONE` for absolute mouse, `HID_ITF_PROTOCOL_MOUSE` for relative mouse | `main.cpp:283-285` matches spec §6.1/6.2/6.3 |
| V10 | **Self-reset semantics** for SET_KBD/SET_MOUSE: ack with `PONG_OK` (RESET_REQUIRED bit set in subsequent pongs only via line 649), schedule restart 100ms later | `main.cpp:704-722, 816-820` matches spec §7.4 |
| V11 | **NVS persistence** of output modes (Pico uses watchdog scratch reg; ESP32-equivalent is NVS via `Preferences`) | `main.cpp:619-632` — correct adaptation |
| V12 | **REPEAT command** caches `prev_resp_code` and re-emits | `main.cpp:636-642` matches spec §4.2 |
| V13 | **PONG_OK status word construction** including RESET_REQUIRED, LED bits, OFFLINE bits, `OUT1_DYNAMIC` flag | `main.cpp:644-663` matches spec §5.1, §5.2 |
| V14 | **8-byte UART parser with 100ms inter-byte timeout** | `main.cpp:823-839` matches spec §7.2 (firmware uses no resync — accepts any 8 bytes as a frame, CRC catches misalignment, host recovers via REPEAT) |
| V15 | **MCU-code → USB-HID-usage table** (1..115) with modifier handling at 77..84 → 0xE0..0xE7 | `main.cpp:339-456` — verified against `kvmd/keymap.csv` ranges in spec §4.7.1 |
| V16 | **Modifier handling**: USB usage `0xE0..0xE7` → bit `1 << (usage & 0x07)` | `main.cpp:556-562` matches Pico's `ph_usb_kbd.c` snippet in spec §4.7.1 |
| V17 | **Mouse button select/state two-bit encoding** for main and extra bytes | `main.cpp:587-616` matches spec §4.9 |
| V18 | **MOUSE_WHEEL** ignores horizontal byte, uses byte 1 (vertical) | `main.cpp:756-758` matches spec §4.10 |
| V19 | **MOUSE_ABS coordinate decoding** — signed int16 big-endian | `main.cpp:742-743` matches spec §4.8 |
| V20 | **MOUSE_REL coordinate decoding** — signed int8 dx/dy | `main.cpp:765` matches spec §4.11 |
| V21 | **Serial number from MAC** | `main.cpp:794-797` matches spec §6.5 |
| V22 | **`tud_descriptor_*_cb` are weak in arduino-esp32** — overriding them in user code is the canonical pattern | Verified via [arduino-esp32 esp32-hal-tinyusb.c](https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-tinyusb.c) — both cbs declared `__attribute__((weak))`. User-defined non-weak versions correctly replace framework defaults. |
| V23 | **Pin choice GPIO16/GPIO17 for UART** is safe on ESP32-S3 with octal PSRAM (R8) — PSRAM uses GPIO33-37 only on the S3 (the GPIO16/17-PSRAM conflict is for the older ESP32-WROVER, not S3) | Verified via Espressif ESP-IDF GPIO docs — only GPIO26-32 (quad SPI flash/PSRAM) and GPIO33-37 (octal extension lines) are reserved on S3-R8 |
| V24 | **`ARDUINO_USB_MODE=0`** correctly selects USB-OTG (TinyUSB) instead of the default USB-Serial-JTAG | Verified — mode 0 = OTG, mode 1 = ROM USB-Serial/JTAG |
| V25 | **`ARDUINO_USB_CDC_ON_BOOT=0`** prevents Arduino from binding a default CDC-ACM interface that would conflict with our HID-only descriptor | Verified |
| V26 | **`CFG_TUD_HID=3`** allocates 3 HID instances in TinyUSB config | Correct for our 3-interface design |

---

## ISSUES — Things that are wrong or risky

### CRITICAL

**None found that block first-boot bring-up.** The firmware should enumerate as a 3-HID composite device on first power-on with the current code.

### HIGH

**H1. Self-reset path can be triggered before the SET_KBD/SET_MOUSE response physically leaves the UART**
- **Location:** `main.cpp:704-722, 816-820`
- **Problem:** On `CMD_SET_KBD`/`CMD_SET_MOUSE`, the firmware calls `send_response(PONG_OK)` (which writes to `Serial1`) and then sets `reset_scheduled_at = millis()`. After ≥100ms in `loop()`, `esp_restart()` fires. Arduino's `Serial1.write()` is buffered — the bytes may not have actually transmitted before `esp_restart()` resets the UART peripheral. kvmd will then see no response (or a partial response), retry via REPEAT, get `RESP_NONE` after reboot, and treat it as a stale-state error.
- **Fix:** Add `Serial1.flush()` after `send_response()` in `case CMD_SET_KBD` and `case CMD_SET_MOUSE`, OR insert `Serial1.flush()` immediately before `esp_restart()` at line 818.

  ```cpp
  // main.cpp:817 — replace
  if (millis() - reset_scheduled_at >= 100) {
      Serial1.flush();   // ← add this line
      esp_restart();
  }
  ```

**H2. `Serial1` (UART1) — verify no clash with the USB-CDC/JTAG reset path**
- **Location:** `main.cpp:806`
- **Problem:** When `ARDUINO_USB_CDC_ON_BOOT=0` is set, USB-OTG is owned by TinyUSB. But Arduino's `HardwareSerial` for `Serial1` initialises UART1 fine on GPIO16/17. **However**, on first flash you will not have a USB-CDC console for `Serial.print` debugging since it would require `ARDUINO_USB_CDC_ON_BOOT=1` and an additional CDC interface (more endpoints, more report IDs, larger TUD config — non-trivial). This is acceptable but means **the firmware has no debug-print channel** during bring-up other than the UART1-to-Pi link itself. If something is wrong with UART parsing you have no visibility.
- **Fix (optional, recommended for bring-up only):** Add a temporary GPIO debug LED toggle on packet receipt. Or wire a USB-TTL adapter to a separate UART (e.g., UART2 on GPIO15/GPIO18) for diagnostic logging. Do NOT enable `ARDUINO_USB_CDC_ON_BOOT=1` — it breaks the HID-only descriptor.

**H3. `desc_str[1+i] = str[i]` UTF-16LE conversion incomplete for non-ASCII chars**
- **Location:** `main.cpp:328-330`
- **Problem:** Direct byte-to-uint16 widening is correct only for 7-bit ASCII. The strings `"PiKVM"`, `"PiKVM HID"`, and the hex serial are ASCII so this works. Low actual risk but the pattern is fragile if anyone adds an em-dash or accented char to the product name.
- **Fix (optional):** This is the same pattern TinyUSB's own examples use; leave as-is unless internationalising.

### MEDIUM

**M1. `platformio.ini` is missing recommended PSRAM enable flags**
- **Location:** `platformio.ini`
- **Problem:** The canonical N16R8 config (per [sivar2311/ESP32-PlatformIO-Flash-and-PSRAM-configurations](https://github.com/sivar2311/ESP32-PlatformIO-Flash-and-PSRAM-configurations)) sets all of:
  ```
  board_build.flash_mode = qio
  board_build.psram_type = opi
  board_upload.maximum_size = 16777216
  build_flags = -DBOARD_HAS_PSRAM
  ```
  Our config sets `memory_type = qio_opi` (which encodes both) but skips the explicit per-component flags and `-DBOARD_HAS_PSRAM`. The firmware does not allocate from PSRAM, so functionally this works — but `ESP.getPsramSize()` returns 0 and runtime PSRAM diagnostics are blind.
- **Fix:** Append to `platformio.ini` build_flags and add explicit board fields:
  ```ini
  board_build.flash_mode = qio
  board_build.psram_type = opi
  board_upload.maximum_size = 16777216
  build_flags =
      -DBOARD_HAS_PSRAM
      ; ... existing flags ...
  ```

**M2. `default_16MB.csv` partition file not provided in repo**
- **Location:** `platformio.ini:10`
- **Problem:** `board_build.partitions = default_16MB.csv` references a file that must exist either in the project root or in the platform-espressif32 framework's `partitions/` directory. PlatformIO ships `default_16MB.csv` as part of platform-espressif32 v6+, so this should resolve — but if your installed platform version is older, the build will fail with "partition file not found".
- **Fix:** Either commit a `default_16MB.csv` to the project root (recommended), or pin `platform = espressif32 @ ^6.6.0` so the bundled partition file is guaranteed available.

**M3. Mouse offline-detection logic checks BOTH mouse interfaces but only one is active at a time**
- **Location:** `main.cpp:657-659`
  ```cpp
  if (!tud_hid_n_ready(ITF_ABS_MOUSE) && !tud_hid_n_ready(ITF_REL_MOUSE)) {
      resp[1] |= PONG_MOUSE_OFFLINE;
  }
  ```
- **Problem:** Reports "mouse offline" only if BOTH interfaces are unready. In practice on first enumeration, the host will mark both interfaces ready and this will not light up. But strictly per the spec, the ACTIVE mouse interface should drive the offline bit. If `active_mouse_mode == OUT1_MOUSE_USB_ABS` and only the abs interface is unready (e.g., its endpoint hasn't completed setup), kvmd will not see the offline indicator.
- **Fix:** Check only the active interface:
  ```cpp
  uint8_t mm = active_mouse_mode & OUT1_MOUSE_MASK;
  bool mouse_off = false;
  if (mm == OUT1_MOUSE_USB_ABS)      mouse_off = !tud_hid_n_ready(ITF_ABS_MOUSE);
  else if (mm == OUT1_MOUSE_USB_REL) mouse_off = !tud_hid_n_ready(ITF_REL_MOUSE);
  if (mouse_off) resp[1] |= PONG_MOUSE_OFFLINE;
  ```

**M4. Partial-frame timeout error response is sent at the wrong CRC position**
- **Location:** `main.cpp:836-839` — calls `send_response(RESP_TIMEOUT_ERROR)` (i.e., `0x48`)
- **Problem:** `send_response()` checks `code & PONG_OK` (high bit) at line 646. `0x48` does NOT have the high bit set, so the function correctly enters the "bare error" branch (line 664) and writes `resp[1] = 0x48`. ✓ This is actually fine.
- **Verdict:** No change needed. (Initially flagged on suspicion; correct on read.)

### LOW

**L1. `hid_report_descriptor[]` (lines 97-201) is dead code**
- **Location:** `main.cpp:97-201`
- **Problem:** This 105-byte combined descriptor with embedded `REPORT_ID` prefixes is never referenced. The actual descriptors used are the per-interface `desc_kbd_report` / `desc_abs_mouse_report` / `desc_rel_mouse_report` (lines 233-268).
- **Fix:** Delete lines 97-201 to reduce flash footprint by ~105 bytes and avoid confusion.

**L2. `CONFIG_TOTAL_LEN` macro is defined but never used**
- **Location:** `main.cpp:230`
- **Problem:** Cosmetic. The actual `wTotalLength` is computed inline at line 282. Macro is dead.
- **Fix:** Delete line 230, or change line 282 to use `CONFIG_TOTAL_LEN`.

**L3. `monitor_speed = 115200` may be unhelpful**
- **Location:** `platformio.ini:24`
- **Problem:** With `ARDUINO_USB_CDC_ON_BOOT=0` and TinyUSB owning the OTG port, `pio device monitor` will not see anything via USB. The 115200 baud rate matches our UART1 (Pi link) baud — but you can't easily monitor that without a UART-USB bridge.
- **Fix (optional):** Remove or change comment. Not functionally an issue.

**L4. `prev_resp_code` initialized to `RESP_NONE` (`0x24`) — this is correct per spec but worth a comment**
- **Location:** `main.cpp:476`
- **Problem:** Spec §5.1 says `0x24` = "REPEAT before any prior command" tells the host the MCU just rebooted and state is stale. The firmware will return this on the very first REPEAT after boot, which is correct behaviour and triggers a clean re-init on the host. Worth a `//` annotation so future maintainers don't "fix" it.
- **Fix:** Add a comment.

**L5. `case CMD_SET_CONNECTED` returns `PONG_OK` immediately (no-op)**
- **Location:** `main.cpp:725-726`
- **Problem:** Correct per spec §4.5 ("The Pico firmware accepts it but ignores the payload and just returns PONG_OK"). ✓
- **Verdict:** No change needed.

---

## SUGGESTIONS — Minor improvements

| # | Suggestion |
|---|------------|
| S1 | Add `Serial1.flush()` before `esp_restart()` (see H1) — this is the only meaningful correctness fix beyond the existing code. |
| S2 | Delete dead `hid_report_descriptor[]` and `CONFIG_TOTAL_LEN` to reduce flash and reader confusion (see L1, L2). |
| S3 | Make mouse-offline detection scope-aware to the active mouse mode (see M3). |
| S4 | Add `-DBOARD_HAS_PSRAM` for runtime visibility, even though PSRAM is unused (see M1). |
| S5 | Consider pinning `platform = espressif32 @ ^6.6.0` to guarantee `default_16MB.csv` is bundled (see M2). |
| S6 | Add a brief comment block at the top of `loop()` explaining that `tud_task()` is run by the Arduino USB FreeRTOS task (`xTaskCreate(usb_device_task, ...)` inside `tinyusb_init()`) — i.e., why we don't need to call it from `loop()`. Saves reader confusion. |
| S7 | The `mcu_to_usb` table is a hand-port. Consider adding a one-line comment noting "if kvmd's `keymap.csv` is regenerated upstream, regenerate this table via `python3 scripts/genmap.py < keymap.csv`" — even if the script doesn't exist, a TODO marker prevents drift. |
| S8 | `Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN)` works on Arduino-ESP32 v3.x; if pinning to v2.x, the API is identical but check `HardwareSerial.h` for `setPins()` deprecation messages. |
| S9 | Consider adding `Serial1.setRxBufferSize(64)` (or 128) before `Serial1.begin()` to ensure the default 256-byte RX FIFO is sufficient for back-to-back 8-byte packets at 115200. Default is fine but explicit is safer. |

---

## PINOUT VERIFICATION

| Pin | Use | Conflict Check | Verdict |
|-----|-----|----------------|---------|
| GPIO16 | UART1 RX (← Pi GPIO14 TX) | NOT used by S3 octal PSRAM (PSRAM is on GPIO33-37). The GPIO16/17-PSRAM conflict applies to the legacy ESP32-WROVER, NOT the ESP32-S3. | ✅ SAFE |
| GPIO17 | UART1 TX (→ Pi GPIO15 RX) | Same as GPIO16 — free on S3-R8. | ✅ SAFE |
| GPIO19 | USB-OTG D− (internal, fixed) | Reserved by USB-OTG when `ARDUINO_USB_MODE=0`. Do not use as GPIO. | ✅ ACTIVE (correct) |
| GPIO20 | USB-OTG D+ (internal, fixed) | Reserved by USB-OTG when `ARDUINO_USB_MODE=0`. Do not use as GPIO. | ✅ ACTIVE (correct) |
| GPIO0, 3, 45, 46 | Strapping pins | Not used by firmware | ✅ AVOIDED |
| GPIO26-32 | Quad SPI flash + PSRAM (always reserved on S3) | Not used by firmware | ✅ AVOIDED |
| GPIO33-37 | Octal PSRAM extension lines (R8 only) | Not used by firmware | ✅ AVOIDED |
| GPIO48 | Onboard RGB LED on most ESP32-S3-DevKitC-1 boards | Not used by firmware | ✅ FREE (available for debug if needed) |

**Conclusion:** Pin choices are safe and correct. **No conflicts with PSRAM, USB-OTG, strapping pins, or onboard peripherals on the canonical ESP32-S3-DevKitC-1-N16R8 layout.**

---

## REFERENCES

1. **PiKVM kvmd source — Pico HID firmware**: <https://github.com/pikvm/kvmd/tree/master/hid/pico/src>
2. **PiKVM kvmd source — host-side serial driver**: <https://github.com/pikvm/kvmd/tree/master/kvmd/plugins/hid>
3. **arduino-esp32 cores/esp32/esp32-hal-tinyusb.c** (proves `tud_descriptor_*_cb` are weak): <https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-tinyusb.c>
4. **arduino-esp32 cores/esp32/USB.cpp** (proves `USB.begin()` does hardware init beyond descriptors and is required): <https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/USB.cpp>
5. **arduino-esp32 USB examples directory**: <https://github.com/espressif/arduino-esp32/tree/master/libraries/USB/examples>
6. **arduino-esp32 USB API documentation** (VID/PID setters): <https://docs.espressif.com/projects/arduino-esp32/en/latest/api/usb.html>
7. **TinyUSB upstream — hid_composite example**: <https://github.com/hathach/tinyusb/blob/master/examples/device/hid_composite/src/usb_descriptors.c>
8. **TinyUSB upstream — `TUD_HID_DESCRIPTOR` / `TUD_HID_INOUT_DESCRIPTOR` macros**: <https://github.com/hathach/tinyusb/blob/master/src/device/usbd.h>
9. **Espressif ESP-IDF S3 GPIO docs** (proves GPIO16/17 free on S3, reserved range is 26-37 only): <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html>
10. **PlatformIO N16R8 community config (sivar2311)**: <https://github.com/sivar2311/ESP32-PlatformIO-Flash-and-PSRAM-configurations>
11. **PlatformIO N16R8 community discussion**: <https://community.platformio.org/t/new-esp32-s3-n16r8/41568>
12. **ESP32-S3 Datasheet (canonical pin reservations)**: <https://documentation.espressif.com/esp32-s3_datasheet_en.pdf>
13. **arduino-esp32 USB CDC/DFU tutorial** (proves `ARDUINO_USB_MODE` semantics): <https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/cdc_dfu_flash.html>
14. **Independent Rust port of PiKVM HID**: <https://github.com/jannic/pikvm-hid-stm32>

---

## SUMMARY

The firmware is **structurally sound and ready to flash for first bring-up**. There are **no CRITICAL issues** that would prevent enumeration or basic operation. The single most impactful fix is **H1 (add `Serial1.flush()` before `esp_restart()`)** — without it, SET_KBD/SET_MOUSE round-trips during a session-mode change may lose the ack and require kvmd to recover via REPEAT/timeout.

The proposed "bypass `USB.begin()` and call `tud_init()` directly" optimisation is **NOT necessary and would break hardware init**. The current pattern (override weak `tud_descriptor_*_cb` callbacks AND call `USB.begin()`) is the canonical Arduino-ESP32 pattern for fully-custom composite HID — verified against the framework source. `USB.begin()` performs USB PHY config, DM/DP pin routing, IRQ install, and FreeRTOS task creation; the user-provided weak overrides correctly replace the framework's default descriptors during enumeration.
