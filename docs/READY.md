# ESP32-S3 PiKVM Pico HID — Ready for Test

**Build date:** 2026-04-25 14:55 EDT
**Build result:** SUCCESS (320KB flash, 31KB RAM)
**Platform:** espressif32 @ ^6.6.0, Arduino framework
**Board:** ESP32-S3-DevKitC-1 (N16R8: 16MB flash, 8MB PSRAM)
**Flashed to:** /dev/cu.usbmodem5B900311831

## Pin Assignments

| ESP32-S3 Pin | Function | Connects to |
|---|---|---|
| GPIO16 | UART1 RX | Pi GPIO14 (TX) |
| GPIO17 | UART1 TX | Pi GPIO15 (RX) |
| GND | Ground | Pi GND |
| GPIO19/20 | USB-OTG (D-/D+) | Target PC (via "USB" port on devkit) |

## Expected USB Enumeration

| Field | Value |
|---|---|
| VID | 0x1209 (pid.codes) |
| PID | 0xEDA2 |
| Manufacturer | PiKVM |
| Product | PiKVM HID |
| Serial | ESP32 MAC address (hex) |
| Interfaces | 3 HID: boot keyboard, absolute mouse, boot relative mouse |

## Test Commands

### 1. Verify USB HID enumeration (Mac)

Plug a USB-C cable from the ESP32-S3's **"USB" port** (not "UART") into the Mac, then run:

```
system_profiler SPUSBDataType | grep -B1 -A 12 -iE "0x1209|pikvm"
```

Expected output (approximate):

```
PiKVM HID:
  Product ID: 0xeda2
  Vendor ID: 0x1209
  Version: 1.00
  Serial Number: XXXXXXXXXXXX
  Speed: Up to 12 Mb/s
  Manufacturer: PiKVM
  Location ID: 0xXXXXXXXX
  Current Available (mA): 500
  Current Required (mA): 200
```

### 2. Verify HID interfaces (Mac)

```
ioreg -p IOUSB -w0 -l | grep -A 20 "PiKVM"
```

Should show 3 HID interface entries under the device.

### 3. UART protocol smoke test (optional)

With a USB-TTL adapter on GPIO16/17, send the PING packet and verify the response:

```
# PING request:  33 01 00 00 00 00 C1 C0
# Expected pong: 34 89 89 01 00 00 XX XX  (XX XX = CRC)
```

## Wiring Diagram (Pi 4 ↔ ESP32-S3 ↔ Target)

```
                    ┌─────────────────────────┐
                    │      Raspberry Pi 4      │
                    │                          │
                    │  Pin 8  (GPIO14 TX) ──────────┐
                    │  Pin 10 (GPIO15 RX) ─────────┐│
                    │  Pin 6  (GND)       ────────┐││
                    └─────────────────────────┘   │││
                                                  │││
                    ┌─────────────────────────┐   │││
                    │      ESP32-S3-N16R8      │   │││
                    │                          │   │││
                    │  GPIO16 (UART1 RX) ──────────┘│
                    │  GPIO17 (UART1 TX) ───────────┘
                    │  GND              ────────────┘
                    │                          │
  [To Mac for       │  "UART" port (USB-C) ◄───── USB cable to Mac
   reflashing only] │                          │
                    │  "USB"  port (USB-C) ◄───── USB cable to target
  [HID interface    │  (GPIO19/20 USB-OTG)     │   (Arch box)
   to target PC]    └─────────────────────────┘
```

## kvmd Configuration (Pi side)

In `/etc/kvmd/override.yaml`:

```yaml
kvmd:
    hid:
        type: serial
        device: /dev/kvmd-hid
        reset_pin: -1
        reset_self: true
        power_detect_pin: -1
```

Note: `reset_pin: -1` disables hardware reset (ESP32-S3 self-resets via
`esp_restart()` after SET_KBD/SET_MOUSE). `power_detect_pin: -1` disables
power detection (not wired).

## Protocol Implementation Summary

- 115200 8N1, no flow control
- 8-byte fixed packets, CRC-16/MODBUS (poly 0xA001, init 0xFFFF), big-endian
- All 11 opcodes implemented
- All 5 error codes implemented
- Full 116-entry MCU-to-USB keymap from kvmd/keymap.csv
- NVS persistence for keyboard/mouse mode across reboots
- Self-reboot with Serial1.flush() guarantee after SET_KBD/SET_MOUSE
