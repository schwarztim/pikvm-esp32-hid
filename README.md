# PiKVM ESP32-S3 HID

**Wire-compatible drop-in replacement for the [PiKVM Pico HID](https://github.com/pikvm/pico-hid-usb) using an ESP32-S3-N16R8.**

PiKVM v3+ uses a Raspberry Pi Pico as a USB HID gadget, connected to the host Pi via UART. The Pico runs firmware that translates serial commands into USB HID reports for keyboard, absolute mouse, and boot relative mouse. This project ports that firmware to the ESP32-S3, which has native USB-OTG and is widely available.

`kvmd` (the PiKVM daemon) cannot tell the difference between a real Pico HID and this firmware. Configure `kvmd` with `hid.type: serial` and it talks to the ESP32-S3 over UART exactly as it would talk to a Pico.

## Why ESP32-S3 instead of Pico?

- 16 MB flash + 8 MB PSRAM (vs. Pico's 2 MB flash, no PSRAM)
- Wi-Fi/BT (unused here, but available for future features)
- Often cheaper than RP2040 + breakout
- More readily available

## Hardware

| Component | Spec |
|---|---|
| MCU | ESP32-S3-N16R8 (ESP32-S3-DevKitC-1 or compatible) |
| USB-OTG port | "USB" port on devkit (GPIO19/20) — connects to target PC |
| Serial-JTAG port | "UART" port on devkit — connects to Mac/PC for flashing |
| UART RX | GPIO16 ← Pi GPIO14 (TX, pin 8) |
| UART TX | GPIO17 → Pi GPIO15 (RX, pin 10) |
| GND | GND ← Pi GND (pin 6) |

3.3 V CMOS levels on both sides — no level shifter needed.

## Wiring

```
                ┌─────────────────────────┐
                │   Raspberry Pi 4 (host) │
                │                          │
                │  Pin 8  (GPIO14, TX) ────────┐
                │  Pin 10 (GPIO15, RX) ───────┐│
                │  Pin 6  (GND)        ──────┐││
                └─────────────────────────┘  │││
                                             │││
                ┌─────────────────────────┐  │││
                │       ESP32-S3-N16R8     │  │││
                │                          │  │││
                │  GPIO16 (UART1 RX)  ─────────┘│
                │  GPIO17 (UART1 TX)  ──────────┘
                │  GND                ──────────┘
                │                          │
                │  "USB" port (USB-OTG) ◄──── USB cable to TARGET PC
                │  "UART" port          ◄──── USB cable to dev machine (re-flash only)
                └─────────────────────────┘
```

## USB descriptors

Composite HID device, VID `0x1209` / PID `0xEDA2` (`PiKVM` / `PiKVM HID`).

| Interface | Class | Notes |
|---|---|---|
| 0 | HID — boot keyboard | 8-bit modifier byte + 6-key rollover |
| 1 | HID — absolute mouse | 0..32767 X/Y range, scroll wheel, 3 buttons |
| 2 | HID — boot relative mouse | Standard relative mouse, 3 buttons |

Identical to the [pico-hid-usb composite descriptor](https://github.com/pikvm/kvmd/tree/master/hid/pico).

## Build & flash

Requires [PlatformIO Core](https://platformio.org/install/cli):

```bash
git clone https://github.com/schwarztim/pikvm-esp32-hid.git
cd pikvm-esp32-hid
pio run                                        # build
pio run --target upload                        # flash via "UART" port (USB-Serial-JTAG)
```

After flash, plug the **"USB" port** into a host PC. The host should see:

```
$ system_profiler SPUSBDataType | grep -A 8 -iE "0x1209|pikvm"
PiKVM HID:
  Product ID: 0xeda2
  Vendor ID: 0x1209
  ...
```

On Linux:

```
$ dmesg | tail
usb X-X: New USB device found, idVendor=1209, idProduct=eda2
input: PiKVM PiKVM HID as /devices/.../input/inputN
hid-generic ...: input,hidraw0: USB HID v1.11 Keyboard
hid-generic ...: input,hidraw1: USB HID v1.11 Mouse
```

## Configure kvmd to use this device

On the PiKVM host:

```yaml
# /etc/kvmd/override.yaml
kvmd:
    hid:
        type: serial
        device: /dev/kvmd-hid
        reset_pin: -1
        reset_self: true
```

`reset_self: true` makes kvmd issue a protocol-level reset over UART rather than yanking a physical RUN line — which the ESP32-S3 doesn't have wired by default.

## Protocol & verification

- [`docs/PICO_HID_PROTOCOL.md`](docs/PICO_HID_PROTOCOL.md) — byte-level UART protocol spec, CRC-16/MODBUS verification vector, opcode table, MCU-code → USB-HID-usage-code lookup table.
- [`docs/FIRMWARE_VERIFICATION.md`](docs/FIRMWARE_VERIFICATION.md) — cross-check report against ESP-IDF, Adafruit_TinyUSB_Arduino, and arduino-esp32 reference implementations.

## License

MIT — see [LICENSE](LICENSE).

## Acknowledgements

- [pikvm/pico-hid-usb](https://github.com/pikvm/pico-hid-usb) — protocol reference (GPLv3)
- [pikvm/kvmd](https://github.com/pikvm/kvmd) — host-side daemon
- [srepac/kvmd-armbian](https://github.com/srepac/kvmd-armbian) — kvmd installer for non-PiKVM-image hosts
