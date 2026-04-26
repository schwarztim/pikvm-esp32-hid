# PiKVM Pico HID — Wire Protocol Reference

**Target audience:** firmware author building an ESP32-S3 drop-in replacement for the PiKVM Pico HID, driven natively by `kvmd` over UART.

**Authoritative sources** (all current as of `kvmd` master @ commit `6a5cbe7`, post-tag `v4.99`, pulled 2026-04-25):

- Pico firmware C source: <https://github.com/pikvm/kvmd/tree/master/hid/pico/src>
- `kvmd` Python host driver: <https://github.com/pikvm/kvmd/tree/master/kvmd/plugins/hid>
- User-facing wiring docs: <https://github.com/pikvm/pikvm/blob/master/docs/pico_hid.md>
- Independent Rust port (corroborates protocol): <https://github.com/jannic/pikvm-hid-stm32>

All byte-level facts below are quoted from these sources. Where the user-facing handbook contradicts the firmware source, **the firmware source is canonical** — the daemon talks to the firmware, not to the docs.

---

## 0. TL;DR — The minimum a drop-in clone must implement

1. Listen on UART @ 115200 8N1, no flow control. RX = host TX, TX = host RX.
2. Read fixed 8-byte requests starting with magic `0x33`. Bytes [0..6) are header+payload; bytes [6..8) are big-endian CRC-16/MODBUS over bytes [0..6).
3. Reply with a fixed 8-byte response starting with magic `0x34`, plus big-endian CRC-16/MODBUS in bytes [6..8).
4. If 100 ms elapse mid-packet without completing 8 bytes, reset the parser and emit the timeout response (`0x48`).
5. Implement the 11 commands listed in §4 — at minimum `PING (0x01)`, `KBD_KEY (0x11)`, `MOUSE_ABS (0x12)`, `MOUSE_BUTTON (0x13)`, `MOUSE_WHEEL (0x14)`, `MOUSE_REL (0x15)`, `CLEAR_HID (0x10)`, `SET_KBD (0x03)`, `SET_MOUSE (0x04)`, `REPEAT (0x02)`, `SET_CONNECTED (0x05)`. Unknown commands return `0x45`.
6. Build the response status byte from the runtime state flags in §5 — kvmd parses bits, so wrong bits = wrong UI.
7. Expose USB HID with the exact descriptors quoted in §6 (Logitech-style 8-key boot keyboard + abs-or-rel mouse). VID `0x1209`, PID `0xEDA2` for HID; the bridge mode uses PID `0xEDA3` and a CDC-only descriptor — out of scope for this port.

---

## 1. UART parameters

From `kvmd/hid/pico/src/ph_com_uart.c` (lines 32–53):

```c
#define _BUS        uart1
#define _SPEED      115200
#define _RX_PIN     21
#define _TX_PIN     20
#define _TIMEOUT_US 100000

void ph_com_uart_init(void (*data_cb)(const u8 *), void (*timeout_cb)(void)) {
    _data_cb = data_cb;
    _timeout_cb = timeout_cb;
    uart_init(_BUS, _SPEED);
    gpio_set_function(_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(_TX_PIN, GPIO_FUNC_UART);
}
```

The Pico SDK's `uart_init(uart1, 115200)` defaults to **8 data bits, no parity, 1 stop bit**, no hardware flow control. The Pico firmware never reconfigures format or enables RTS/CTS. The host driver opens the port with `serial.Serial(path, 115200, timeout=read_timeout)` (defaults: 8N1, no flow control) — see `kvmd/plugins/hid/serial.py`:

```python
"speed":        Option(115200, type=valid_tty_speed),
"read_timeout": Option(2.0,    type=valid_float_f01),
```

| Parameter        | Value                                                              |
|------------------|--------------------------------------------------------------------|
| Baud             | **115200**                                                         |
| Data bits        | **8**                                                              |
| Parity           | **None**                                                           |
| Stop bits        | **1**                                                              |
| Flow control     | **None** (no RTS/CTS, no XON/XOFF)                                 |
| Inter-byte gap   | up to **100 ms**; longer triggers parser reset + `0x48` (TIMEOUT)  |
| Host read timeout| **2.0 s** (driver default; configurable in `override.yaml`)        |

Independent confirmation — `pikvm-hid-stm32/src/main.rs:132`: `serial::Config::default().baudrate(115_200.bps())`.

---

## 2. Pin assignments

### 2.1 Pi side (`kvmd` host)

`kvmd` opens whatever device is bound to the `/dev/kvmd-hid` udev symlink. From `configs/os/udev/v0-hdmi-rpi3.rules`:

```
KERNEL=="ttyAMA0", SYMLINK+="kvmd-hid"
```

So on the Pi the HID UART is **`ttyAMA0`** = the Pi's primary BCM PL011, exposed on the 40-pin header as:

| Pi 40-pin | BCM GPIO | Function | Direction |
|-----------|----------|----------|-----------|
| Pin 8     | GPIO14   | UART TX  | Pi→Pico   |
| Pin 10    | GPIO15   | UART RX  | Pico→Pi   |
| Pin 6 (etc.) | GND   | GND      | shared    |

Reset and power-detect are GPIO sideband, configured in `/etc/kvmd/override.yaml`. Defaults from `kvmd/plugins/hid/_mcu/__init__.py`:

```python
"reset_pin":              Option(4,     type=valid_gpio_pin_optional),
"reset_inverted":         Option(False, type=valid_bool),
"reset_delay":            Option(0.1,   type=valid_float_f01),
"power_detect_pin":       Option(-1,    type=valid_gpio_pin_optional),
"power_detect_pull_down": Option(False, type=valid_bool),
```

The official UART config from `docs/pico_hid.md` overrides these to:

```yaml
kvmd:
    hid:
        type: serial
        device: /dev/kvmd-hid
        reset_pin: 25            # Pi BCM GPIO25
        reset_inverted: true
        reset_self: true
        power_detect_pin: 16     # Pi BCM GPIO16
        power_detect_pull_down: true
```

`reset_self: true` means kvmd does **not** drive a hardware reset line — it expects the MCU to reset itself in response to `SET_KBD` / `SET_MOUSE` and signal that with the `RESET_REQUIRED` flag. The Pico does this via the watchdog (`watchdog_reboot(0, 0, 100)` in `main.c` after a successful response). For the ESP32-S3 port, replicate this: after acking a `SET_KBD`/`SET_MOUSE`, schedule a self-reboot ~100 ms later. `power_detect_pin` is read by `kvmd` only — the firmware doesn't see it.

### 2.2 Pico/MCU side — UART mode

**Authoritative pin numbers come from `ph_com_uart.c`, NOT from the handbook.** The handbook page <https://docs.pikvm.org/pico_hid/> lists Pico GP0/GP1; that is **wrong** — those are the Pico SDK's default `uart0` debug pins, used by `ph_debug.c` (a debug-only print channel), not by the protocol UART.

The actual protocol UART is on **Pico `uart1` instance, GP20 (TX) and GP21 (RX)**:

| Pico GPx | RP2040 alt fn  | Wired to                  | Direction       |
|----------|----------------|---------------------------|-----------------|
| GP20     | UART1 TX       | Pi GPIO15 (UART RX)       | Pico→Pi         |
| GP21     | UART1 RX       | Pi GPIO14 (UART TX)       | Pi→Pico         |
| GND      | —              | Pi GND                    | shared          |
| VSYS     | 5V in (via 1N5819 diode) | Pi 5V              | power           |
| RUN      | reset (active low) | Pi GPIO25 (config)    | Pi→Pico (optional, `reset_self=true` makes this unused) |

**[AMBIGUOUS]** The handbook diagram for "classic Serial (UART)" shows Pi TX→Pico GP1 and Pi RX→Pico GP0. The firmware source unambiguously uses GP20/GP21 for `uart1`. The handbook is a documentation drift — confirm experimentally on first bring-up by toggling GP20/GP21 and watching with a logic analyzer. The firmware source is authoritative for any new build that flashes the current `pico-hid.uf2`.

### 2.3 Pico/MCU side — UART vs SPI mode select

From `ph_com.c` (lines 35–58):

```c
#define _USE_SPI_PIN 22

void ph_com_init(...) {
    gpio_init(_USE_SPI_PIN);
    gpio_set_dir(_USE_SPI_PIN, GPIO_IN);
    gpio_pull_up(_USE_SPI_PIN);
    sleep_ms(10);
    _use_spi = gpio_get(_USE_SPI_PIN);   // floating/high → SPI; tied to GND → UART
    _COM(init, data_cb, timeout_cb);
}
```

| GP22 state at boot | Mode |
|--------------------|------|
| floating (pulled high internally)  | **SPI** (default) |
| tied to GND                        | **UART**          |

For an ESP32-S3 UART-only replacement: ignore SPI entirely, always boot in UART mode. You may either tie an analogous pin low and read it, or just hard-code UART.

### 2.4 Pico/MCU side — runtime mode-selection jumpers

From `ph_outputs.c` (lines 34–44, 54–116). All are pulled up internally and active when **tied to GND**:

| Pico GPx | Macro                       | Effect when GND'd                              |
|----------|-----------------------------|------------------------------------------------|
| GP2      | `_PS2_ENABLED_PIN`          | Enable PS/2 keyboard & mouse stack             |
| GP3      | `_PS2_SET_KBD_PIN`          | Prefer PS/2 keyboard (if PS/2 enabled)         |
| GP4      | `_PS2_SET_MOUSE_PIN`        | Prefer PS/2 mouse (if PS/2 enabled)            |
| GP5      | `_BRIDGE_MODE_PIN`          | Enter "USB CDC bridge" mode (different VID/PID, no UART/SPI) |
| GP6      | `_USB_DISABLED_PIN`         | Disable USB HID entirely                       |
| GP7      | `_USB_ENABLE_W98_PIN`       | Advertise the Win98-friendly absolute-mouse mode |
| GP8      | `_USB_SET_MOUSE_REL_PIN`    | Prefer relative USB mouse over absolute        |
| GP9      | `_USB_SET_MOUSE_W98_PIN`    | Prefer the Win98 absolute mouse                |

For an ESP32 port, you can pick any equivalent GPIOs (or hard-code the desired output mode) — the host doesn't enforce these; they only seed the firmware's initial `outputs1` byte (see §5).

### 2.5 Pico/MCU side — PS/2 wiring (only relevant if implementing PS/2 emulation)

From `docs/pico_hid.md`:

| Pico GPx | Function                  |
|----------|---------------------------|
| GP11     | PS/2 keyboard data        |
| GP12     | PS/2 keyboard clock       |
| GP13     | PS/2 5V supply (optional, switched output) |
| GP14     | PS/2 mouse data           |
| GP15     | PS/2 mouse clock          |
| GP26     | (optional) PS/2 keyboard passthrough data  |
| GP27     | (optional) PS/2 keyboard passthrough clock |
| GP16     | (optional) PS/2 mouse passthrough data     |
| GP17     | (optional) PS/2 mouse passthrough clock    |

Out of scope for a USB-HID-only ESP32 replacement.

---

## 3. Packet framing

### 3.1 Geometry

- Every request and every response is **exactly 8 bytes**. Variable-length frames do not exist.
- Big-endian for any multi-byte integer (CRC, mouse coords).
- The wire is half-duplex command/response: host writes 8 bytes, MCU writes 8 bytes back. There is no "MCU-initiated" frame and no streaming.

### 3.2 Request frame (Pi→Pico)

| Offset | Size | Field        | Value/notes                                        |
|--------|------|--------------|----------------------------------------------------|
| 0      | 1    | `MAGIC_REQ`  | **`0x33`** — `PH_PROTO_MAGIC`                      |
| 1      | 1    | `CMD`        | command opcode (see §4)                            |
| 2..6   | 4    | `PAYLOAD`    | command-specific; pad unused bytes with `0x00`     |
| 6..8   | 2    | `CRC16`      | CRC-16/MODBUS over bytes [0..6), big-endian (`high, low`) |

Built by `kvmd/plugins/hid/_mcu/proto.py`:

```python
def _make_request(cmd: bytes) -> bytes:
    assert len(cmd) == 5, cmd
    req = b"\x33" + cmd
    req += struct.pack(">H", bitbang.make_crc16(req))
    assert len(req) == 8, req
    return req
```

### 3.3 Response frame (Pico→Pi)

The current firmware always emits an 8-byte response. The host driver also accepts a legacy 4-byte response (used by older Arduino HIDs) when byte[0] is `0x33`; the Pico firmware uses byte[0] = `0x34`.

| Offset | Size | Field        | Value/notes                                          |
|--------|------|--------------|------------------------------------------------------|
| 0      | 1    | `MAGIC_RESP` | **`0x34`** — `PH_PROTO_MAGIC_RESP`                   |
| 1      | 1    | `STATUS`     | response code; high bit set = "pong-with-state" (see §5) |
| 2      | 1    | `OUTPUTS1`   | active outputs: kbd type, mouse type, dynamic flag (see §5) |
| 3      | 1    | `OUTPUTS2`   | available outputs + connect state (see §5)           |
| 4..6   | 2    | reserved     | always `0x00 0x00` in current firmware               |
| 6..8   | 2    | `CRC16`      | CRC-16/MODBUS over bytes [0..6), big-endian          |

Built by `hid/pico/src/main.c` (`_send_response`):

```c
u8 resp[8] = {0};
resp[0] = PH_PROTO_MAGIC_RESP;       // 0x34
if (code & PH_PROTO_PONG_OK) {       // 0x80
    resp[1] = PH_PROTO_PONG_OK;
    if (_reset_required) {
        resp[1] |= PH_PROTO_PONG_RESET_REQUIRED;  // 0x40
    }
    resp[2] = PH_PROTO_OUT1_DYNAMIC;             // 0x80
    resp[1] |= ph_cmd_get_offlines();
    resp[1] |= ph_cmd_kbd_get_leds();
    resp[2] |= ph_g_outputs_active;
    resp[3] |= ph_g_outputs_avail;
} else {
    resp[1] = code;                  // bare error code
}
ph_split16(ph_crc16(resp, 6), &resp[6], &resp[7]);
ph_com_write(resp);
```

### 3.4 CRC-16

**Algorithm: CRC-16/MODBUS** — also called CRC-16-IBM or "ARC-reflected".

Parameters:
- Polynomial: **`0xA001`** (reversed `0x8005`)
- Initial value: **`0xFFFF`**
- Input reflected: yes (LSB-first inside each byte)
- Output reflected: yes
- Final XOR: **`0x0000`**
- Width: 16 bits
- Tx order on the wire: **big-endian** (`high_byte, low_byte`) — note this is the *byte order on the wire*, not the bit order

Reference implementation (firmware) — `hid/pico/src/ph_tools.h`:

```c
inline u16 ph_crc16(const u8 *buf, uz len) {
    const u16 polinom = 0xA001;
    u16 crc = 0xFFFF;
    for (uz byte_count = 0; byte_count < len; ++byte_count) {
        crc = crc ^ buf[byte_count];
        for (uz bit_count = 0; bit_count < 8; ++bit_count) {
            if ((crc & 0x0001) == 0) {
                crc = crc >> 1;
            } else {
                crc = crc >> 1;
                crc = crc ^ polinom;
            }
        }
    }
    return crc;
}
```

Reference implementation (host) — `kvmd/bitbang.py`:

```python
def make_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = crc ^ byte
        for _ in range(8):
            if crc & 0x0001 == 0:
                crc = crc >> 1
            else:
                crc = crc >> 1
                crc = crc ^ 0xA001
    return crc
```

Wire layout of the CRC field (bytes 6 and 7) — big-endian:

```c
inline void ph_split16(u16 from, u8 *to_a, u8 *to_b) {
    *to_a = (u8)(from >> 8);   // high byte
    *to_b = (u8)(from & 0xFF); // low byte
}
```

**Independent confirmation** — `pikvm-hid-stm32/src/proto.rs:197`: `crc::crc16::Digest::new_with_initial(0xa001, 0xffff)`.

### 3.5 Quick CRC sanity vector

Given the canonical PING request (`req = 0x33 0x01 0x00 0x00 0x00 0x00 ?? ??`):
- CRC-16/MODBUS over `33 01 00 00 00 00` = `0xC1C0`
- Wire bytes 6, 7 = `0xC1`, `0xC0`
- Full PING frame = **`33 01 00 00 00 00 C1 C0`**

(Verified by running the `make_crc16` reference function above against the listed bytes.) Use this exact byte sequence as the first integration checkpoint when bringing up the firmware: PiKVM's `kvmd` issues PING any time the event queue is empty, so it will arrive within ~100 ms of any successful connection.

---

## 4. Command set

All command opcodes are defined in `hid/pico/src/ph_proto.h`:

```c
#define PH_PROTO_CMD_PING               ((u8)0x01)
#define PH_PROTO_CMD_REPEAT             ((u8)0x02)
#define PH_PROTO_CMD_SET_KBD            ((u8)0x03)
#define PH_PROTO_CMD_SET_MOUSE          ((u8)0x04)
#define PH_PROTO_CMD_SET_CONNECTED      ((u8)0x05)
#define PH_PROTO_CMD_CLEAR_HID          ((u8)0x10)
#define PH_PROTO_CMD_KBD_KEY            ((u8)0x11)
#define PH_PROTO_CMD_MOUSE_ABS          ((u8)0x12)
#define PH_PROTO_CMD_MOUSE_BUTTON       ((u8)0x13)
#define PH_PROTO_CMD_MOUSE_WHEEL        ((u8)0x14)
#define PH_PROTO_CMD_MOUSE_REL          ((u8)0x15)
```

Dispatch table (from `main.c:_handle_request`):

```c
switch (data[1]) {
    case PH_PROTO_CMD_PING:           return PH_PROTO_PONG_OK;
    case PH_PROTO_CMD_SET_KBD:        HANDLE(ph_cmd_set_kbd, true);
    case PH_PROTO_CMD_SET_MOUSE:      HANDLE(ph_cmd_set_mouse, true);
    case PH_PROTO_CMD_SET_CONNECTED:  return PH_PROTO_PONG_OK; // no-op except for Arduino AUM
    case PH_PROTO_CMD_CLEAR_HID:      HANDLE(ph_cmd_send_clear, false);
    case PH_PROTO_CMD_KBD_KEY:        HANDLE(ph_cmd_kbd_send_key, false);
    case PH_PROTO_CMD_MOUSE_BUTTON:   HANDLE(ph_cmd_mouse_send_button, false);
    case PH_PROTO_CMD_MOUSE_ABS:      HANDLE(ph_cmd_mouse_send_abs, false);
    case PH_PROTO_CMD_MOUSE_REL:      HANDLE(ph_cmd_mouse_send_rel, false);
    case PH_PROTO_CMD_MOUSE_WHEEL:    HANDLE(ph_cmd_mouse_send_wheel, false);
    case PH_PROTO_CMD_REPEAT:         return 0;
}
return PH_PROTO_RESP_INVALID_ERROR;
```

Where `HANDLE(fn, x_reset)` calls `fn(data + 2)` (i.e., the 5-byte payload starting at offset 2) and returns `PH_PROTO_PONG_OK`. If `x_reset` is true, the firmware schedules a watchdog reset after the response is sent.

### 4.1 PING — `0x01`

| Byte    | Value   |
|---------|---------|
| 0       | `0x33`  |
| 1       | `0x01`  |
| 2..6    | `0x00`  |
| 6..8    | CRC-16  |

Canonical bytes: `33 01 00 00 00 00 C1 C0`.

Action: none. Response: `PONG_OK` with current state (see §5).

### 4.2 REPEAT — `0x02`

Tells the MCU to **re-send the previous response verbatim**. Used by the host on CRC failures (`kvmd`'s `__process_request` falls back to `REQUEST_REPEAT` if `check_response` fails).

Implementation in firmware — `_send_response`:

```c
static void _send_response(u8 code) {
    static u8 prev_code = PH_PROTO_RESP_NONE;
    if (code == 0) {
        code = prev_code; // Repeat the last code
    } else {
        prev_code = code;
    }
    ...
}
```

So `REPEAT` returns 0 from the dispatcher → `_send_response(0)` → uses the cached `prev_code`. **The ESP32 port must keep `prev_code` (and the rest of the response state) cached** between commands.

### 4.3 SET_KBD — `0x03` (1-byte payload)

| Byte    | Value/meaning                                   |
|---------|-------------------------------------------------|
| 0       | `0x33`                                          |
| 1       | `0x03`                                          |
| 2       | new keyboard mode (3-bit field, see below)      |
| 3..6    | `0x00` (unused)                                 |
| 6..8    | CRC-16                                          |

Mode codes (`OUT1_KBD_MASK = 0b00000111`):

| Code        | Meaning   |
|-------------|-----------|
| `0b000`     | disabled  |
| `0b001`     | usb       |
| `0b011`     | ps/2      |

After ack, the MCU **must reboot** (`x_reset = true`). The Pico does this via watchdog. Response is `PONG_OK | RESET_REQUIRED`.

Host-side definition in `proto.py`:

```python
_KEYBOARD_NAMES_TO_CODES = {"disabled": 0b00000000, "usb": 0b00000001, "ps2": 0b00000011}
def make_request(self) -> bytes:
    code = _KEYBOARD_NAMES_TO_CODES.get(self.keyboard, 0)
    return _make_request(struct.pack(">BBxxx", 0x03, code))
```

### 4.4 SET_MOUSE — `0x04` (1-byte payload)

Identical framing to `SET_KBD`. Mode codes (`OUT1_MOUSE_MASK = 0b00111000`):

| Code        | Meaning           |
|-------------|-------------------|
| `0b000000`  | disabled          |
| `0b001000`  | usb_abs (default) |
| `0b010000`  | usb_rel           |
| `0b011000`  | ps2               |
| `0b100000`  | usb_win98 (abs, doubled coordinates) |

Same reboot-after-ack semantics as `SET_KBD`.

### 4.5 SET_CONNECTED — `0x05`

Legacy Arduino AUM (auto-USB-mux) toggle. **The Pico firmware accepts it but ignores the payload** and just returns `PONG_OK`. Implement as a no-op that succeeds.

### 4.6 CLEAR_HID — `0x10`

Force all keys up, release all mouse buttons. From `ph_cmds.c`:

```c
void ph_cmd_send_clear(const u8 *args) { // 0 bytes
    (void)args;
    ph_usb_send_clear();
    ph_ps2_send_clear();
}
```

Payload bytes 2..6 are all `0x00`.

### 4.7 KBD_KEY — `0x11` (2-byte payload)

| Byte | Meaning                                                          |
|------|------------------------------------------------------------------|
| 0    | `0x33`                                                           |
| 1    | `0x11`                                                           |
| 2    | **MCU keycode** (NOT a USB HID usage code — see below)           |
| 3    | state: `0` = release, `1` = press                                |
| 4..6 | `0x00`                                                           |
| 6..8 | CRC-16                                                           |

```python
def make_request(self) -> bytes:
    code = KEYMAP[self.code].mcu.code
    return _make_request(struct.pack(">BBBxx", 0x11, code, int(self.state)))
```

```c
void ph_cmd_kbd_send_key(const u8 *args) { // 2 bytes
    const u8 key = ph_usb_keymap(args[0]);  // <-- translate MCU code → USB usage
    if (key > 0) {
        if (PH_O_IS_KBD_USB) {
            ph_usb_kbd_send_key(key, args[1]);
        } else if (PH_O_IS_KBD_PS2) {
            ph_ps2_kbd_send_key(key, args[1]);
        }
    }
}
```

#### 4.7.1 MCU code → USB HID usage translation

**The firmware ships its own integer keycode table** — kvmd does NOT send the standard USB HID usage value; it sends a sequential integer (1, 2, 3, …) defined in `keymap.csv` column 3 (`mcu_code`). At firmware build time, `genmap.py` renders `ph_usb_keymap.h.mako` into a C `switch` statement that maps the MCU code to the standard USB HID usage code.

Source of truth: `kvmd/keymap.csv` — 116 rows, MCU codes 1..115 (some gaps reserved). Format (excerpted):

```
web_name,evdev_name,mcu_code,usb_key,ps2_key,at1_code,x11_names
KeyA,KEY_A,1,0x04,reg:0x1c,0x1e,"^XK_A,XK_a"
KeyB,KEY_B,2,0x05,reg:0x32,0x30,"^XK_B,XK_b"
...
ControlLeft,KEY_LEFTCTRL,77,^0x01,reg:0x14,0x1d,XK_Control_L
ShiftLeft,KEY_LEFTSHIFT,78,^0x02,reg:0x12,0x2a,XK_Shift_L
AltLeft,KEY_LEFTALT,79,^0x04,reg:0x11,0x38,XK_Alt_L
...
F20,KEY_F20,115,0x6f,,0x5a,
```

The `usb_key` column is the standard 8-bit USB HID Keyboard/Keypad page usage. A leading `^` marks **modifiers**: the value is interpreted as a *bitmask* (0x01 = LCtrl, 0x02 = LShift, 0x04 = LAlt, 0x08 = LGUI, 0x10 = RCtrl, 0x20 = RShift, 0x40 = RAlt, 0x80 = RGUI). For modifiers, the firmware converts that bitmask back into a USB HID usage in the `0xE0..0xE7` range:

```python
@property
def arduino_mod_code(self) -> int:
    code = self.code
    offset = 0
    while not (code & 0x1):
        code >>= 1
        offset += 1
    return ((0xE << 4) | offset)  # 0xE0..0xE7
```

In `ph_usb_kbd.c`:

```c
if (key >= HID_KEY_CONTROL_LEFT && key <= HID_KEY_GUI_RIGHT) { // 0xE0..0xE7
    key = 1 << (key & 0x07);  // turn it back into a modifier bitmask bit
    if (state) _kbd_mods |= key; else _kbd_mods &= ~key;
} else { // Regular keys go into the 6-key rollover slots
    ...
}
_kbd_sync_report(true);
```

**For the ESP32-S3 port**: copy `keymap.csv` from the kvmd repo verbatim and either run `genmap.py` against the same Mako template, or hand-port the resulting `switch` statement. Don't try to derive it — the MCU codes are an arbitrary historical numbering, not algorithmically related to USB HID usage.

Unknown MCU codes return `0` from `ph_usb_keymap()` and the keypress is silently dropped (NOT an error response).

### 4.8 MOUSE_ABS — `0x12` (4-byte payload)

| Byte | Meaning                                       |
|------|-----------------------------------------------|
| 0    | `0x33`                                        |
| 1    | `0x12`                                        |
| 2..4 | X coordinate, **signed 16-bit big-endian**    |
| 4..6 | Y coordinate, **signed 16-bit big-endian**    |
| 6..8 | CRC-16                                        |

```python
def make_request(self) -> bytes:
    return _make_request(struct.pack(">Bhh", 0x12, self.to_x, self.to_y))
```

```c
void ph_cmd_mouse_send_abs(const u8 *args) { // 4 bytes
    if (PH_O_IS_MOUSE_USB_ABS) {
        const s16 x = ph_merge8_s16(args[0], args[1]);
        const s16 y = ph_merge8_s16(args[2], args[3]);
        ph_usb_mouse_send_abs(x, y);
    }
}
```

Range: `[-32768, +32767]` per axis. The firmware then maps that to the USB HID descriptor's `[0, 32767]` range in `_mouse_abs_send_report`:

```c
u16 x = ((s32)_mouse_abs_x + 32768) / 2;
u16 y = ((s32)_mouse_abs_y + 32768) / 2;
if (PH_O_MOUSE(USB_W98)) { x <<= 1; y <<= 1; }   // Win98 mode: full 16-bit range
```

Ignored if active mouse is not absolute USB.

### 4.9 MOUSE_BUTTON — `0x13` (2-byte payload)

| Byte | Meaning                                                         |
|------|-----------------------------------------------------------------|
| 0    | `0x33`                                                          |
| 1    | `0x13`                                                          |
| 2    | "main" buttons byte — left/right/middle select+state            |
| 3    | "extra" buttons byte — back/forward (i.e., side buttons) select+state |
| 4..6 | `0x00`                                                          |
| 6..8 | CRC-16                                                          |

Each button uses **two bits** in its byte: a `SELECT` bit ("this command updates this button") and a `STATE` bit (1=press, 0=release). If `SELECT` is 0, the existing button state is unchanged.

Byte 2 layout (`args[0]`):

| Bit | Mask        | Meaning                           |
|-----|-------------|-----------------------------------|
| 7   | `0x80`      | LEFT_SELECT                       |
| 6   | `0x40`      | RIGHT_SELECT                      |
| 5   | `0x20`      | MIDDLE_SELECT                     |
| 3   | `0x08`      | LEFT_STATE   (1=down, 0=up)       |
| 2   | `0x04`      | RIGHT_STATE                       |
| 1   | `0x02`      | MIDDLE_STATE                      |

Byte 3 layout (`args[1]`):

| Bit | Mask        | Meaning                           |
|-----|-------------|-----------------------------------|
| 7   | `0x80`      | BACKWARD_SELECT (mouse button "Up"/Prev) |
| 6   | `0x40`      | FORWARD_SELECT  (mouse button "Down"/Next) |
| 3   | `0x08`      | BACKWARD_STATE                    |
| 2   | `0x04`      | FORWARD_STATE                     |

`kvmd`'s `MouseButtonEvent` only updates one button per request (it picks the right byte based on which button changed):

```python
(code, state_pressed, is_main) = {
    ecodes.BTN_LEFT:    (0b10000000, 0b00001000, True),
    ecodes.BTN_RIGHT:   (0b01000000, 0b00000100, True),
    ecodes.BTN_MIDDLE:  (0b00100000, 0b00000010, True),
    ecodes.BTN_BACK:    (0b10000000, 0b00001000, False),  # Up
    ecodes.BTN_FORWARD: (0b01000000, 0b00000100, False),  # Down
}[self.code]
if self.state:
    code |= state_pressed
if is_main:
    main_code = code; extra_code = 0
else:
    main_code = 0; extra_code = code
return _make_request(struct.pack(">BBBxx", 0x13, main_code, extra_code))
```

Firmware handler:

```c
#define HANDLE(x_byte_n, x_button) { \
    if (args[x_byte_n] & PH_PROTO_CMD_MOUSE_##x_button##_SELECT) { \
        const bool m_state = !!(args[x_byte_n] & PH_PROTO_CMD_MOUSE_##x_button##_STATE); \
        ph_usb_mouse_send_button(MOUSE_BUTTON_##x_button, m_state); \
    } }
HANDLE(0, LEFT); HANDLE(0, RIGHT); HANDLE(0, MIDDLE);
HANDLE(1, BACKWARD); HANDLE(1, FORWARD);
```

### 4.10 MOUSE_WHEEL — `0x14` (2-byte payload)

| Byte | Meaning                                                       |
|------|---------------------------------------------------------------|
| 0    | `0x33`                                                        |
| 1    | `0x14`                                                        |
| 2    | horizontal wheel delta (s8) — **firmware ignores this byte** |
| 3    | vertical wheel delta (s8), `[-127, +127]`                     |
| 4..6 | `0x00`                                                        |
| 6..8 | CRC-16                                                        |

Host:

```python
# Горизонтальная прокрутка пока не поддерживается
return _make_request(struct.pack(">Bxbxx", 0x14, self.delta_y))
```

(`>Bxbxx` = opcode, padding, signed_byte=delta_y, two padding) — so actually byte 2 is `0x00` from kvmd and byte 3 carries delta_y. Firmware passes both bytes through (`args[0]`, `args[1]`) to `ph_usb_mouse_send_wheel(args[0], args[1])` but the implementation in `ph_usb.c:_mouse_abs_send_report` discards `h`:

```c
static void _mouse_abs_send_report(s8 h, s8 v) {
    (void)h; // Horizontal scrolling is not supported due BIOS/UEFI compatibility reasons
    ...
```

For an ESP32 port, accept and ignore byte 2; only act on byte 3 (vertical).

### 4.11 MOUSE_REL — `0x15` (2-byte payload)

| Byte | Meaning                                |
|------|----------------------------------------|
| 0    | `0x33`                                 |
| 1    | `0x15`                                 |
| 2    | delta X, signed 8-bit `[-127, +127]`   |
| 3    | delta Y, signed 8-bit `[-127, +127]`   |
| 4..6 | `0x00`                                 |
| 6..8 | CRC-16                                 |

Host:

```python
return _make_request(struct.pack(">Bbbxx", 0x15, self.delta_x, self.delta_y))
```

Firmware:

```c
void ph_cmd_mouse_send_rel(const u8 *args) { // 2 bytes
    if (PH_O_IS_MOUSE_USB_REL) {
        ph_usb_mouse_send_rel(args[0], args[1]);
    } else if (PH_O_IS_MOUSE_PS2) {
        ph_ps2_mouse_send_rel(args[0], args[1]);
    }
}
```

Ignored if mouse is in absolute USB mode.

---

## 5. Response format

### 5.1 Status byte (response byte [1])

Bare error codes (high bit clear) — sent when the request was bad or there was no payload to act on:

| Code  | Macro                            | Meaning                                                  |
|-------|----------------------------------|----------------------------------------------------------|
| `0x20`| (legacy) `RESP_OK`               | Plain "ok" used by old Arduino HIDs in 4-byte responses. The new Pico firmware never emits this; the host driver still recognizes it (`code == 0x20 → return True`). |
| `0x24`| `RESP_NONE`                      | "No previous response cached" — only emitted if `REPEAT` is received before any other command has run. Host treats this as "MCU was rebooted". |
| `0x40`| `RESP_CRC_ERROR`                 | Request CRC didn't match.                                |
| `0x45`| `RESP_INVALID_ERROR`             | Unknown opcode.                                          |
| `0x48`| `RESP_TIMEOUT_ERROR`             | The 100 ms inter-byte gap was exceeded mid-frame.        |

Pong code (high bit set) — sent on every successful command:

| Bit | Mask    | Macro                                | Meaning                                    |
|-----|---------|--------------------------------------|--------------------------------------------|
| 7   | `0x80`  | `PONG_OK`                            | "Command processed" — must always be set in pong   |
| 6   | `0x40`  | `PONG_RESET_REQUIRED`                | MCU is about to reboot (after `SET_KBD`/`SET_MOUSE`) |
| 4   | `0x10`  | `PONG_MOUSE_OFFLINE`                 | USB host hasn't enumerated the mouse interface |
| 3   | `0x08`  | `PONG_KBD_OFFLINE`                   | USB host hasn't enumerated the keyboard interface |
| 2   | `0x04`  | `PONG_NUM`                           | Num Lock LED on                            |
| 1   | `0x02`  | `PONG_SCROLL`                        | Scroll Lock LED on                         |
| 0   | `0x01`  | `PONG_CAPS`                          | Caps Lock LED on                           |

### 5.2 OUTPUTS1 byte (response byte [2])

Layout (low bits = active modes, high bit = "I am a modern firmware that fills this byte in"):

| Bit | Mask    | Macro                       | Meaning                              |
|-----|---------|------------------------------|--------------------------------------|
| 7   | `0x80`  | `OUT1_DYNAMIC`              | Set on every pong from the new firmware. **Required**: kvmd ignores OUT1/OUT2 unless this bit is set (`if outputs1 & 0b10000000`). |
| 5..3| `0x38`  | `OUT1_MOUSE_MASK`           | Active mouse mode (see encoding below) |
| 2..0| `0x07`  | `OUT1_KBD_MASK`             | Active keyboard mode                 |

Keyboard sub-encoding (low 3 bits):

| Value      | Macro                  | Meaning   |
|------------|------------------------|-----------|
| `0b000`    | (disabled)             | disabled  |
| `0b001`    | `OUT1_KBD_USB`         | USB       |
| `0b011`    | `OUT1_KBD_PS2`         | PS/2      |

Mouse sub-encoding (bits 3..5):

| Value         | Macro                  | Meaning       |
|---------------|------------------------|---------------|
| `0b000000`    | (disabled)             | disabled      |
| `0b001000`    | `OUT1_MOUSE_USB_ABS`   | USB absolute  |
| `0b010000`    | `OUT1_MOUSE_USB_REL`   | USB relative  |
| `0b011000`    | `OUT1_MOUSE_PS2`       | PS/2          |
| `0b100000`    | `OUT1_MOUSE_USB_W98`   | USB Win98 abs |

### 5.3 OUTPUTS2 byte (response byte [3])

Available capabilities (advertised to kvmd so the UI can show "you can switch keyboard to PS/2"):

| Bit | Mask    | Macro                       | Meaning                                   |
|-----|---------|------------------------------|-------------------------------------------|
| 7   | `0x80`  | `OUT2_CONNECTABLE`          | This MCU honors `SET_CONNECTED`           |
| 6   | `0x40`  | `OUT2_CONNECTED`            | Currently "connected" (only meaningful if CONNECTABLE) |
| 2   | `0x04`  | `OUT2_HAS_USB_W98`          | USB Win98 abs mouse mode is wired up      |
| 1   | `0x02`  | `OUT2_HAS_PS2`              | PS/2 stack is wired up                    |
| 0   | `0x01`  | `OUT2_HAS_USB`              | USB HID stack is wired up                 |

For an ESP32-S3 USB-only HID port, the minimum legitimate output is:

```
OUTPUTS1 = OUT1_DYNAMIC | OUT1_KBD_USB | OUT1_MOUSE_USB_ABS  = 0x80 | 0x01 | 0x08 = 0x89
OUTPUTS2 = OUT2_HAS_USB                                       = 0x01
```

Then OR the keyboard LED bits and offline bits into the **STATUS** byte each frame.

### 5.4 Host-side parsing (for verification)

`kvmd/plugins/hid/_mcu/__init__.py:get_state` decodes the response into the kvmd state dict:

```python
status = resp[1] << 16
if len(resp) > 4:
    status |= (resp[2] << 8) | resp[3]
...
"keyboard": {
    "online": (online and not (pong & 0b00001000)),
    "leds": {
        "caps":   bool(pong & 0b00000001),
        "scroll": bool(pong & 0b00000010),
        "num":    bool(pong & 0b00000100),
    },
    "outputs": keyboard_outputs,
},
"mouse": {
    "online": (online and not (pong & 0b00010000)),
    ...
},
```

If your firmware never lights any of the LED bits, kvmd shows the keyboard LEDs as off. If you never set `OUT1_DYNAMIC`, the kvmd UI shows "no outputs available" and refuses to let the user pick a mode.

---

## 6. USB HID descriptors (target-host facing)

The Pico HID exposes itself as **USB 2.0 Full-Speed**, **VID `0x1209` (pid.codes), PID `0xEDA2`** for the HID variant. (PID `0xEDA3` is reserved for the bridge variant which is CDC-only — not relevant here.)

From `ph_usb.c`:

```c
.bcdUSB             = 0x0200,
.bDeviceClass       = 0,        // Composite via interface association
.bDeviceSubClass    = 0,
.bDeviceProtocol    = 0,
.bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
.idVendor           = 0x1209,   // pid.codes
.idProduct          = 0xEDA2,
.bcdDevice          = 0x0100,
.iManufacturer      = 1,        // "PiKVM"
.iProduct           = 2,        // "PiKVM HID"
.iSerialNumber      = 3,        // unique per board
.bNumConfigurations = 1,
```

Configuration: one HID interface for the keyboard (HID protocol = `KEYBOARD`, polling = 1 ms) and one HID interface for the mouse (protocol = `NONE` for absolute, `MOUSE` for relative, polling = 1 ms). Endpoints are interrupt-IN at addresses 0x81 and 0x82, 64-byte packets.

### 6.1 Keyboard HID report descriptor

Verbatim from `hid/pico/src/ph_usb_kbd.c` (Logitech-style 8-modifier-bit + reserved + 5-LED + 6-key boot keyboard layout):

```c
0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
0x09, 0x06,         // USAGE (Keyboard)
0xA1, 0x01,         // COLLECTION (Application)

// Modifiers
0x05, 0x07,         // USAGE_PAGE (Keyboard)
0x19, 0xE0,         // USAGE_MINIMUM (Keyboard LeftControl)
0x29, 0xE7,         // USAGE_MAXIMUM (Keyboard Right GUI)
0x15, 0x00,         // LOGICAL_MINIMUM (0)
0x25, 0x01,         // LOGICAL_MAXIMUM (1)
0x75, 0x01,         // REPORT_SIZE (1)
0x95, 0x08,         // REPORT_COUNT (8)
0x81, 0x02,         // INPUT (Data,Var,Abs)

// Reserved byte
0x95, 0x01,         // REPORT_COUNT (1)
0x75, 0x08,         // REPORT_SIZE (8)
0x81, 0x01,         // INPUT (Const)

// LEDs output (5 bits)
0x95, 0x05,         // REPORT_COUNT (5)
0x75, 0x01,         // REPORT_SIZE (1)
0x05, 0x08,         // USAGE_PAGE (LEDs)
0x19, 0x01,         // USAGE_MINIMUM (Num Lock)
0x29, 0x05,         // USAGE_MAXIMUM (Kana)
0x91, 0x02,         // OUTPUT (Data,Var,Abs)

// Padding 3 bits in output
0x95, 0x01,         // REPORT_COUNT (1)
0x75, 0x03,         // REPORT_SIZE (3)
0x91, 0x01,         // OUTPUT (Const)

// 6 keys
0x95, 0x06,         // REPORT_COUNT (6)
0x75, 0x08,         // REPORT_SIZE (8)
0x15, 0x00,         // LOGICAL_MINIMUM (0)
0x26, 0xFF, 0x00,   // LOGICAL_MAXIMUM (0xFF)
0x05, 0x07,         // USAGE_PAGE (Keyboard)
0x19, 0x00,         // USAGE_MINIMUM (Reserved)
0x2A, 0xFF, 0x00,   // USAGE_MAXIMUM (0xFF)
0x81, 0x00,         // INPUT (Data,Array,Abs)

0xC0,               // END_COLLECTION
```

Report layout (8 bytes IN, 1 byte OUT):

```
IN report (Pico → target):
  byte 0    : modifier bitmask (LCtrl=0x01, LShift=0x02, LAlt=0x04, LGUI=0x08,
                                RCtrl=0x10, RShift=0x20, RAlt=0x40, RGUI=0x80)
  byte 1    : reserved (always 0)
  bytes 2-7 : up to 6 currently-pressed key usage codes

OUT report (target → Pico, captured into ph_g_usb_kbd_leds):
  byte 0    : LED bitmask (NumLock=0x01, CapsLock=0x02, ScrollLock=0x04,
                            Compose=0x08, Kana=0x10)
```

The HID interface protocol byte is `HID_ITF_PROTOCOL_KEYBOARD` so it works in BIOS/UEFI as a boot keyboard.

### 6.2 Absolute mouse HID report descriptor

Verbatim from `ph_usb_mouse.c`:

```c
0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
0x09, 0x02,         // USAGE (Mouse)
0xA1, 0x01,         // COLLECTION (Application)
0x09, 0x01,         // USAGE (Pointer)
0xA1, 0x00,         // COLLECTION (Physical)

// 8 buttons
0x05, 0x09,         // USAGE_PAGE (Button)
0x19, 0x01,         // USAGE_MINIMUM (Button 1)
0x29, 0x08,         // USAGE_MAXIMUM (Button 8)
0x15, 0x00,         // LOGICAL_MINIMUM (0)
0x25, 0x01,         // LOGICAL_MAXIMUM (1)
0x95, 0x08,         // REPORT_COUNT (8)
0x75, 0x01,         // REPORT_SIZE (1)
0x81, 0x02,         // INPUT (Data,Var,Abs)

// X, Y absolute, 16-bit, 0..32767
0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
0x09, 0x30,         // USAGE (X)
0x09, 0x31,         // USAGE (Y)
0x16, 0x00, 0x00,   // LOGICAL_MINIMUM (0)
0x26, 0xFF, 0x7F,   // LOGICAL_MAXIMUM (32767)
0x75, 0x10,         // REPORT_SIZE (16)
0x95, 0x02,         // REPORT_COUNT (2)
0x81, 0x02,         // INPUT (Data,Var,Abs)

// Wheel relative, signed 8-bit
0x09, 0x38,         // USAGE (Wheel)
0x15, 0x81,         // LOGICAL_MINIMUM (-127)
0x25, 0x7F,         // LOGICAL_MAXIMUM (127)
0x75, 0x08,         // REPORT_SIZE (8)
0x95, 0x01,         // REPORT_COUNT (1)
0x81, 0x06,         // INPUT (Data,Var,Rel)

0xC0,               // END_COLLECTION (Physical)
0xC0,               // END_COLLECTION
```

Report layout (6 bytes):

```
byte 0   : button bitmask (Btn1=0x01, Btn2=0x02, Btn3=0x04, ...)
bytes 1-2: X, little-endian uint16, range 0..32767
bytes 3-4: Y, little-endian uint16
byte 5   : wheel delta, signed int8
```

The HID interface protocol byte is `HID_ITF_PROTOCOL_NONE` for this descriptor (it's not a boot mouse — it's an absolute pointer).

### 6.3 Relative (boot) mouse HID report descriptor

```c
0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
0x09, 0x02,         // USAGE (Mouse)
0xA1, 0x01,         // COLLECTION (Application)
0x09, 0x01,         // USAGE (Pointer)
0xA1, 0x00,         // COLLECTION (Physical)

// 8 buttons
0x05, 0x09,         // USAGE_PAGE (Button)
0x19, 0x01,         // USAGE_MINIMUM (Button 1)
0x29, 0x08,         // USAGE_MAXIMUM (Button 8)
0x15, 0x00,         // LOGICAL_MINIMUM (0)
0x25, 0x01,         // LOGICAL_MAXIMUM (1)
0x95, 0x08,         // REPORT_COUNT (8)
0x75, 0x01,         // REPORT_SIZE (1)
0x81, 0x02,         // INPUT (Data,Var,Abs)

// X, Y, Wheel relative, signed 8-bit
0x05, 0x01,         // USAGE_PAGE (Generic Desktop)
0x09, 0x30,         // USAGE (X)
0x09, 0x31,         // USAGE (Y)
0x09, 0x38,         // USAGE (Wheel)
0x15, 0x81,         // LOGICAL_MINIMUM (-127)
0x25, 0x7F,         // LOGICAL_MAXIMUM (127)
0x75, 0x08,         // REPORT_SIZE (8)
0x95, 0x03,         // REPORT_COUNT (3)
0x81, 0x06,         // INPUT (Data,Var,Rel)

0xC0,               // END_COLLECTION (Physical)
0xC0,               // END_COLLECTION
```

Report layout (4 bytes):

```
byte 0: button bitmask
byte 1: dx (s8)
byte 2: dy (s8)
byte 3: wheel (s8)
```

HID interface protocol byte is `HID_ITF_PROTOCOL_MOUSE` (so it works as a boot mouse).

### 6.4 Win98 mouse mode

Same descriptor as the absolute mouse (§6.2). The only difference is that the firmware doubles the X/Y mapping (`x <<= 1; y <<= 1`) before emitting the report — this side-steps a Microsoft Windows 9x quirk. Set `OUT2_HAS_USB_W98` only if you implement this code path.

### 6.5 String descriptors

```
String 0  : language ID (0x0409 = English US)
String 1  : "PiKVM"               (manufacturer)
String 2  : "PiKVM HID"           (product)  ← "PiKVM HID Bridge" in bridge mode
String 3  : <Pico unique board ID> (serial)  ← use ESP32 MAC or chip ID
```

For drop-in compatibility, you can keep the same VID/PID (`0x1209/0xEDA2`) — pid.codes' Pi-KVM allocation is intended for compatible firmware. The serial number must be unique per device.

---

## 7. State machine

### 7.1 Boot

`hid/pico/src/main.c:main`:

```c
ph_outputs_init();    // Read mode jumpers, restore previous mode from watchdog scratch reg
ph_ps2_init();        // Bring up PS/2 stack if enabled
ph_usb_init();        // tud_init(0) if USB is enabled
ph_com_init(_data_handler, _timeout_handler);  // Choose UART or SPI based on GP22
while (true) {
    ph_usb_task();
    ph_ps2_task();
    if (!_reset_required) {
        ph_com_task();
    }
}
```

Notably:
- The Pico **persists the active output mode across reboots** by storing it in a watchdog scratch register (`watchdog_hw->scratch[0]`) with its own CRC-16. This is how `SET_KBD`/`SET_MOUSE` survives the deliberate reboot. For the ESP32 port: use NVS or RTC slow memory to persist the mode byte across `esp_restart()`.
- Once `_reset_required` is set, the firmware stops servicing UART traffic until the reboot completes.

### 7.2 Per-byte UART RX state

`ph_com_uart_task`:

```c
void ph_com_uart_task(void) {
    if (uart_is_readable(_BUS)) {
        _buf[_index] = (u8)uart_getc(_BUS);
        if (_index == 7) {
            _data_cb(_buf);   // full 8-byte packet received → dispatch
            _index = 0;
        } else {
            _last_ts = time_us_64();
            ++_index;
        }
    } else if (_index > 0) {
        if (_last_ts + _TIMEOUT_US < time_us_64()) {  // 100ms inter-byte timeout
            _timeout_cb();    // emit 0x48 (TIMEOUT) response
            _index = 0;
        }
    }
}
```

Key behaviors:

1. **No magic-byte resync**: the firmware accepts any 8 bytes as a frame. If the framing slips, the CRC will fail and the host will recover via `REPEAT`. Your ESP32 implementation can be more aggressive (resync on `0x33`) without breaking compatibility — kvmd doesn't care, it just retries.
2. **Inter-byte timeout = 100 ms** (`_TIMEOUT_US = 100000`). On timeout, emit a response with status byte `0x48` (`RESP_TIMEOUT_ERROR`) and reset the parser.
3. **Bad CRC** → respond with status `0x40` (`RESP_CRC_ERROR`). The host will then send `REPEAT`; respond with the same status until it sends a fresh, valid command.
4. **Unknown opcode** → respond with status `0x45` (`RESP_INVALID_ERROR`). This is treated by kvmd as a *permanent* error for that request — it will not retry.
5. **REPEAT before any prior command** → status `0x24` (`RESP_NONE`), which kvmd interprets as "the MCU rebooted, my state is stale" and triggers a full reset/clear cycle.

### 7.3 Idle / heartbeat

When the host has nothing to send, it sends `PING` ~10 Hz (the `__hid_loop` polls the event queue with a 100 ms timeout, falls through to `REQUEST_PING` on empty). The MCU should respond to PING within a few ms — kvmd's `read_timeout` defaults to 2 s but treats any missed response as "offline" (`busy=False, online=False`) which lights up an alarm in the UI.

### 7.4 Reset behavior

Two reset paths:

1. **Hardware reset (RUN line)** — kvmd asserts a Pi GPIO (default GPIO4, configured GPIO25) for `reset_delay` (default 100 ms) when it can't reach the MCU. The Pico's RUN pin is active-low, so kvmd uses `reset_inverted: true` in `override.yaml`.
2. **Self-reset** — after `SET_KBD`/`SET_MOUSE`, the MCU acks with `PONG_OK | RESET_REQUIRED` and reboots ~100 ms later. kvmd, with `reset_self: true`, expects this and will pause for 1 s before reconnecting:

   ```python
   except _SelfResetError:
       time.sleep(1)  # Pico перезагружается сам вскоре после ответа
       reset = False
   ```

The ESP32-S3 port should at minimum implement self-reset. Hardware reset is a "last resort" path — if you boot promptly (≤2 s) on a power cycle, you can omit the RUN pin and the host won't notice.

### 7.5 USB enumeration / "offline" reporting

While the target host hasn't yet enumerated the keyboard/mouse interface, the firmware sets `PONG_KBD_OFFLINE` / `PONG_MOUSE_OFFLINE` in the response status. This is informational only — kvmd shows it as a yellow indicator. The Pico debounces the offline state with a 50 ms threshold (`offline_ts + 50000 < now_ts` in `ph_usb_task`).

---

## 8. Existing ESP32 / alternative ports

| Project | URL | Notes |
|---------|-----|-------|
| **`microxblue/pikvm_hid`** | <https://github.com/microxblue/pikvm_hid> | STM32F103 ("BluePill") port. Uses STM32 UART3 (PB10 TX, PB11 RX) to Pi GPIO14/15. Implements the same `0x33`/`0x34` 8-byte protocol. Useful as a second reference implementation. |
| **`jannic/pikvm-hid-stm32`** | <https://github.com/jannic/pikvm-hid-stm32> | Independent Rust STM32 port. **Verified to use the same 115200 baud, `0x33` magic, `crc16(0xa001, 0xffff)` we documented above.** Its `proto.rs` lines 164–197 are the cleanest non-C reference. Note: it emits a 4-byte response (`0x33 0x80 …`) — the legacy format — not the 8-byte `0x34` format. The current kvmd driver accepts both. |
| **No public ESP32 port found** | — | A web search for `"esp32 pikvm hid"`, `"pikvm pico-hid esp32"`, and `"pikvm 0x33 esp32"` (April 2026) returned no results. This brief is the spec for the new port. |

If you discover an ESP32 port mid-build, prefer it over a fresh implementation only if it tracks the **current** kvmd commit (post-2024 SPI/UART selector, post-`OUT1_DYNAMIC` response format).

---

## Appendix A: Bring-up checklist for the ESP32-S3 firmware

1. Wire UART2 (or any free UART instance) RX to Pi BCM GPIO14 (TX), TX to Pi BCM GPIO15 (RX), GND common, 115200 8N1.
2. Implement the CRC-16 from §3.4 — verify against `33 01 00 00 00 00 → CRC C1 C0`.
3. Implement the 8-byte fixed parser with 100 ms inter-byte timeout (§7.2).
4. Reply to PING with `34 89 89 01 00 00 <CRC_HI> <CRC_LO>` (status = PONG_OK + dynamic+kbd_usb+mouse_usb_abs, outputs2 = HAS_USB). This is kvmd's heartbeat — get this working first; the kvmd UI should flip "online".
5. Wire up TinyUSB or `tinyusb_hid_*` (ESP32-S3 native USB OTG) with the descriptors in §6. Don't use `usb_hid_keyboard_ll` shortcuts — kvmd parses USB enumeration via the OS, but the *target* PC needs the exact descriptors above for boot keyboard support.
6. Implement KBD_KEY (§4.7) using a copy of `keymap.csv`. Run `genmap.py` against `ph_usb_keymap.h.mako` to generate a static lookup table; commit the rendered `.h` so you don't need a Python build step.
7. Implement MOUSE_ABS, MOUSE_REL, MOUSE_BUTTON, MOUSE_WHEEL, CLEAR_HID.
8. Implement SET_KBD/SET_MOUSE with NVS-backed mode persistence and self-reset. SET_CONNECTED can be a hard-coded `PONG_OK`.
9. Implement REPEAT — cache the last response status byte and re-emit on REPEAT (using current outputs/LEDs).

## Appendix B: File index of authoritative sources

| File (in `pikvm/kvmd@master`) | What it tells you |
|---|---|
| `hid/pico/src/ph_proto.h`        | All command, response, and flag constants (single source of truth) |
| `hid/pico/src/ph_com_uart.c`     | UART parameters, pins, 8-byte packet handling, 100ms timeout |
| `hid/pico/src/ph_com_spi.c`      | SPI alternative — informational; not used over UART |
| `hid/pico/src/ph_com.c`          | UART vs SPI mode selection (GP22) |
| `hid/pico/src/main.c`            | Top-level dispatch and response builder |
| `hid/pico/src/ph_cmds.c`         | Per-command argument decoding |
| `hid/pico/src/ph_outputs.c`      | Output-mode resolution and persistence (watchdog scratch) |
| `hid/pico/src/ph_usb.c`          | USB device descriptor, VID/PID, HID report dispatch |
| `hid/pico/src/ph_usb_kbd.c`      | Keyboard HID report descriptor (8/1/6 layout) |
| `hid/pico/src/ph_usb_mouse.c`    | Absolute + relative mouse HID report descriptors |
| `hid/pico/src/ph_usb_keymap.h.mako` | Generator template for MCU code → USB usage table |
| `hid/pico/src/ph_tools.h`        | CRC-16 reference implementation (poly 0xA001, init 0xFFFF) |
| `kvmd/plugins/hid/serial.py`     | Host UART driver — confirms baud and frame size |
| `kvmd/plugins/hid/_mcu/proto.py` | Host request builders — confirms byte layouts (`>BBxxx`, `>Bhh`, `>Bbbxx`, etc.) |
| `kvmd/plugins/hid/_mcu/__init__.py` | Host state-machine, pong parsing, retry/REPEAT logic |
| `kvmd/bitbang.py`                | Host CRC-16 reference implementation |
| `kvmd/mouse.py`                  | MouseRange ±32768, MouseDelta ±127 — argument bounds |
| `kvmd/keymap.csv`                | The 116-entry web/evdev/MCU/USB keycode table |
| `configs/os/udev/v0-hdmi-rpi3.rules` | Confirms `/dev/kvmd-hid → ttyAMA0` (Pi UART) |
| `pikvm/docs/pico_hid.md`         | Official wiring/config guide (note: GP0/GP1 claim is wrong; trust the firmware) |
