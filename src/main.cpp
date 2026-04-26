/*
 * ESP32-S3 PiKVM Pico HID — Drop-in Replacement Firmware
 *
 * Wire-compatible with kvmd's Pico HID serial protocol.
 * Protocol: 115200 8N1, 8-byte fixed packets, CRC-16/MODBUS.
 *
 * PIN ASSIGNMENT (ESP32-S3):
 *   USB-OTG:  GPIO19 (D-) / GPIO20 (D+)  — ACTIVE, do NOT use for anything else
 *   UART1 RX: GPIO16  ← connect to Pi GPIO14 (UART TX)
 *   UART1 TX: GPIO17  → connect to Pi GPIO15 (UART RX)
 *   GND:      any GND  — connect to Pi GND
 *   5V:       5V pin   ← connect to Pi 5V (powers ESP32-S3 via USB or VIN)
 *
 * VID/PID: 0x1209 / 0xEDA2 ("PiKVM HID")
 */

#include <Arduino.h>
#include <Preferences.h>
#include "USB.h"
#include "tusb.h"
#include "esp_mac.h"

// ─── Pin Configuration ─────────────────────────────────────────────
#define UART_RX_PIN   16
#define UART_TX_PIN   17
#define UART_NUM      1
#define UART_BAUD     115200

// ─── Protocol Constants ────────────────────────────────────────────
#define MAGIC_REQ           0x33
#define MAGIC_RESP          0x34
#define PACKET_SIZE         8
#define INTERBYTE_TIMEOUT_MS 100

// Commands
#define CMD_PING            0x01
#define CMD_REPEAT          0x02
#define CMD_SET_KBD         0x03
#define CMD_SET_MOUSE       0x04
#define CMD_SET_CONNECTED   0x05
#define CMD_CLEAR_HID       0x10
#define CMD_KBD_KEY         0x11
#define CMD_MOUSE_ABS       0x12
#define CMD_MOUSE_BUTTON    0x13
#define CMD_MOUSE_WHEEL     0x14
#define CMD_MOUSE_REL       0x15

// Response codes
#define RESP_NONE           0x24
#define RESP_CRC_ERROR      0x40
#define RESP_INVALID_ERROR  0x45
#define RESP_TIMEOUT_ERROR  0x48
#define PONG_OK             0x80
#define PONG_RESET_REQUIRED 0x40
#define PONG_MOUSE_OFFLINE  0x10
#define PONG_KBD_OFFLINE    0x08
#define PONG_NUM            0x04
#define PONG_SCROLL         0x02
#define PONG_CAPS           0x01

// Output flags
#define OUT1_DYNAMIC        0x80
#define OUT1_KBD_USB        0x01
#define OUT1_KBD_PS2        0x03
#define OUT1_MOUSE_USB_ABS  0x08
#define OUT1_MOUSE_USB_REL  0x10
#define OUT1_MOUSE_PS2      0x18
#define OUT1_MOUSE_USB_W98  0x20
#define OUT1_KBD_MASK       0x07
#define OUT1_MOUSE_MASK     0x38

#define OUT2_CONNECTABLE    0x80
#define OUT2_CONNECTED      0x40
#define OUT2_HAS_USB_W98    0x04
#define OUT2_HAS_PS2        0x02
#define OUT2_HAS_USB        0x01

// Mouse button select/state masks (byte 2 = main buttons)
#define BTN_LEFT_SELECT     0x80
#define BTN_RIGHT_SELECT    0x40
#define BTN_MIDDLE_SELECT   0x20
#define BTN_LEFT_STATE      0x08
#define BTN_RIGHT_STATE     0x04
#define BTN_MIDDLE_STATE    0x02
// Mouse button select/state masks (byte 3 = extra buttons)
#define BTN_BACKWARD_SELECT 0x80
#define BTN_FORWARD_SELECT  0x40
#define BTN_BACKWARD_STATE  0x08
#define BTN_FORWARD_STATE   0x04

// ─── TinyUSB Callbacks ──────────────────────────────────────────────

static char serial_str[18];

extern "C" {

uint8_t const *tud_descriptor_device_cb(void) {
    static tusb_desc_device_t const desc_device = {
        .bLength            = sizeof(tusb_desc_device_t),
        .bDescriptorType    = TUSB_DESC_DEVICE,
        .bcdUSB             = 0x0200,
        .bDeviceClass       = 0x00,
        .bDeviceSubClass    = 0x00,
        .bDeviceProtocol    = 0x00,
        .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor           = 0x1209,
        .idProduct          = 0xEDA2,
        .bcdDevice          = 0x0100,
        .iManufacturer      = 1,
        .iProduct           = 2,
        .iSerialNumber      = 3,
        .bNumConfigurations = 1,
    };
    return (uint8_t const *)&desc_device;
}

// Configuration descriptor: 3 HID interfaces (keyboard, abs mouse, rel mouse)
// Per-interface report descriptors (no Report-ID prefix — required for boot protocol)
static const uint8_t desc_kbd_report[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
    0x95, 0x05, 0x75, 0x01, 0x05, 0x08,
    0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x26, 0xFF, 0x00,
    0x05, 0x07, 0x19, 0x00, 0x2A, 0xFF, 0x00, 0x81, 0x00,
    0xC0
};

static const uint8_t desc_abs_mouse_report[] = {
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,
    0x09, 0x01, 0xA1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x08,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x08, 0x75, 0x01, 0x81, 0x02,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
    0x16, 0x00, 0x00, 0x26, 0xFF, 0x7F,
    0x75, 0x10, 0x95, 0x02, 0x81, 0x02,
    0x09, 0x38, 0x15, 0x81, 0x25, 0x7F,
    0x75, 0x08, 0x95, 0x01, 0x81, 0x06,
    0xC0, 0xC0
};

static const uint8_t desc_rel_mouse_report[] = {
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,
    0x09, 0x01, 0xA1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x08,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x08, 0x75, 0x01, 0x81, 0x02,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7F,
    0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
    0xC0, 0xC0
};

// Endpoint addresses
#define EPNUM_KBD       0x81
#define EPNUM_KBD_OUT   0x01
#define EPNUM_ABS_MOUSE 0x82
#define EPNUM_REL_MOUSE 0x83

// Interface numbers
#define ITF_KBD         0
#define ITF_ABS_MOUSE   1
#define ITF_REL_MOUSE   2

static uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 3, 0, TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN + TUD_HID_DESC_LEN + TUD_HID_DESC_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_INOUT_DESCRIPTOR(ITF_KBD, 0, HID_ITF_PROTOCOL_KEYBOARD, sizeof(desc_kbd_report), EPNUM_KBD_OUT, EPNUM_KBD, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(ITF_ABS_MOUSE, 0, HID_ITF_PROTOCOL_NONE, sizeof(desc_abs_mouse_report), EPNUM_ABS_MOUSE, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(ITF_REL_MOUSE, 0, HID_ITF_PROTOCOL_MOUSE, sizeof(desc_rel_mouse_report), EPNUM_REL_MOUSE, CFG_TUD_HID_EP_BUFSIZE, 1),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    switch (instance) {
        case ITF_KBD:       return desc_kbd_report;
        case ITF_ABS_MOUSE: return desc_abs_mouse_report;
        case ITF_REL_MOUSE: return desc_rel_mouse_report;
        default:            return desc_kbd_report;
    }
}

static uint16_t const string_desc_langid[] = { (uint16_t)(TUSB_DESC_STRING << 8 | 4), 0x0409 };
static const char *string_desc_arr[] = {
    NULL,           // 0: language (handled separately)
    "PiKVM",        // 1: Manufacturer
    "PiKVM HID",    // 2: Product
    serial_str,     // 3: Serial
};

static uint16_t _desc_str[33];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    if (index == 0) return string_desc_langid;

    const char *str;
    if (index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
        str = string_desc_arr[index];
    } else {
        return NULL;
    }
    if (!str) return NULL;

    uint8_t chr_count = strlen(str);
    if (chr_count > 31) chr_count = 31;

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    for (uint8_t i = 0; i < chr_count; i++) {
        _desc_str[1 + i] = str[i];
    }
    return _desc_str;
}

} // extern "C"

// ─── MCU Code → USB HID Usage Lookup Table ──────────────────────────
// Source: kvmd/keymap.csv (MCU codes 1..115 → USB HID usage codes)
// Modifier keys (MCU 77-84) return 0xE0..0xE7 (HID modifier usage range)
// Source: pikvm/kvmd keymap.csv (fetched from master, verified 2026-04-25)
// Modifiers (MCU 77-84) map to USB HID 0xE0-0xE7 and are handled specially
static const uint8_t mcu_to_usb[116] = {
    0x00,  //   0: (unused)
    0x04,  //   1: KeyA
    0x05,  //   2: KeyB
    0x06,  //   3: KeyC
    0x07,  //   4: KeyD
    0x08,  //   5: KeyE
    0x09,  //   6: KeyF
    0x0A,  //   7: KeyG
    0x0B,  //   8: KeyH
    0x0C,  //   9: KeyI
    0x0D,  //  10: KeyJ
    0x0E,  //  11: KeyK
    0x0F,  //  12: KeyL
    0x10,  //  13: KeyM
    0x11,  //  14: KeyN
    0x12,  //  15: KeyO
    0x13,  //  16: KeyP
    0x14,  //  17: KeyQ
    0x15,  //  18: KeyR
    0x16,  //  19: KeyS
    0x17,  //  20: KeyT
    0x18,  //  21: KeyU
    0x19,  //  22: KeyV
    0x1A,  //  23: KeyW
    0x1B,  //  24: KeyX
    0x1C,  //  25: KeyY
    0x1D,  //  26: KeyZ
    0x1E,  //  27: Digit1
    0x1F,  //  28: Digit2
    0x20,  //  29: Digit3
    0x21,  //  30: Digit4
    0x22,  //  31: Digit5
    0x23,  //  32: Digit6
    0x24,  //  33: Digit7
    0x25,  //  34: Digit8
    0x26,  //  35: Digit9
    0x27,  //  36: Digit0
    0x28,  //  37: Enter
    0x29,  //  38: Escape
    0x2A,  //  39: Backspace
    0x2B,  //  40: Tab
    0x2C,  //  41: Space
    0x2D,  //  42: Minus
    0x2E,  //  43: Equal
    0x2F,  //  44: BracketLeft
    0x30,  //  45: BracketRight
    0x31,  //  46: Backslash
    0x33,  //  47: Semicolon          (NOTE: no 0x32 in keymap — skipped)
    0x34,  //  48: Quote
    0x35,  //  49: Backquote
    0x36,  //  50: Comma
    0x37,  //  51: Period
    0x38,  //  52: Slash
    0x39,  //  53: CapsLock
    0x3A,  //  54: F1
    0x3B,  //  55: F2
    0x3C,  //  56: F3
    0x3D,  //  57: F4
    0x3E,  //  58: F5
    0x3F,  //  59: F6
    0x40,  //  60: F7
    0x41,  //  61: F8
    0x42,  //  62: F9
    0x43,  //  63: F10
    0x44,  //  64: F11
    0x45,  //  65: F12
    0x46,  //  66: PrintScreen
    0x49,  //  67: Insert
    0x4A,  //  68: Home
    0x4B,  //  69: PageUp
    0x4C,  //  70: Delete
    0x4D,  //  71: End
    0x4E,  //  72: PageDown
    0x4F,  //  73: ArrowRight
    0x50,  //  74: ArrowLeft
    0x51,  //  75: ArrowDown
    0x52,  //  76: ArrowUp
    0xE0,  //  77: ControlLeft         (modifier ^0x01 → 0xE0)
    0xE1,  //  78: ShiftLeft           (modifier ^0x02 → 0xE1)
    0xE2,  //  79: AltLeft             (modifier ^0x04 → 0xE2)
    0xE3,  //  80: MetaLeft            (modifier ^0x08 → 0xE3)
    0xE4,  //  81: ControlRight        (modifier ^0x10 → 0xE4)
    0xE5,  //  82: ShiftRight          (modifier ^0x20 → 0xE5)
    0xE6,  //  83: AltRight            (modifier ^0x40 → 0xE6)
    0xE7,  //  84: MetaRight           (modifier ^0x80 → 0xE7)
    0x48,  //  85: Pause
    0x47,  //  86: ScrollLock
    0x53,  //  87: NumLock
    0x65,  //  88: ContextMenu
    0x54,  //  89: NumpadDivide
    0x55,  //  90: NumpadMultiply
    0x56,  //  91: NumpadSubtract
    0x57,  //  92: NumpadAdd
    0x58,  //  93: NumpadEnter
    0x59,  //  94: Numpad1
    0x5A,  //  95: Numpad2
    0x5B,  //  96: Numpad3
    0x5C,  //  97: Numpad4
    0x5D,  //  98: Numpad5
    0x5E,  //  99: Numpad6
    0x5F,  // 100: Numpad7
    0x60,  // 101: Numpad8
    0x61,  // 102: Numpad9
    0x62,  // 103: Numpad0
    0x63,  // 104: NumpadDecimal
    0x66,  // 105: Power
    0x64,  // 106: IntlBackslash
    0x89,  // 107: IntlYen
    0x87,  // 108: IntlRo
    0x88,  // 109: KanaMode
    0x8A,  // 110: Convert
    0x8B,  // 111: NonConvert
    0x7F,  // 112: AudioVolumeMute
    0x80,  // 113: AudioVolumeUp
    0x81,  // 114: AudioVolumeDown
    0x6F,  // 115: F20
};

// ─── Global State ──────────────────────────────────────────────────
static Preferences prefs;

// Active output modes
static uint8_t active_kbd_mode   = OUT1_KBD_USB;       // 0x01 = USB keyboard
static uint8_t active_mouse_mode = OUT1_MOUSE_USB_ABS; // 0x08 = USB absolute mouse

// Keyboard HID state
static uint8_t kbd_mods = 0;
static uint8_t kbd_keys[6] = {0};
static uint8_t kbd_leds = 0;

// Mouse state
static uint8_t  mouse_buttons = 0;
static int16_t  mouse_abs_x = 0;
static int16_t  mouse_abs_y = 0;

// Protocol state
static uint8_t prev_resp_code = RESP_NONE;
static bool    reset_required = false;
static unsigned long reset_scheduled_at = 0;

// UART RX buffer
static uint8_t  rx_buf[PACKET_SIZE];
static uint8_t  rx_index = 0;
static unsigned long rx_last_byte_ms = 0;

// ─── CRC-16/MODBUS ─────────────────────────────────────────────────
static uint16_t crc16_modbus(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ─── USB HID Functions ─────────────────────────────────────────────

static void send_kbd_report() {
    if (!tud_hid_n_ready(ITF_KBD)) return;
    uint8_t report[8];
    report[0] = kbd_mods;
    report[1] = 0; // reserved
    memcpy(&report[2], kbd_keys, 6);
    tud_hid_n_report(ITF_KBD, 0, report, sizeof(report));
}

static void send_abs_mouse_report(int8_t wheel) {
    if (!tud_hid_n_ready(ITF_ABS_MOUSE)) return;
    // Map from protocol range [-32768..32767] to descriptor range [0..32767]
    uint16_t x = (uint16_t)(((int32_t)mouse_abs_x + 32768) / 2);
    uint16_t y = (uint16_t)(((int32_t)mouse_abs_y + 32768) / 2);
    uint8_t report[6];
    report[0] = mouse_buttons;
    report[1] = x & 0xFF;        // little-endian
    report[2] = (x >> 8) & 0xFF;
    report[3] = y & 0xFF;
    report[4] = (y >> 8) & 0xFF;
    report[5] = (uint8_t)wheel;
    tud_hid_n_report(ITF_ABS_MOUSE, 0, report, sizeof(report));
}

static void send_rel_mouse_report(int8_t dx, int8_t dy, int8_t wheel) {
    if (!tud_hid_n_ready(ITF_REL_MOUSE)) return;
    uint8_t report[4];
    report[0] = mouse_buttons;
    report[1] = (uint8_t)dx;
    report[2] = (uint8_t)dy;
    report[3] = (uint8_t)wheel;
    tud_hid_n_report(ITF_REL_MOUSE, 0, report, sizeof(report));
}

static void clear_all_hid() {
    kbd_mods = 0;
    memset(kbd_keys, 0, sizeof(kbd_keys));
    mouse_buttons = 0;
    send_kbd_report();
    if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_ABS) {
        send_abs_mouse_report(0);
    } else if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_REL) {
        send_rel_mouse_report(0, 0, 0);
    }
}

// ─── Keyboard Key Handling ──────────────────────────────────────────

static void handle_kbd_key(uint8_t mcu_code, uint8_t state) {
    if (mcu_code == 0 || mcu_code > 115) return;
    uint8_t usb_key = mcu_to_usb[mcu_code];
    if (usb_key == 0) return;

    if (usb_key >= 0xE0 && usb_key <= 0xE7) {
        uint8_t mod_bit = 1 << (usb_key & 0x07);
        if (state) {
            kbd_mods |= mod_bit;
        } else {
            kbd_mods &= ~mod_bit;
        }
    } else {
        if (state) {
            // Find empty slot
            for (int i = 0; i < 6; i++) {
                if (kbd_keys[i] == usb_key) break; // already pressed
                if (kbd_keys[i] == 0) {
                    kbd_keys[i] = usb_key;
                    break;
                }
            }
        } else {
            for (int i = 0; i < 6; i++) {
                if (kbd_keys[i] == usb_key) {
                    kbd_keys[i] = 0;
                    break;
                }
            }
        }
    }
    send_kbd_report();
}

// ─── Mouse Button Handling ──────────────────────────────────────────

static void handle_mouse_button(uint8_t main_byte, uint8_t extra_byte) {
    // Main buttons: left=bit0, right=bit1, middle=bit2
    if (main_byte & BTN_LEFT_SELECT) {
        if (main_byte & BTN_LEFT_STATE) mouse_buttons |= 0x01;
        else mouse_buttons &= ~0x01;
    }
    if (main_byte & BTN_RIGHT_SELECT) {
        if (main_byte & BTN_RIGHT_STATE) mouse_buttons |= 0x02;
        else mouse_buttons &= ~0x02;
    }
    if (main_byte & BTN_MIDDLE_SELECT) {
        if (main_byte & BTN_MIDDLE_STATE) mouse_buttons |= 0x04;
        else mouse_buttons &= ~0x04;
    }
    // Extra buttons: backward=bit3, forward=bit4
    if (extra_byte & BTN_BACKWARD_SELECT) {
        if (extra_byte & BTN_BACKWARD_STATE) mouse_buttons |= 0x08;
        else mouse_buttons &= ~0x08;
    }
    if (extra_byte & BTN_FORWARD_SELECT) {
        if (extra_byte & BTN_FORWARD_STATE) mouse_buttons |= 0x10;
        else mouse_buttons &= ~0x10;
    }
    // Send on whichever mouse mode is active
    if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_ABS) {
        send_abs_mouse_report(0);
    } else if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_REL) {
        send_rel_mouse_report(0, 0, 0);
    }
}

// ─── NVS Persistence ───────────────────────────────────────────────

static void load_output_modes() {
    prefs.begin("pikvm", true); // read-only
    active_kbd_mode = prefs.getUChar("kbd_mode", OUT1_KBD_USB);
    active_mouse_mode = prefs.getUChar("mouse_mode", OUT1_MOUSE_USB_ABS);
    prefs.end();
}

static void save_output_modes() {
    prefs.begin("pikvm", false);
    prefs.putUChar("kbd_mode", active_kbd_mode);
    prefs.putUChar("mouse_mode", active_mouse_mode);
    prefs.end();
}

// ─── Protocol Response Builder ─────────────────────────────────────

static void send_response(uint8_t code) {
    if (code == 0) {
        code = prev_resp_code; // REPEAT: reuse last
    } else {
        prev_resp_code = code;
    }

    uint8_t resp[PACKET_SIZE] = {0};
    resp[0] = MAGIC_RESP;

    if (code & PONG_OK) {
        resp[1] = PONG_OK;
        if (reset_required) {
            resp[1] |= PONG_RESET_REQUIRED;
        }
        // Keyboard LED bits from USB host
        if (kbd_leds & 0x01) resp[1] |= PONG_NUM;
        if (kbd_leds & 0x02) resp[1] |= PONG_CAPS;
        if (kbd_leds & 0x04) resp[1] |= PONG_SCROLL;
        // Offline detection
        if (!tud_hid_n_ready(ITF_KBD)) resp[1] |= PONG_KBD_OFFLINE;
        uint8_t mm = active_mouse_mode & OUT1_MOUSE_MASK;
        bool mouse_off = false;
        if (mm == OUT1_MOUSE_USB_ABS)      mouse_off = !tud_hid_n_ready(ITF_ABS_MOUSE);
        else if (mm == OUT1_MOUSE_USB_REL) mouse_off = !tud_hid_n_ready(ITF_REL_MOUSE);
        if (mouse_off) resp[1] |= PONG_MOUSE_OFFLINE;
        // OUTPUTS1: dynamic flag + active modes
        resp[2] = OUT1_DYNAMIC | (active_kbd_mode & OUT1_KBD_MASK) | (active_mouse_mode & OUT1_MOUSE_MASK);
        // OUTPUTS2: available capabilities
        resp[3] = OUT2_HAS_USB;
    } else {
        resp[1] = code; // bare error code
    }

    uint16_t crc = crc16_modbus(resp, 6);
    resp[6] = (crc >> 8) & 0xFF; // big-endian: high byte first
    resp[7] = crc & 0xFF;

    Serial1.write(resp, PACKET_SIZE);
}

// ─── Command Dispatcher ────────────────────────────────────────────

static void handle_packet(const uint8_t *data) {
    // Verify magic
    if (data[0] != MAGIC_REQ) {
        send_response(RESP_CRC_ERROR);
        return;
    }

    // Verify CRC
    uint16_t rx_crc = ((uint16_t)data[6] << 8) | data[7];
    uint16_t calc_crc = crc16_modbus(data, 6);
    if (rx_crc != calc_crc) {
        send_response(RESP_CRC_ERROR);
        return;
    }

    uint8_t cmd = data[1];
    const uint8_t *args = data + 2; // 4 bytes of payload

    switch (cmd) {
        case CMD_PING:
            send_response(PONG_OK);
            break;

        case CMD_REPEAT:
            send_response(0); // re-send previous
            break;

        case CMD_SET_KBD: {
            uint8_t new_mode = args[0] & OUT1_KBD_MASK;
            active_kbd_mode = new_mode;
            save_output_modes();
            reset_required = true;
            send_response(PONG_OK);
            reset_scheduled_at = millis();
            break;
        }

        case CMD_SET_MOUSE: {
            uint8_t new_mode = args[0] & OUT1_MOUSE_MASK;
            active_mouse_mode = new_mode;
            save_output_modes();
            reset_required = true;
            send_response(PONG_OK);
            reset_scheduled_at = millis();
            break;
        }

        case CMD_SET_CONNECTED:
            send_response(PONG_OK); // no-op, per Pico firmware
            break;

        case CMD_CLEAR_HID:
            clear_all_hid();
            send_response(PONG_OK);
            break;

        case CMD_KBD_KEY:
            if ((active_kbd_mode & OUT1_KBD_MASK) == OUT1_KBD_USB) {
                handle_kbd_key(args[0], args[1]);
            }
            send_response(PONG_OK);
            break;

        case CMD_MOUSE_ABS:
            if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_ABS) {
                mouse_abs_x = (int16_t)(((uint16_t)args[0] << 8) | args[1]);
                mouse_abs_y = (int16_t)(((uint16_t)args[2] << 8) | args[3]);
                send_abs_mouse_report(0);
            }
            send_response(PONG_OK);
            break;

        case CMD_MOUSE_BUTTON:
            handle_mouse_button(args[0], args[1]);
            send_response(PONG_OK);
            break;

        case CMD_MOUSE_WHEEL:
            if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_ABS) {
                send_abs_mouse_report((int8_t)args[1]); // args[0]=h (ignored), args[1]=v
            } else if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_REL) {
                send_rel_mouse_report(0, 0, (int8_t)args[1]);
            }
            send_response(PONG_OK);
            break;

        case CMD_MOUSE_REL:
            if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_REL) {
                send_rel_mouse_report((int8_t)args[0], (int8_t)args[1], 0);
            }
            send_response(PONG_OK);
            break;

        default:
            send_response(RESP_INVALID_ERROR);
            break;
    }
}

// ─── TinyUSB LED Callback (keyboard LED output report) ─────────────

extern "C" void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    (void)report_id;
    if (instance == ITF_KBD && report_type == HID_REPORT_TYPE_OUTPUT && bufsize >= 1) {
        kbd_leds = buffer[0];
    }
}

extern "C" uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;
}

// ─── Setup ──────────────────────────────────────────────────────────

void setup() {
    // Generate serial string from MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(serial_str, sizeof(serial_str), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Load persisted output modes from NVS
    load_output_modes();

    // Configure USB identity before init
    USB.VID(0x1209);
    USB.PID(0xEDA2);
    USB.manufacturerName("PiKVM");
    USB.productName("PiKVM HID");
    USB.serialNumber(serial_str);
    USB.begin();

    // Initialize UART to PiKVM host
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // Wait for USB enumeration
    delay(500);
}

// ─── Main Loop ──────────────────────────────────────────────────────

void loop() {
    // Handle pending self-reset (100ms after SET_KBD/SET_MOUSE ack)
    if (reset_required && reset_scheduled_at > 0) {
        if (millis() - reset_scheduled_at >= 100) {
            Serial1.flush();
            esp_restart();
        }
        return; // stop processing UART while waiting to reset
    }

    // Process UART bytes
    while (Serial1.available()) {
        rx_buf[rx_index] = Serial1.read();
        if (rx_index == PACKET_SIZE - 1) {
            handle_packet(rx_buf);
            rx_index = 0;
        } else {
            rx_last_byte_ms = millis();
            rx_index++;
        }
    }

    // Inter-byte timeout check
    if (rx_index > 0 && (millis() - rx_last_byte_ms >= INTERBYTE_TIMEOUT_MS)) {
        send_response(RESP_TIMEOUT_ERROR);
        rx_index = 0;
    }
}
