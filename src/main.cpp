/*
 * ESP32-S3 PiKVM Pico HID — Drop-in Replacement Firmware
 *
 * Wire-compatible with kvmd's Pico HID serial protocol.
 * Protocol: 115200 8N1, 8-byte fixed packets, CRC-16/MODBUS.
 *
 * PIN ASSIGNMENT (ESP32-S3):
 *   USB-OTG:  GPIO19 (D-) / GPIO20 (D+)
 *   UART1 RX: GPIO16  <- Pi GPIO14 (TX)
 *   UART1 TX: GPIO17  -> Pi GPIO15 (RX)
 *   UART0:    CH343 debug port (Serial monitor)
 */

#include <Arduino.h>
#include <Preferences.h>
#include "USB.h"
#include "USBHIDKeyboard.h"
#include "USBHIDMouse.h"
#include "USBHID.h"
#include "tusb.h"
#include "esp_mac.h"

#if __has_include("config.h")
#  include "config.h"
#  include <WiFi.h>
#  include <ArduinoOTA.h>
#  define HAVE_WIFI 1
#else
#  define HAVE_WIFI 0
#endif

#define UART_RX_PIN   16
#define UART_TX_PIN   17
#define UART_BAUD     115200

#define MAGIC_REQ           0x33
#define MAGIC_RESP          0x34
#define PACKET_SIZE         8
#define INTERBYTE_TIMEOUT_MS 100

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
#define CMD_HUMANIZE        0x20

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

#define OUT1_DYNAMIC        0x80
#define OUT1_KBD_USB        0x01
#define OUT1_KBD_MASK       0x07
#define OUT1_MOUSE_USB_ABS  0x08
#define OUT1_MOUSE_USB_REL  0x10
#define OUT1_MOUSE_MASK     0x38

#define OUT2_CONNECTABLE    0x80
#define OUT2_CONNECTED      0x40
#define OUT2_HAS_USB        0x01

#define BTN_LEFT_SELECT     0x80
#define BTN_RIGHT_SELECT    0x40
#define BTN_MIDDLE_SELECT   0x20
#define BTN_LEFT_STATE      0x08
#define BTN_RIGHT_STATE     0x04
#define BTN_MIDDLE_STATE    0x02
#define BTN_BACKWARD_SELECT 0x80
#define BTN_FORWARD_SELECT  0x40
#define BTN_BACKWARD_STATE  0x08
#define BTN_FORWARD_STATE   0x04

// ---- Absolute Mouse HID Device (not in Arduino standard library) ----
static const uint8_t abs_mouse_report_desc[] = {
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

#define HID_REPORT_ID_ABS_MOUSE 3

class AbsMouse : public USBHIDDevice {
public:
    USBHID hid;
    AbsMouse() {}
    void begin() { hid.addDevice(this, sizeof(abs_mouse_report_desc)); }
    uint16_t _onGetDescriptor(uint8_t *dst) {
        memcpy(dst, abs_mouse_report_desc, sizeof(abs_mouse_report_desc));
        return sizeof(abs_mouse_report_desc);
    }
    bool send(uint8_t buttons, uint16_t x, uint16_t y, int8_t wheel) {
        uint8_t r[6] = { buttons, (uint8_t)(x & 0xFF), (uint8_t)(x >> 8),
                         (uint8_t)(y & 0xFF), (uint8_t)(y >> 8), (uint8_t)wheel };
        return hid.SendReport(HID_REPORT_ID_ABS_MOUSE, r, sizeof(r));
    }
};

// ---- MCU Code -> USB HID Usage Lookup ----
static const uint8_t mcu_to_usb[116] = {
    0x00, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
    0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
    0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20,
    0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A,
    0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x33, 0x34, 0x35,
    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x49, 0x4A, 0x4B,
    0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0xE0, 0xE1, 0xE2,
    0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0x48, 0x47, 0x53, 0x65, 0x54,
    0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E,
    0x5F, 0x60, 0x61, 0x62, 0x63, 0x66, 0x64, 0x89, 0x87, 0x88,
    0x8A, 0x8B, 0x7F, 0x80, 0x81, 0x6F,
};

// ---- Global State ----
static Preferences prefs;
static USBHIDKeyboard Keyboard;
static USBHIDMouse RelMouse;
static AbsMouse AbsMouseDev;

static uint8_t active_kbd_mode   = OUT1_KBD_USB;
static uint8_t active_mouse_mode = OUT1_MOUSE_USB_ABS;
static uint8_t kbd_mods = 0, kbd_keys[6] = {0}, kbd_leds = 0;
static uint8_t mouse_buttons = 0;
static int16_t mouse_abs_x = 0, mouse_abs_y = 0;
static bool humanize_enabled = false;
static uint8_t prev_resp_code = RESP_NONE;
static bool reset_required = false;
static unsigned long reset_scheduled_at = 0;
static uint8_t rx_buf[PACKET_SIZE];
static uint8_t rx_index = 0;
static unsigned long rx_last_byte_ms = 0;
static uint32_t stat_packets_ok = 0, stat_crc_errors = 0, stat_timeouts = 0;
static unsigned long boot_ms = 0, last_status_ms = 0;
static char serial_str[18];

#if HAVE_WIFI
static WiFiServer tcp_server(80);
static bool wifi_was_connected = false;
#endif

// ---- CRC-16/MODBUS ----
static uint16_t crc16_modbus(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

// ---- USB HID send functions ----
static void send_kbd_report() {
    if (!tud_mounted()) return;
    if (humanize_enabled) delayMicroseconds(esp_random() % 3000);
    KeyReport report;
    report.modifiers = kbd_mods;
    report.reserved = 0;
    memcpy(report.keys, kbd_keys, 6);
    Keyboard.sendReport(&report);
}

static void send_abs_mouse_report(int8_t wheel) {
    if (!tud_mounted()) return;
    if (humanize_enabled) delayMicroseconds(esp_random() % 3000);
    uint16_t x = (uint16_t)(((int32_t)mouse_abs_x + 32768) / 2);
    uint16_t y = (uint16_t)(((int32_t)mouse_abs_y + 32768) / 2);
    AbsMouseDev.send(mouse_buttons, x, y, wheel);
}

static void send_rel_mouse_report(int8_t dx, int8_t dy, int8_t wheel) {
    if (!tud_mounted()) return;
    if (humanize_enabled) delayMicroseconds(esp_random() % 3000);
    RelMouse.move(dx, dy, wheel);
}

static void clear_all_hid() {
    kbd_mods = 0;
    memset(kbd_keys, 0, sizeof(kbd_keys));
    mouse_buttons = 0;
    send_kbd_report();
    if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_ABS) send_abs_mouse_report(0);
    else if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_REL) send_rel_mouse_report(0, 0, 0);
}

static void handle_kbd_key(uint8_t mcu_code, uint8_t state) {
    if (mcu_code == 0 || mcu_code > 115) return;
    uint8_t usb_key = mcu_to_usb[mcu_code];
    if (usb_key == 0) return;
    if (usb_key >= 0xE0 && usb_key <= 0xE7) {
        uint8_t mod_bit = 1 << (usb_key & 0x07);
        if (state) kbd_mods |= mod_bit; else kbd_mods &= ~mod_bit;
    } else {
        if (state) { for (int i = 0; i < 6; i++) { if (kbd_keys[i] == usb_key) break; if (kbd_keys[i] == 0) { kbd_keys[i] = usb_key; break; } } }
        else { for (int i = 0; i < 6; i++) { if (kbd_keys[i] == usb_key) { kbd_keys[i] = 0; break; } } }
    }
    send_kbd_report();
}

static void handle_mouse_button(uint8_t m, uint8_t e) {
    if (m & BTN_LEFT_SELECT)   { if (m & BTN_LEFT_STATE) mouse_buttons |= 0x01; else mouse_buttons &= ~0x01; }
    if (m & BTN_RIGHT_SELECT)  { if (m & BTN_RIGHT_STATE) mouse_buttons |= 0x02; else mouse_buttons &= ~0x02; }
    if (m & BTN_MIDDLE_SELECT) { if (m & BTN_MIDDLE_STATE) mouse_buttons |= 0x04; else mouse_buttons &= ~0x04; }
    if (e & BTN_BACKWARD_SELECT) { if (e & BTN_BACKWARD_STATE) mouse_buttons |= 0x08; else mouse_buttons &= ~0x08; }
    if (e & BTN_FORWARD_SELECT)  { if (e & BTN_FORWARD_STATE) mouse_buttons |= 0x10; else mouse_buttons &= ~0x10; }
    if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_ABS) send_abs_mouse_report(0);
    else if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_REL) send_rel_mouse_report(0, 0, 0);
}

static void load_output_modes() { prefs.begin("pikvm", true); active_kbd_mode = prefs.getUChar("kbd_mode", OUT1_KBD_USB); active_mouse_mode = prefs.getUChar("mouse_mode", OUT1_MOUSE_USB_ABS); humanize_enabled = prefs.getBool("humanize", false); prefs.end(); }
static void save_output_modes() { prefs.begin("pikvm", false); prefs.putUChar("kbd_mode", active_kbd_mode); prefs.putUChar("mouse_mode", active_mouse_mode); prefs.putBool("humanize", humanize_enabled); prefs.end(); }

// ---- Protocol Response ----
static void send_response(uint8_t code) {
    if (code == 0) code = prev_resp_code; else prev_resp_code = code;
    uint8_t resp[PACKET_SIZE] = {0};
    resp[0] = MAGIC_RESP;
    if (code & PONG_OK) {
        resp[1] = PONG_OK;
        if (reset_required) resp[1] |= PONG_RESET_REQUIRED;
        if (kbd_leds & 0x01) resp[1] |= PONG_NUM;
        if (kbd_leds & 0x02) resp[1] |= PONG_CAPS;
        if (kbd_leds & 0x04) resp[1] |= PONG_SCROLL;
        if (!tud_mounted()) resp[1] |= PONG_KBD_OFFLINE | PONG_MOUSE_OFFLINE;
        resp[2] = OUT1_DYNAMIC | (active_kbd_mode & OUT1_KBD_MASK) | (active_mouse_mode & OUT1_MOUSE_MASK);
        resp[3] = OUT2_CONNECTABLE | OUT2_HAS_USB;
        if (tud_mounted()) resp[3] |= OUT2_CONNECTED;
    } else {
        resp[1] = code;
    }
    uint16_t crc = crc16_modbus(resp, 6);
    resp[6] = (crc >> 8) & 0xFF;
    resp[7] = crc & 0xFF;
    Serial1.write(resp, PACKET_SIZE);
}

// ---- Command Dispatcher ----
static void handle_packet(const uint8_t *data) {
    if (data[0] != MAGIC_REQ) { send_response(RESP_CRC_ERROR); return; }
    uint16_t rx_crc = ((uint16_t)data[6] << 8) | data[7];
    if (rx_crc != crc16_modbus(data, 6)) { stat_crc_errors++; send_response(RESP_CRC_ERROR); return; }
    stat_packets_ok++;
    uint8_t cmd = data[1];
    const uint8_t *args = data + 2;
    switch (cmd) {
        case CMD_PING: send_response(PONG_OK); break;
        case CMD_REPEAT: send_response(0); break;
        case CMD_SET_KBD: active_kbd_mode = args[0] & OUT1_KBD_MASK; save_output_modes(); reset_required = true; send_response(PONG_OK); reset_scheduled_at = millis(); break;
        case CMD_SET_MOUSE: active_mouse_mode = args[0] & OUT1_MOUSE_MASK; save_output_modes(); reset_required = true; send_response(PONG_OK); reset_scheduled_at = millis(); break;
        case CMD_SET_CONNECTED: send_response(PONG_OK); break;
        case CMD_CLEAR_HID: clear_all_hid(); send_response(PONG_OK); break;
        case CMD_KBD_KEY: if ((active_kbd_mode & OUT1_KBD_MASK) == OUT1_KBD_USB) handle_kbd_key(args[0], args[1]); send_response(PONG_OK); break;
        case CMD_MOUSE_ABS:
            if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_ABS) { mouse_abs_x = (int16_t)(((uint16_t)args[0] << 8) | args[1]); mouse_abs_y = (int16_t)(((uint16_t)args[2] << 8) | args[3]); send_abs_mouse_report(0); }
            send_response(PONG_OK); break;
        case CMD_MOUSE_BUTTON: handle_mouse_button(args[0], args[1]); send_response(PONG_OK); break;
        case CMD_MOUSE_WHEEL:
            if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_ABS) send_abs_mouse_report((int8_t)args[1]);
            else if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_REL) send_rel_mouse_report(0, 0, (int8_t)args[1]);
            send_response(PONG_OK); break;
        case CMD_MOUSE_REL:
            if ((active_mouse_mode & OUT1_MOUSE_MASK) == OUT1_MOUSE_USB_REL) send_rel_mouse_report((int8_t)args[0], (int8_t)args[1], 0);
            send_response(PONG_OK); break;
        case CMD_HUMANIZE:
            humanize_enabled = (args[0] != 0);
            save_output_modes();
            Serial.printf("[humanize] %s\n", humanize_enabled ? "ON" : "OFF");
            send_response(PONG_OK); break;
        default: send_response(RESP_INVALID_ERROR); break;
    }
}

// ---- Setup ----
void setup() {
    Serial.begin(115200);
    Serial.println("\n[pikvm-hid] boot v2 (Arduino HID)");

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(serial_str, sizeof(serial_str), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    load_output_modes();

    // Logitech USB Keyboard identity — macOS has a native profile for this
    // VID/PID, so the Keyboard Setup Assistant dialog does not fire.
    // For the conference demo this also makes the device fingerprint match
    // a real Logitech keyboard rather than advertising itself as PiKVM.
    // The mirror-device feature (see .planning/seeds) will make this
    // runtime-configurable via voice once the NVS descriptor system lands.
    USB.VID(0x046d);
    USB.PID(0xc31c);
    USB.manufacturerName("Logitech");
    USB.productName("USB Keyboard");
    USB.serialNumber(serial_str);
    Keyboard.begin();
    AbsMouseDev.begin();
    RelMouse.begin();
    USB.begin();

    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    boot_ms = millis();
    delay(500);
    Serial.printf("[pikvm-hid] mounted=%d\n", tud_mounted());

#if HAVE_WIFI
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("[pikvm-hid] wifi connecting...");
    // OTA: once WiFi comes up, ArduinoOTA.begin() is called in loop().
    // Push firmware from Pi: pio run -e esp32s3_ota -t upload
    ArduinoOTA.setHostname(OTA_HOSTNAME);
#  ifdef OTA_PASSWORD
    ArduinoOTA.setPassword(OTA_PASSWORD);
#  endif
    ArduinoOTA.onStart([]() { Serial.println("[ota] starting"); });
    ArduinoOTA.onEnd([]()   { Serial.println("[ota] done — rebooting"); });
    ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[ota] error %u\n", e); });
#endif
}

// ---- Main Loop ----
void loop() {
    if (reset_required && reset_scheduled_at > 0) {
        if (millis() - reset_scheduled_at >= 100) { Serial1.flush(); esp_restart(); }
        return;
    }

    while (Serial1.available()) {
        rx_buf[rx_index] = Serial1.read();
        if (rx_index == PACKET_SIZE - 1) { handle_packet(rx_buf); rx_index = 0; }
        else { rx_last_byte_ms = millis(); rx_index++; }
    }

    if (rx_index > 0 && (millis() - rx_last_byte_ms >= INTERBYTE_TIMEOUT_MS)) {
        stat_timeouts++;
        send_response(RESP_TIMEOUT_ERROR);
        rx_index = 0;
    }

#if HAVE_WIFI
    ArduinoOTA.handle();
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifi_was_connected) {
            wifi_was_connected = true;
            tcp_server.begin();
            ArduinoOTA.begin();
            Serial.printf("[pikvm-hid] wifi up ip=%s\n", WiFi.localIP().toString().c_str());
        }
        WiFiClient client = tcp_server.available();
        if (client) {
            unsigned long t = millis();
            while (client.connected() && millis() - t < 200) { if (client.available()) { client.read(); break; } delay(1); }
            client.printf("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\n\r\n"
                "{\"up\":%lu,\"mounted\":%d,\"pkts\":%lu,\"crc_err\":%lu,\"fw\":\"" __DATE__ "\"}\n",
                (millis() - boot_ms) / 1000, tud_mounted(), stat_packets_ok, stat_crc_errors);
            client.stop();
        }
    } else if (wifi_was_connected) { wifi_was_connected = false; }
#endif

    if (millis() - last_status_ms >= 5000) {
        last_status_ms = millis();
        Serial.printf("[status] up=%lus mounted=%d pkt=%lu crc=%lu\n",
            (millis() - boot_ms) / 1000, tud_mounted(), stat_packets_ok, stat_crc_errors);
    }
}
