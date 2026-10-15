/*
 * fg_icc_buttons_esp32.ino — FG Falcon ICC fascia buttons -> ESP32 -> USB serial (+ HID on the S3)
 *
 * ESP32 port of pico/fg_icc_buttons: same inputs, same event protocol, same key names, so rpi/fdim_pi.py
 * (--pico <port>) does not know or care which MCU is on the other end. Reads the fascia the way the FDIM did
 * (8R29-14D017-AC firmware teardown, docs/PICO-ICC-buttons.md):
 *
 *   * 5 x 8 KEY MATRIX   — rows = FDIM PCS0..4, columns = FDIM PCT0..7 (27 keys), 5 ms scan, majority-of-3 debounce
 *   * 6 DIRECT KEYS      — FDIM P50..P55: HAZARD MENU EJECT LOCK UNLOCK DOME_LAMP (switch to ground)
 *   * ROTARY ENCODER     — volume / menu knob (FDIM P612/P613), the FDIM's own algorithm, one event per detent
 *   * CABIN TEMP SENSOR  — the NTC on the fascia PCB (FDIM ANI0) -> "CTS code" (code/6 degC)
 *   * ILLUMINATION       — PWM for the fascia LEDs, set by the host ("DIM n")
 *
 * ---------------------------------------------------------------------------------------------------------------
 * BOARDS / IDE
 *   Arduino IDE + the "esp32" core by Espressif (Boards Manager URL
 *   https://espressif.github.io/arduino-esp32/package_esp32_index.json), core 2.0.x or 3.x. No libraries needed.
 *   * ESP32-S3 (DevKitC-1 etc.): native USB. Tools -> USB CDC On Boot -> Enabled gives the serial port on the
 *     USB connector; Tools -> USB Mode -> "USB-OTG (TinyUSB)" additionally gives the HID keyboard (USE_HID).
 *   * ESP32 classic (WROOM-32 DevKitC): serial through the board's USB-UART bridge (/dev/ttyUSB0 on the Pi).
 *     No native USB, so no HID — the serial path is the one fdim_pi.py uses in the car anyway.
 *   * ESP32-C3/C6: too few GPIOs for the matrix (needs 23 lines) -> use an S3 or the Pico.
 *
 * ---------------------------------------------------------------------------------------------------------------
 * WIRING  (fascia switches are dry contacts; the internal pull-ups bias them from 3V3 — no 5 V anywhere)
 *
 *                      ESP32-S3                         ESP32 classic (WROOM-32)
 *   rows   PCS0..4    4 5 6 7 8                         13 14 27 26 25
 *   cols   PCT0..7    9 10 11 12 13 14 15 16            16 17 18 19 21 22 23 5
 *   direct P50..55    17 18 21 39 40 41                 4 15 32 33 34* 35*     (* input-only pins: add 10 kΩ pull-ups to 3V3)
 *   encoder A / B     42 / 2                            2 / 39*                (* input-only: add a 10 kΩ pull-up)
 *   CTS NTC           1  (ADC1)                         36 (VP, ADC1)
 *   illumination PWM  47                                12 (keep a 10 kΩ pull-down on the LED driver input — boot strapping pin)
 *   CTS network: 3V3 --[10 kΩ]--+-- ADC pin ; NTC from that node to GND.
 *
 *   Rows are driven LOW one at a time and left hi-Z otherwise (how the FDIM's row driver worked); columns and direct
 *   keys are INPUT_PULLUP, LOW = pressed. Keep the FDIM's row/column order on the fascia connector and the key names
 *   below come out right by themselves; otherwise use mapping mode (MAP 1) and edit the tables.
 *
 * ---------------------------------------------------------------------------------------------------------------
 * SERIAL PROTOCOL (115200 8N1, one line per message) — identical to the Pico
 *   -> host  HELLO fg-icc-buttons 2 keys=33 esp32 | KEY <NAME> DOWN|UP | ENC +1|-1 | CTS <code> |
 *            MAT r c DOWN|UP, DIRECT n DOWN|UP (mapping mode) | PONG / OK / INFO ...
 *   <- host  MAP 1|0 | DIM 0-255 | HID 1|0 | INFO | PING
 */

#include <Arduino.h>

#define USE_HID        1      // ESP32-S3 in USB-OTG mode only; compiles out everywhere else
#define SCAN_MS        5      // matrix / direct sample period (the FDIM scanned every 5 ms)
#define DEBOUNCE_N     3      // samples that must agree (FDIM: majority of 3)
#define CTS_PERIOD_MS  1000   // cabin temperature report interval
#define ENC_SAMPLE_US  500    // encoder polling

// ------------------------------------------------------------------------------------------- pin tables
#if defined(CONFIG_IDF_TARGET_ESP32S3)
static const uint8_t ROW_PINS[5]    = { 4, 5, 6, 7, 8 };                     // FDIM PCS0..PCS4
static const uint8_t COL_PINS[8]    = { 9, 10, 11, 12, 13, 14, 15, 16 };     // FDIM PCT0..PCT7
static const uint8_t DIRECT_PINS[6] = { 17, 18, 21, 39, 40, 41 };            // FDIM P50..P55
static const uint8_t ENC_A = 42, ENC_B = 2;                                  // FDIM P612 / P613
static const uint8_t CTS_PIN = 1;                                            // FDIM ANI0
static const uint8_t ILLUM_PIN = 47;
#elif defined(CONFIG_IDF_TARGET_ESP32)
static const uint8_t ROW_PINS[5]    = { 13, 14, 27, 26, 25 };
static const uint8_t COL_PINS[8]    = { 16, 17, 18, 19, 21, 22, 23, 5 };
static const uint8_t DIRECT_PINS[6] = { 4, 15, 32, 33, 34, 35 };             // 34/35: external pull-ups
static const uint8_t ENC_A = 2, ENC_B = 39;                                  // 39: external pull-up
static const uint8_t CTS_PIN = 36;
static const uint8_t ILLUM_PIN = 12;
#else
#error "Needs an ESP32-S3 or a classic ESP32: the C3/C6 do not have enough GPIOs for the 5x8 matrix."
#endif

// CTS thermistor model (edit to your part; the factory sensor is a small NTC on the fascia PCB)
static const float CTS_R_BIAS = 10000.0f;   // ohms, from 3V3 to the ADC node
static const float CTS_R25    = 10000.0f;   // NTC resistance at 25 degC
static const float CTS_BETA   = 3950.0f;    // NTC beta
static const float CTS_OFFSET = 0.0f;       // degC trim
static const float CTS_VREF_MV = 3300.0f;   // the rail feeding the bias resistor

// ------------------------------------------------------------------------------------------- key names
// Matrix: index = row*8 + col, exactly the FDIM's own scan order. "" = no key at that position.
static const char *const MATRIX_NAMES[40] = {
    /* row 0 */ "CLIMATE_OFF", "BLOWER_DOWN", "MODE",         "DSC",       "LOAD",        "", "MFB4", "MFB3",
    /* row 1 */ "RECIRC",      "BLOWER_UP",   "FRONT_DEMIST", "SCAN",      "OK",          "", "MFB5", "MFB2",
    /* row 2 */ "AC",          "",            "TEMP_DOWN",    "CD_AUX",    "AUDIO_ONOFF", "", "MFB6", "MFB1",
    /* row 3 */ "AUTO",        "",            "TEMP_UP",      "FM_AM",     "SEEK_DOWN",   "", "",     "",
    /* row 4 */ "",            "",            "REAR_DEMIST",  "BACK_HOME", "SEEK_UP",     "", "",     "",
};
static const char *const DIRECT_NAMES[6] = { "HAZARD", "MENU", "EJECT", "LOCK", "UNLOCK", "DOME_LAMP" };

// ------------------------------------------------------------------------------------------- HID (S3, USB-OTG)
#if USE_HID && defined(CONFIG_IDF_TARGET_ESP32S3) && defined(ARDUINO_USB_MODE) && (ARDUINO_USB_MODE == 0)
#include "USB.h"
#include "USBHIDKeyboard.h"
#define HID_OK 1
static USBHIDKeyboard Keyboard;
// keymap for fdim_pi.py's skin mode (0 = no HID key; climate/body keys are serial-only)
struct HidMap { const char *name; uint8_t key; };
static const HidMap HID_MAP[] = {
    { "MFB1", '1' }, { "MFB2", '2' }, { "MFB3", '3' }, { "MFB4", '4' }, { "MFB5", '5' }, { "MFB6", '6' },
    { "SEEK_UP", KEY_UP_ARROW }, { "SEEK_DOWN", KEY_DOWN_ARROW }, { "OK", KEY_RETURN }, { "BACK_HOME", KEY_ESC },
    { "MENU", 'm' }, { "AUDIO_ONOFF", 'p' }, { "CD_AUX", 's' }, { "FM_AM", 'r' }, { "SCAN", 'a' },
    { "LOAD", 'l' }, { "EJECT", 'e' },
};
static uint8_t hid_key_for(const char *name) {
    for (size_t i = 0; i < sizeof HID_MAP / sizeof HID_MAP[0]; i++)
        if (!strcmp(HID_MAP[i].name, name)) return HID_MAP[i].key;
    return 0;
}
#else
#define HID_OK 0
#endif

// ------------------------------------------------------------------------------------------- state
static uint8_t mat_raw[DEBOUNCE_N][5];    // sample history, bit = column pressed
static uint8_t mat_state[5];              // debounced
static uint8_t dir_raw[DEBOUNCE_N];       // bit n = direct key n pressed
static uint8_t dir_state;
static uint8_t sample_idx;
static bool mapping = false, hid_on = HID_OK;
static uint32_t t_scan, t_cts, t_enc;
static int last_cts = -1;
static uint8_t enc_a_prev, enc_b_prev;

static void emit_key(const char *name, bool down) {
    Serial.print("KEY "); Serial.print(name); Serial.println(down ? " DOWN" : " UP");
#if HID_OK
    if (hid_on) {
        uint8_t k = hid_key_for(name);
        if (k) { if (down) Keyboard.press(k); else Keyboard.release(k); }
    }
#endif
}

static uint8_t majority3(uint8_t a, uint8_t b, uint8_t c) { return (a & b) | (a & c) | (b & c); }

static void scan_matrix() {
    uint8_t rows[5];
    for (uint8_t r = 0; r < 5; r++) {
        pinMode(ROW_PINS[r], OUTPUT); digitalWrite(ROW_PINS[r], LOW);        // select row (like the FDIM's row driver)
        delayMicroseconds(60);
        uint8_t cols = 0;
        for (uint8_t c = 0; c < 8; c++) if (digitalRead(COL_PINS[c]) == LOW) cols |= 1 << c;
        pinMode(ROW_PINS[r], INPUT);                                          // release row (hi-Z)
        rows[r] = cols;
    }
    uint8_t d = 0;
    for (uint8_t n = 0; n < 6; n++) if (digitalRead(DIRECT_PINS[n]) == LOW) d |= 1 << n;

    for (uint8_t r = 0; r < 5; r++) mat_raw[sample_idx][r] = rows[r];
    dir_raw[sample_idx] = d;
    sample_idx = (sample_idx + 1) % DEBOUNCE_N;

    for (uint8_t r = 0; r < 5; r++) {
        uint8_t ns = majority3(mat_raw[0][r], mat_raw[1][r], mat_raw[2][r]);
        uint8_t ch = ns ^ mat_state[r];
        if (!ch) continue;
        mat_state[r] = ns;
        for (uint8_t c = 0; c < 8; c++) if (ch & (1 << c)) {
            bool down = ns & (1 << c);
            const char *name = MATRIX_NAMES[r * 8 + c];
            if (mapping) { Serial.print("MAT "); Serial.print(r); Serial.print(' '); Serial.print(c); Serial.println(down ? " DOWN" : " UP"); }
            if (name[0]) emit_key(name, down);
            else { char tmp[8]; snprintf(tmp, sizeof tmp, "R%dC%d", r, c); emit_key(tmp, down); }
        }
    }
    uint8_t nd = majority3(dir_raw[0], dir_raw[1], dir_raw[2]);
    uint8_t chd = nd ^ dir_state;
    if (chd) {
        dir_state = nd;
        for (uint8_t n = 0; n < 6; n++) if (chd & (1 << n)) {
            bool down = nd & (1 << n);
            if (mapping) { Serial.print("DIRECT "); Serial.print(n); Serial.println(down ? " DOWN" : " UP"); }
            emit_key(DIRECT_NAMES[n], down);
        }
    }
}

static void poll_encoder() {
    uint8_t a = digitalRead(ENC_A) == LOW, b = digitalRead(ENC_B) == LOW;   // active-low contacts
    int d = 0;
    if (a != enc_a_prev && b == enc_b_prev && a != b) d = +1;               // FDIM: A edge with B steady = right
    else if (b != enc_b_prev && a == enc_a_prev && a != b) d = -1;          //       B edge with A steady = left
    enc_a_prev = a; enc_b_prev = b;
    if (d) {
        Serial.println(d > 0 ? "ENC +1" : "ENC -1");
#if HID_OK
        if (hid_on) { uint8_t k = d > 0 ? KEY_RIGHT_ARROW : KEY_LEFT_ARROW; Keyboard.press(k); Keyboard.release(k); }
#endif
    }
}

static int read_cts_code() {
    uint32_t acc = 0;
    for (int i = 0; i < 16; i++) acc += analogReadMilliVolts(CTS_PIN);     // eFuse-calibrated millivolts
    float v = (acc / 16.0f) / CTS_VREF_MV;                                  // fraction of the rail at the NTC node
    if (v <= 0.01f || v >= 0.99f) return -1;                                // open / short
    float r = CTS_R_BIAS * v / (1.0f - v);                                  // NTC to GND, bias to 3V3
    float invT = 1.0f / 298.15f + logf(r / CTS_R25) / CTS_BETA;
    float tc = 1.0f / invT - 273.15f + CTS_OFFSET;
    int code = (int)lroundf(tc * 6.0f);                                     // 0x313 byte 0: code / 6 = degC
    if (code < 0) code = 0; if (code > 250) code = 250;
    return code;
}

// illumination PWM through LEDC (API differs between core 2.x and 3.x)
static void illum_begin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(ILLUM_PIN, 1000, 8);
#else
    ledcSetup(0, 1000, 8); ledcAttachPin(ILLUM_PIN, 0);
#endif
}
static void illum_set(int v) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(ILLUM_PIN, v);
#else
    ledcWrite(0, v);
#endif
}

static void print_info() {
    Serial.println("INFO matrix rows=PCS0..4 cols=PCT0..7 (row*8+col):");
    for (uint8_t r = 0; r < 5; r++) {
        Serial.print("INFO row "); Serial.print(r); Serial.print(':');
        for (uint8_t c = 0; c < 8; c++) { Serial.print(' '); Serial.print(MATRIX_NAMES[r * 8 + c][0] ? MATRIX_NAMES[r * 8 + c] : "-"); }
        Serial.println();
    }
    Serial.print("INFO direct:"); for (uint8_t n = 0; n < 6; n++) { Serial.print(' '); Serial.print(DIRECT_NAMES[n]); } Serial.println();
    Serial.print("INFO hid="); Serial.print(hid_on ? 1 : 0); Serial.print(" mapping="); Serial.println(mapping ? 1 : 0);
}

static void handle_command(char *line) {
    int v;
    if (!strncmp(line, "PING", 4)) Serial.println("PONG");
    else if (sscanf(line, "MAP %d", &v) == 1) { mapping = v != 0; Serial.println("OK"); }
    else if (sscanf(line, "DIM %d", &v) == 1) { if (v < 0) v = 0; if (v > 255) v = 255; illum_set(v); Serial.println("OK"); }
    else if (sscanf(line, "HID %d", &v) == 1) { hid_on = HID_OK && v != 0; Serial.println(hid_on ? "OK" : "OK (no HID in this build)"); }
    else if (!strncmp(line, "INFO", 4)) print_info();
    else Serial.println("ERR unknown command");
}

void setup() {
    Serial.begin(115200);
    for (uint8_t r = 0; r < 5; r++) pinMode(ROW_PINS[r], INPUT);
    for (uint8_t c = 0; c < 8; c++) pinMode(COL_PINS[c], INPUT_PULLUP);
    for (uint8_t n = 0; n < 6; n++) pinMode(DIRECT_PINS[n], INPUT_PULLUP);
    pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
    analogReadResolution(12);
    illum_begin(); illum_set(0);
#if HID_OK
    Keyboard.begin(); USB.begin();
#endif
    enc_a_prev = digitalRead(ENC_A) == LOW; enc_b_prev = digitalRead(ENC_B) == LOW;
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) {}                 // give the host a moment to open the port
    Serial.println("HELLO fg-icc-buttons 2 keys=33 esp32");
    t_scan = millis(); t_cts = millis(); t_enc = micros();
}

void loop() {
    static char cmd[64]; static uint8_t cmdlen = 0;
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') { if (cmdlen) { cmd[cmdlen] = 0; handle_command(cmd); } cmdlen = 0; }
        else if (cmdlen < sizeof cmd - 1) cmd[cmdlen++] = ch;
    }
    uint32_t now = millis();
    if (now - t_scan >= SCAN_MS) { t_scan += SCAN_MS; scan_matrix(); }
    if (micros() - t_enc >= ENC_SAMPLE_US) { t_enc = micros(); poll_encoder(); }
    if (now - t_cts >= CTS_PERIOD_MS) {
        t_cts = now;
        int code = read_cts_code();
        if (code >= 0 && code != last_cts) { last_cts = code; Serial.print("CTS "); Serial.println(code); }
    }
}
