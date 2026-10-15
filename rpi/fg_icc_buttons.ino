/*
 * fg_icc_buttons.ino — FG Falcon ICC fascia buttons -> Raspberry Pi Pico -> USB (serial events + optional HID)
 *
 * Reads the factory ICC front fascia EXACTLY the way the FDIM did (from the 8R29-14D017-AC firmware teardown):
 *
 *   * a 5 x 8 KEY MATRIX  — 5 row lines (FDIM port PCS0..PCS4) x 8 column lines (PCT0..PCT7), 27 keys used
 *   * 6 DIRECT KEYS       — one wire each, switch to ground (FDIM P50..P55: HAZARD, MENU, EJECT, LOCK, UNLOCK, DOME LAMP)
 *   * 1 ROTARY ENCODER    — volume / menu knob, quadrature A/B (FDIM P612 / P613)
 *   * the CABIN TEMPERATURE SENSOR (CTS) — an NTC thermistor on the fascia PCB (FDIM ADC ANI0)
 *   * button ILLUMINATION — the fascia LEDs; one PWM output here (the FDIM drove them from the dash dimming level)
 *
 * and streams events to the Pi over USB-serial, for rpi/fdim_pi.py which turns them into the exact MS-CAN frames the
 * factory FDIM sent (0x2FC audio / ICC keys, 0x307 climate + body keys, 0x313 cabin temperature).
 *
 * ---------------------------------------------------------------------------------------------------------------
 * BOARD / IDE
 *   Raspberry Pi Pico, Pico W, Pico 2 (any RP2040/RP2350 board).
 *   Arduino IDE: install "Raspberry Pi Pico/RP2040/RP2350" by Earle F. Philhower (Boards Manager URL:
 *   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json).
 *   Tools -> Board -> Raspberry Pi Pico.  Either USB stack works: "Pico SDK" (default) gives the serial port plus the
 *   HID keyboard (the core's Keyboard library is composite-capable); "Adafruit TinyUSB" works too.
 *   Hold BOOTSEL while plugging in the first time, then Upload.
 *
 * ---------------------------------------------------------------------------------------------------------------
 * WIRING (Pico GP numbers; change the tables below to suit your harness)
 *
 *   MATRIX ROWS  (FDIM PCS0..4)   GP2  GP3  GP4  GP5  GP6        driven LOW one at a time, otherwise hi-Z
 *   MATRIX COLS  (FDIM PCT0..7)   GP7  GP8  GP9  GP10 GP11 GP12 GP13 GP14   inputs with pull-ups, LOW = pressed
 *   DIRECT KEYS  (FDIM P50..P55)  GP15 HAZARD   GP16 MENU   GP17 EJECT   GP18 LOCK   GP19 UNLOCK   GP20 DOME_LAMP
 *   ENCODER      (FDIM P612/613)  GP21 = A     GP22 = B     (common -> GND)
 *   CTS NTC      (FDIM ANI0)      GP26/A0: 3V3 --[R_BIAS]--+-- A0 ;  NTC from that node to GND
 *   ILLUMINATION PWM              GP27 -> LED driver (NPN/MOSFET low-side) for the fascia backlight LEDs
 *   USB                           to the Pi (power + serial + HID)
 *
 *   The fascia switches are plain contacts: a key shorts its ROW line to its COLUMN line (matrix) or its line to
 *   ground (direct). The factory module biased them from its own supply through pull-ups; this board does the same
 *   from 3V3, so there is no 5 V anywhere and nothing to level-shift. Keep the FDIM's own row/column order when you
 *   wire the connector and the KEY NAMES below come out right automatically (the names were derived from the
 *   firmware for that order). If you wire them in a different order, use mapping mode (MAP 1) and edit the tables.
 *
 * ---------------------------------------------------------------------------------------------------------------
 * SERIAL PROTOCOL (115200 8N1 over USB-CDC, one line per message, '\n' terminated)
 *
 *   -> Pi   HELLO fg-icc-buttons 2 keys=33          at start-up
 *           KEY <NAME> DOWN | UP                    any fascia key (names: see keys[] / direct[] below)
 *           ENC +1 | -1                             encoder detent right (+1) / left (-1)
 *           CTS <code>                              cabin temperature, code/6 = degC (what 0x313 byte 0 carries)
 *           MAT <row> <col> DOWN|UP                 mapping mode only: raw matrix position
 *           DIRECT <n> DOWN|UP                      mapping mode only: raw direct line
 *           PONG / OK / ERR ...                     replies
 *   <- Pi   MAP 1|0        raw mapping output on/off
 *           DIM <0-255>    fascia illumination level (PWM)
 *           HID 1|0        USB keyboard output on/off (needs USE_HID)
 *           INFO           print the key table
 *           PING
 *
 * Key names (match rpi/fdim_pi.py ICC_KEYS and the firmware's own switch table):
 *   audio/ICC -> 0x2FC : MENU AUDIO_ONOFF SEEK_UP SEEK_DOWN FM_AM SCAN CD_AUX LOAD EJECT OK BACK_HOME MFB1..MFB6
 *   climate   -> 0x307 : CLIMATE_OFF AUTO AC RECIRC FRONT_DEMIST REAR_DEMIST MODE BLOWER_UP BLOWER_DOWN TEMP_UP TEMP_DOWN
 *   body      -> 0x307 : HAZARD LOCK UNLOCK DOME_LAMP DSC
 */

#include <Arduino.h>
#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>   // Tools -> USB Stack -> Adafruit TinyUSB needs this included from the sketch
#endif

#define USE_HID 1          // 1 = also present a USB keyboard (arduino-pico + TinyUSB stack); 0 = serial only
#define SCAN_MS 5          // matrix/direct sample period (the FDIM scanned every 5 ms)
#define DEBOUNCE_N 3       // samples that must agree (FDIM: majority of 3)
#define CTS_PERIOD_MS 1000 // cabin temperature report interval
#define ENC_SAMPLE_US 500  // encoder polling

#if USE_HID && defined(ARDUINO_ARCH_RP2040) && __has_include(<Keyboard.h>)
#include <Keyboard.h>
#define HID_OK 1
#else
#define HID_OK 0
#endif

// ------------------------------------------------------------------------------------------- pin tables
static const uint8_t ROW_PINS[5] = { 2, 3, 4, 5, 6 };                       // FDIM PCS0..PCS4
static const uint8_t COL_PINS[8] = { 7, 8, 9, 10, 11, 12, 13, 14 };         // FDIM PCT0..PCT7
static const uint8_t DIRECT_PINS[6] = { 15, 16, 17, 18, 19, 20 };           // FDIM P50..P55
static const uint8_t ENC_A = 21, ENC_B = 22;                                // FDIM P612 / P613
static const uint8_t CTS_PIN = 26;                                          // A0, FDIM ANI0
static const uint8_t ILLUM_PIN = 27;                                        // PWM out

// CTS thermistor model (edit to your part; the factory sensor is a small NTC on the fascia PCB).
static const float CTS_R_BIAS = 10000.0f;   // ohms, from 3V3 to the ADC node
static const float CTS_R25    = 10000.0f;   // NTC resistance at 25 degC
static const float CTS_BETA   = 3950.0f;    // NTC beta
static const float CTS_OFFSET = 0.0f;       // degC trim

// ------------------------------------------------------------------------------------------- key names
// Matrix: index = row*8 + col, exactly the FDIM's own scan order. "" = no key at that position.
static const char *const MATRIX_NAMES[40] = {
    /* row 0 */ "CLIMATE_OFF", "BLOWER_DOWN", "MODE",         "DSC",       "LOAD",        "",     "MFB4", "MFB3",
    /* row 1 */ "RECIRC",      "BLOWER_UP",   "FRONT_DEMIST", "SCAN",      "OK",          "",     "MFB5", "MFB2",
    /* row 2 */ "AC",          "",            "TEMP_DOWN",    "CD_AUX",    "AUDIO_ONOFF", "",     "MFB6", "MFB1",
    /* row 3 */ "AUTO",        "",            "TEMP_UP",      "FM_AM",     "SEEK_DOWN",   "",     "",     "",
    /* row 4 */ "",            "",            "REAR_DEMIST",  "BACK_HOME", "SEEK_UP",     "",     "",     "",
};
static const char *const DIRECT_NAMES[6] = { "HAZARD", "MENU", "EJECT", "LOCK", "UNLOCK", "DOME_LAMP" };

#if HID_OK
// HID keymap for fdim_pi.py's skin mode (0 = no HID key; climate/body keys are serial-only).
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

// encoder: the FDIM algorithm — an A edge with B steady = RIGHT (+1), a B edge with A steady = LEFT (-1),
// one count per detent. Keeps the factory feel (same counts per turn as the real module).
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
    if (a != enc_a_prev && b == enc_b_prev && a != b) d = +1;               // A edge, B steady
    else if (b != enc_b_prev && a == enc_a_prev && a != b) d = -1;          // B edge, A steady
    enc_a_prev = a; enc_b_prev = b;
    if (d) {
        Serial.println(d > 0 ? "ENC +1" : "ENC -1");
#if HID_OK
        if (hid_on) { Keyboard.press(d > 0 ? KEY_RIGHT_ARROW : KEY_LEFT_ARROW); Keyboard.release(d > 0 ? KEY_RIGHT_ARROW : KEY_LEFT_ARROW); }
#endif
    }
}

static int read_cts_code() {
    uint32_t acc = 0;
    for (int i = 0; i < 16; i++) acc += analogRead(CTS_PIN);
    float v = (acc / 16.0f) / 4095.0f;                   // fraction of 3V3 at the NTC node
    if (v <= 0.001f || v >= 0.999f) return -1;           // open / short
    float r = CTS_R_BIAS * v / (1.0f - v);               // NTC to GND, bias to 3V3
    float invT = 1.0f / 298.15f + logf(r / CTS_R25) / CTS_BETA;
    float tc = 1.0f / invT - 273.15f + CTS_OFFSET;
    int code = (int)lroundf(tc * 6.0f);                  // 0x313 byte 0: code / 6 = degC
    if (code < 0) code = 0; if (code > 250) code = 250;
    return code;
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
    else if (sscanf(line, "DIM %d", &v) == 1) { if (v < 0) v = 0; if (v > 255) v = 255; analogWrite(ILLUM_PIN, v); Serial.println("OK"); }
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
    pinMode(ILLUM_PIN, OUTPUT); analogWrite(ILLUM_PIN, 0);
    pinMode(LED_BUILTIN, OUTPUT);
#if defined(ARDUINO_ARCH_RP2040)
    analogReadResolution(12);
#endif
#if HID_OK
    Keyboard.begin();
#endif
    enc_a_prev = digitalRead(ENC_A) == LOW; enc_b_prev = digitalRead(ENC_B) == LOW;
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) {}                 // give the host a moment to open the port
    Serial.println("HELLO fg-icc-buttons 2 keys=33");
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
    digitalWrite(LED_BUILTIN, (dir_state || mat_state[0] || mat_state[1] || mat_state[2] || mat_state[3] || mat_state[4]) ? HIGH : LOW);
}
