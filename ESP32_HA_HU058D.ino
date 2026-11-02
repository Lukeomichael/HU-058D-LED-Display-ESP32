// AiP33628 panel reference firmware for the HU-058D clock board.
// Drives both display buses from an ESP32-WROOM-32, with no dependency on
// anything beyond the Arduino core. Everything specific to this panel sits in
// the tables near the top, so a port to another MCU comes down to replacing
// the four GPIO writes inside sendFrame.
//
// Mapping and scan parameters come from docs/display-map.md.

#include <Arduino.h>
#include <string.h>

// --- WiFi & networking ---
#include <WiFiManager.h>
#include <Preferences.h>
#include <time.h>
#include <WiFi.h>
#include <PubSubClient.h>

// --- Configuration Defaults ---
const char* AP_SSID = "AutoConnectAP";
const char* NTP_SERVER = "pool.ntp.org";

// --- Hardware Pins ---
const int TRIGGER_PIN = 0;      // GPIO 0, built-in BOOT button
const int LED_PIN = 2;          // GPIO 2, onboard LED
const int BUZZER_PIN = 25;      // GPIO 25, external buzzer
const int TOP_BUTTON = 33;      // GPIO 33, external top button
const int BOTTOM_BUTTON = 32;   // GPIO 32, external bottom button

// --- Button interrupt state ---
volatile bool TOP_PRESSED = false;
volatile unsigned long lastTopPressTime = 0;
#define BUTTON_DEBOUNCE_MS 1000  // 1 s lockout between accepted presses

// --- Stopwatch globals ---
char minute_digit1 = '0';
char minute_digit2 = '0';
char second_digit1 = '0';
char second_digit2 = '0';

static uint32_t stopWatchSeconds = 0;
static bool stopWatchRunning = false;
static unsigned long lastTick = 0;
static bool bottomWasPressed = false;
static unsigned long bottomPressStart = 0;

// --- Stopwatch limit & alarm ---
int stopWatchLimitMinutes = 60;
String topic_stopwatch_limit_cmd;
String topic_stopwatch_limit_state;

static bool alarmActive = false;
static unsigned long alarmStartTime = 0;
#define ALARM_DURATION_MS 10000
static bool stopWatchAtLimit = false;

// --- Buzzer tone ---
unsigned long toneStartTime = 0;
bool isCurrentlyPlaying = false;
#define TONE_FREQ 1000
#define TONE_DURATION 500

// --- Timezone Arrays ---
const char* australian_tz_strings[] = {
  "AEST-10AEDT,M10.1.0,M4.1.0/3",           // 0: NSW, Victoria, ACT, Tasmania
  "AEST-10",                                // 1: Queensland
  "ACST-9:30ACDT,M10.1.0,M4.1.0",           // 2: South Australia
  "ACST-9:30",                              // 3: Northern Territory
  "AWST-8",                                 // 4: Western Australia
  "LHST-10:30LHDT-11,M10.1.0,M4.1.0"       // 5: Lord Howe Island
};

hw_timer_t *scanTimer = NULL;

// --- Globals ---
WiFiManager wm;
Preferences preferences;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

int saved_tz_index = 0;
String saved_mqtt_host = "";
int saved_mqtt_port = 1883;
String saved_mqtt_user = "";
String saved_mqtt_pass = "";
String saved_mqtt_topic = "LEDClock01";

String mqtt_client_id;
String topic_led_cmd;
String topic_led_state;
String topic_buzzer_cmd;
String topic_buzzer_state;
String topic_rgb_cmd;
String topic_rgb_state;
String topic_activity;
String topic_timer_toggle_cmd;
String topic_timer_reset_cmd;
String topic_timer_state;
String topic_timer_elapsed;
String topic_show_timer_cmd;
String topic_show_timer_state;
String topic_show_value_cmd;
String topic_show_value_state;
String topic_display_value_cmd;
String topic_display_value_state;

bool buzzer_state = false;
bool display_state = true;
bool showValue = false;
String displayValue = "";

// --- Display colors ---
int digitRed = 0, digitGreen = 0, digitBlue = 2;
int colonRed = 0, colonGreen = 0, colonBlue = 2;
int amRed = 2, amGreen = 0, amBlue = 0;
int degreeRed = 0, degreeGreen = 0, degreeBlue = 0;

// --- Display bus pins ---
// ESP32 GPIO -> MCU socket pin on the clock board. Ground through socket pin 8.
static const int PIN_CLK = 22;      // socket pin 14 (HU-058D) or 16 (HU-058/SE), driver 1 clock
static const int PIN_DATA = 21;     // socket pin 5,  driver 1 data
static const int PIN_CLK_1 = 19;    // socket pin 1,  driver 2 clock
static const int PIN_DATA_1 = 18;   // socket pin 2,  driver 2 data

static const uint32_t M_CLK = 1UL << PIN_CLK;
static const uint32_t M_DATA = 1UL << PIN_DATA;
static const uint32_t M_CLK_1 = 1UL << PIN_CLK_1;
static const uint32_t M_DATA_1 = 1UL << PIN_DATA_1;

// Stock is 2407us per cycle, four COM pairs, 415.5Hz refresh.
static const uint8_t COM_SEQ[4] = {0x30, 0x0C, 0x03, 0xC0};

// Global current level, 0x0 = 2.5mA through 0xF = 40.4mA.
static volatile uint8_t g_current = 0xD;

// FRAME BUFFER:
// Per driver, per COM pair, up to two sub-frames. A COM pair only gets a
// second sub-frame when something in it sits at half duty, which is how the
// stock firmware mixes colors.
static volatile uint16_t g_ss[2][4][2];
static volatile uint8_t g_nsub[2][4];

// Channel levels, indexed [driver][com pair][led 1..5][channel].
// 0 = off, 1 = half duty, 2 = full.
static uint8_t g_level[2][4][6][3];

enum { CH_BLUE = 0, CH_GREEN = 1, CH_RED = 2 };

// Roughly 100ns at 240MHz.
static inline void IRAM_ATTR delayTick() {
  for (int i = 0; i < 12; i++) { 
    __asm__ __volatile__("nop");
  }
}

// Emit one 30-bit frame and latch it.
// Bits are LSB first: SS[15:0], CS[7:0], IS[3:0], then two reserved zeros.
// Data only ever changes while CLK is low. The latch is a DATA rising edge
// while CLK is held high after the last bit, which is what the 8051 does.
static void IRAM_ATTR sendFrame(bool bus2, uint16_t ss, uint8_t cs, uint8_t is) {
  const uint32_t mclk = bus2 ? M_CLK_1 : M_CLK;
  const uint32_t mdat = bus2 ? M_DATA_1 : M_DATA;

  uint32_t frame = (uint32_t)ss | ((uint32_t)cs << 16) | ((uint32_t)(is & 0xF) << 24);

  GPIO.out_w1tc = mclk | mdat;
  delayTick();

  for (int i = 0; i < 30; i++) {
    if ((frame >> i) & 1) {
      GPIO.out_w1ts = mdat;
    } else {
      GPIO.out_w1tc = mdat;
    }
    delayTick();
    GPIO.out_w1ts = mclk;
    delayTick();
    if (i < 29) {
      GPIO.out_w1tc = mclk;
      delayTick();
    }
  }

  // CLK is still high after bit 29. Pulse DATA to latch.
  GPIO.out_w1tc = mdat;
  delayTick();
  GPIO.out_w1ts = mdat;
  delayTick();
  GPIO.out_w1tc = mdat;
  delayTick();
  GPIO.out_w1tc = mclk;
  delayTick();
}

// PANEL MAP:
// Each digit is one driver plus one pair of COM pairs, ten LED positions.
// Driver 1 carries the hours, driver 2 the minutes.
struct Block {
  uint8_t drv;
  uint8_t comLo;
  uint8_t comHi;
};

static const Block BLOCKS[4] = {
  {0, 0x30, 0xC0},  // digit 1, hour tens,   annunciator AM
  {0, 0x03, 0x0C},  // digit 2, hour ones,   annunciator colon
  {1, 0x30, 0xC0},  // digit 3, minute tens, annunciator date dash
  {1, 0x03, 0x0C},  // digit 4, minute ones, annunciator degree mark
};

// Segment to position within a block. Side 0 is COM low, side 1 is COM high.
struct SegPos {
  uint8_t side;
  uint8_t led;
};

// Order: A B C D E F G, then the annunciator and the block 2 second colon dot.
static const SegPos SEGMAP[9] = {
  {1, 5},  // A top
  {1, 4},  // B top right
  {1, 3},  // C bottom right
  {1, 2},  // D bottom
  {0, 5},  // E bottom left
  {0, 4},  // F top left
  {0, 3},  // G middle
  {1, 1},  // annunciator
  {0, 1},  // second annunciator, block 2 only
};
enum { SEG_ANNUN = 7, SEG_ANNUN2 = 8 };

// Seven segment font, bit 0 = A through bit 6 = G.
static uint8_t glyph(char c) {
  switch (c) {
    case '0': return 0b0111111;
    case '1': return 0b0000110;
    case '2': return 0b1011011;
    case '3': return 0b1001111;
    case '4': return 0b1100110;
    case '5': return 0b1101101;
    case '6': return 0b1111101;
    case '7': return 0b0000111;
    case '8': return 0b1111111;
    case '9': return 0b1101111;
    case '-': return 0b1000000;
    case 'F': return 0b1110001;
    case 'C': return 0b0111001;
    default:  return 0;
  }
}

static int comIndex(uint8_t cs) {
  for (int i = 0; i < 4; i++) {
    if (COM_SEQ[i] == cs) return i;
  }
  return 0;
}

// SEG0 is not part of any LED and nothing on the panel responds to it.
static bool g_seg0 = false;

static void clearPanel() {
  memset(g_level, 0, sizeof(g_level));
  g_seg0 = false;
}

// Light one position at the given per-channel levels.
static void setPos(uint8_t blockIdx, uint8_t segIdx, uint8_t r, uint8_t g, uint8_t b) {
  const Block &blk = BLOCKS[blockIdx];
  const SegPos &sp = SEGMAP[segIdx];
  uint8_t cs = sp.side ? blk.comHi : blk.comLo;
  int ci = comIndex(cs);
  g_level[blk.drv][ci][sp.led][CH_RED] = r;
  g_level[blk.drv][ci][sp.led][CH_GREEN] = g;
  g_level[blk.drv][ci][sp.led][CH_BLUE] = b;
}

// Write one character into a digit at the given color.
static void setDigit(uint8_t blockIdx, char c, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t bits = glyph(c);
  for (int s = 0; s < 7; s++) {
    if (bits & (1 << s)) {
      setPos(blockIdx, s, r, g, b);
    } else {
      setPos(blockIdx, s, 0, 0, 0);
    }
  }
}

const uint16_t levels[16] = {0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE, 0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF};

uint16_t mapToLevel(uint8_t val) {
  return levels[val >> 4];
}

uint8_t levelToValue(uint16_t level) {
  int best = 0;
  uint16_t bestDiff = 0xFFFF;
  for (int i = 0; i < 16; i++) {
    uint16_t diff = abs((int)level - (int)levels[i]);
    if (diff < bestDiff) {
      bestDiff = diff;
      best = i;
    }
  }
  return best * 16 + 8;
}   

static void setAM(uint8_t r, uint8_t g, uint8_t b) { setPos(0, SEG_ANNUN, r, g, b); }
static void setDash(uint8_t r, uint8_t g, uint8_t b) { setPos(2, SEG_ANNUN, r, g, b); }
static void setDegree(uint8_t r, uint8_t g, uint8_t b) { setPos(3, SEG_ANNUN, r, g, b); }

static void setColon(uint8_t r, uint8_t g, uint8_t b) {
  setPos(1, SEG_ANNUN, r, g, b);
  setPos(1, SEG_ANNUN2, r, g, b);
}

// Light only the top colon dot (decimal point).
static void setDecimalPoint(uint8_t r, uint8_t g, uint8_t b) {
  setPos(1, SEG_ANNUN, r, g, b);
}

// Collapse the level array into sub-frames.
// static void render() {
//   for (int d = 0; d < 2; d++) {
//     for (int ci = 0; ci < 4; ci++) {
//       uint16_t full = 0, half = 0;
//       bool needsHalf = false;
//       for (int led = 1; led <= 5; led++) {
//         int base = 3 * led - 2;
//         for (int ch = 0; ch < 3; ch++) {
//           uint8_t lv = g_level[d][ci][led][ch];
//           if (lv == 0) continue;
//           uint16_t bit = 1u << (base + ch);
//           half |= bit;
//           if (lv >= 2) full |= bit;
//           if (lv == 1) needsHalf = true;
//         }
//       }
//       if (g_seg0) {
//         half |= 1;
//         full |= 1;
//       }
//       noInterrupts();
//       if (needsHalf) {
//         g_ss[d][ci][0] = half;
//         g_ss[d][ci][1] = full;
//         g_nsub[d][ci] = 2;
//       } else {
//         g_ss[d][ci][0] = half;
//         g_nsub[d][ci] = 1;
//       }
//       interrupts();
//     }
//   }
// }

static void render() {
  uint16_t ss[2][4][2];
  uint8_t  nsub[2][4];

  for (int d = 0; d < 2; d++) {
    for (int ci = 0; ci < 4; ci++) {
      uint16_t full = 0, half = 0;
      bool needsHalf = false;
      for (int led = 1; led <= 5; led++) {
        int base = 3 * led - 2;
        for (int ch = 0; ch < 3; ch++) {
          uint8_t lv = g_level[d][ci][led][ch];
          if (lv == 0) continue;
          uint16_t bit = 1u << (base + ch);
          half |= bit;
          if (lv >= 2) full |= bit;
          if (lv == 1) needsHalf = true;
        }
      }
      if (g_seg0) { half |= 1; full |= 1; }

      if (needsHalf) {
        ss[d][ci][0] = half;
        ss[d][ci][1] = full;
        nsub[d][ci]  = 2;
      } else {
        ss[d][ci][0] = half;
        nsub[d][ci]  = 1;
      }
    }
  }

  // Single critical section — ~50 ns total
  noInterrupts();
  for (int d = 0; d < 2; d++)
    for (int ci = 0; ci < 4; ci++) {
      g_ss[d][ci][0]     = ss[d][ci][0];
      g_ss[d][ci][1]     = ss[d][ci][1];
      g_nsub[d][ci]      = nsub[d][ci];
    }
  interrupts();
}   

// SCAN:
// A periodic esp_timer, not a busy loop in a task.
// Eight ticks per cycle, two per COM pair.
static const uint32_t TICK_US = 301;  // 8 x 301 = 2408us, 415.3Hz
static volatile uint8_t g_tick = 0;

void IRAM_ATTR scanTick() {
  uint8_t t = g_tick;
  g_tick = (uint8_t)((t + 1) & 7);

  int ci = t >> 1;
  int sub = t & 1;
  uint8_t cs = COM_SEQ[ci];

  if (sub < g_nsub[0][ci]) sendFrame(false, g_ss[0][ci][sub], cs, g_current);
  if (sub < g_nsub[1][ci]) sendFrame(true, g_ss[1][ci][sub], cs, g_current);
}

void IRAM_ATTR topButtonISR() {
  unsigned long now = millis();
  if (now - lastTopPressTime >= BUTTON_DEBOUNCE_MS) {
    lastTopPressTime = now;
    TOP_PRESSED = !TOP_PRESSED;
  }
}

// --- Display helpers ---
void setClockColors(int dR, int dG, int dB, int cR, int cG, int cB,
                    int aR, int aG, int aB, int degR, int degG, int degB) {
  digitRed = dR;   digitGreen = dG;   digitBlue = dB;
  colonRed = cR;   colonGreen = cG;   colonBlue = cB;
  amRed = aR;      amGreen = aG;      amBlue = aB;
  degreeRed = degR; degreeGreen = degG; degreeBlue = degB;
}

void setCurrentValue(uint8_t newValue) {
  if (newValue >= 0 && newValue <= 0xF) {
    g_current = newValue;
  }
}

void checkButtonForReset() {
  static unsigned long button_press_start_time = 0;
  static bool was_pressed = false;

  if (digitalRead(TRIGGER_PIN) == LOW || digitalRead(TOP_BUTTON) == LOW) {
    if (!was_pressed) {
      button_press_start_time = millis();
      was_pressed = true;
      Serial.println("BOOT button pressed. Keep holding to reset Wi-Fi configuration...");
    }

    if (millis() - button_press_start_time >= 10000) {
      Serial.println("\n[RESET] Button held for 10 seconds! Clearing network cache and restarting...");
      wm.resetSettings();
      delay(1000);
      ESP.restart();
    }
  } else {
    if (was_pressed) {
      Serial.println("BOOT button released early. Reset cancelled.");
      was_pressed = false;
      Serial.println(saved_mqtt_host);
      Serial.println(saved_mqtt_user);
      Serial.println(saved_mqtt_pass);
      Serial.println("*****************************************");
    }
  }
}

void checkTopButtonForAP() {
  static unsigned long topButtonPressStart = 0;
  static bool topWasHeld = false;

  if (digitalRead(TOP_BUTTON) == LOW) {
    if (!topWasHeld) {
      topButtonPressStart = millis();
      topWasHeld = true;
      Serial.println("TOP button held. Keep holding 10 s to enter AP mode...");
    }
    if (millis() - topButtonPressStart >= 10000) {
      Serial.println("\n[AP] TOP button held 10 s! Entering AP mode...");
      topButtonPressStart = 0;
      topWasHeld = false;

      mqttClient.disconnect();
      WiFi.disconnect();
      delay(500);

      // Blocks until the portal is saved or times out.
      // If new credentials are saved, it attempts to connect.
      wm.startConfigPortal(AP_SSID);

      Serial.println("AP portal finished. Resuming normal operation.");
    }
  } else {
    if (topWasHeld) {
      if (millis() - topButtonPressStart < 10000) {
        Serial.println("TOP button released early. AP mode cancelled.");
      }
      topWasHeld = false;
      topButtonPressStart = 0;
    }
  }
}   

void publishTimerState() {
  if (!mqttClient.connected()) return;

  String state = stopWatchRunning ? "running" : "stopped";
  mqttClient.publish(topic_timer_state.c_str(), state.c_str(), true);

  char buf[8];
  snprintf(buf, sizeof(buf), "%02u:%02u",
           (unsigned)(stopWatchSeconds / 60),
           (unsigned)(stopWatchSeconds % 60));
  mqttClient.publish(topic_timer_elapsed.c_str(), buf, true);
}

void publishShowTimerState() {
  if (!mqttClient.connected()) return;
  mqttClient.publish(topic_show_timer_state.c_str(), TOP_PRESSED ? "ON" : "OFF", true);
}

void publishShowValueState() {
  if (!mqttClient.connected()) return;
  mqttClient.publish(topic_show_value_state.c_str(), showValue ? "ON" : "OFF", true);
  mqttClient.publish(topic_display_value_state.c_str(), displayValue.c_str(), true);
}

void updateStopWatch() {
  static bool wasRunning = false;
  static bool wasTopPressed = false;
  static bool wasShowValue = false;
  static unsigned long lastTimerPublish = 0;

  unsigned long now = millis();
  bool bottomNow = (digitalRead(BOTTOM_BUTTON) == LOW);

  if (TOP_PRESSED) {
    // Falling edge → toggle start/stop
    if (bottomNow && !bottomWasPressed) {
      if (!stopWatchRunning) {
        if (stopWatchAtLimit) {
          stopWatchAtLimit = false;
          stopWatchSeconds = 0;
        }
      }
      stopWatchRunning = !stopWatchRunning;
      lastTick = now;
      bottomPressStart = now;
    }

    // Still held ≥ 2 s → stop + reset
    if (bottomNow && bottomWasPressed) {
      if (now - bottomPressStart >= 2000) {
        stopWatchRunning = false;
        stopWatchSeconds = 0;
        stopWatchAtLimit = false;
        lastTick = now;
        bottomPressStart = 0;
      }
    }
  }

  bottomWasPressed = bottomNow;

  // Count (catches up after gaps)
  if (stopWatchRunning) {
    unsigned long elapsed = now - lastTick;
    if (elapsed >= 1000) {
      stopWatchSeconds += elapsed / 1000;
      lastTick += (elapsed / 1000) * 1000;
    }
  } else {
    lastTick = now;
  }

  // Derive MM:SS digit characters
  unsigned int mins = stopWatchSeconds / 60;
  unsigned int secs = stopWatchSeconds % 60;

  if (mins < 10) {
    minute_digit1 = ' ';
  } else {
    minute_digit1 = (mins / 10) + '0';
  }

  minute_digit2 = (mins % 10) + '0';
  second_digit1 = (secs / 10) + '0';
  second_digit2 = (secs % 10) + '0';

  // Check against MQTT-settable limit
  if (stopWatchSeconds >= (uint32_t)stopWatchLimitMinutes * 60) {
    stopWatchSeconds = (uint32_t)stopWatchLimitMinutes * 60;
    stopWatchRunning = false;
    lastTick = now;
    bottomPressStart = 0;

    if (!stopWatchAtLimit) {
      stopWatchAtLimit = true;
      alarmActive = true;
      alarmStartTime = now;
      Serial.println("Stopwatch limit reached! 10 s alarm started.");

      if (mqttClient.connected()) {
        String payload = "Stopwatch alarm: limit of " + String(stopWatchLimitMinutes) + " min reached";
        mqttClient.publish(topic_activity.c_str(), payload.c_str(), true);
      }
    }
  }

  // Auto-return to clock display after alarm finishes
  if (stopWatchAtLimit && !alarmActive) {
    TOP_PRESSED = false;
    stopWatchAtLimit = false;
    stopWatchSeconds = 0;
    Serial.println("Alarm finished – returning to clock display.");
  }

  // Publish timer state to HA
  if (stopWatchRunning != wasRunning) {
    wasRunning = stopWatchRunning;
    lastTimerPublish = now;
    publishTimerState();
  } else if (stopWatchRunning && now - lastTimerPublish >= 1000) {
    lastTimerPublish = now;
    publishTimerState();
  }

  // Publish show timer state on physical button change
  if (TOP_PRESSED != wasTopPressed) {
    wasTopPressed = TOP_PRESSED;
    publishShowTimerState();
  }

  // Publish show value state on change
  if (showValue != wasShowValue) {
    wasShowValue = showValue;
    publishShowValueState();
  }
}

// Render the display value string to the panel.
// Format: [d][d].[d][F|C]  (up to 99.9 with optional F or C suffix)
// Position 0 = tens, Position 1 = ones + decimal point (colon dot),
// Position 2 = tenths, Position 3 = F/C/blank.
void renderDisplayValue() {
  String s = displayValue;
  s.trim();
  if (s.length() > 6) s = s.substring(0, 6);

  char d0 = ' ', d1 = ' ', d2 = ' ', d3 = ' ';
  bool dot = false;

  // Extract F/C suffix from last character
  char suffix = ' ';
  String numPart = s;
  if (s.length() > 0) {
    char last = s[s.length() - 1];
    if (last == 'F' || last == 'C') {
      suffix = last;
      numPart = s.substring(0, s.length() - 1);
      setDegree(amRed, amGreen, amBlue);
    } else {
      setDegree(0, 0, 0);
    }
  }

  int dotIdx = numPart.indexOf('.');

  if (suffix != ' ' || dotIdx >= 0) {
    String intPart, fracPart;
    if (dotIdx >= 0) {
      intPart = numPart.substring(0, dotIdx);
      fracPart = numPart.substring(dotIdx + 1);
    } else {
      intPart = numPart;
      fracPart = "";
    }

    String cleanInt = "";
    for (int i = 0; i < intPart.length(); i++)
      if (intPart[i] >= '0' && intPart[i] <= '9') cleanInt += intPart[i];

    String cleanFrac = "";
    for (int i = 0; i < fracPart.length(); i++)
      if (fracPart[i] >= '0' && fracPart[i] <= '9') cleanFrac += fracPart[i];

    // Determine if fractional part is effectively zero (only for F/C suffix)
    bool fracIsZero = false;
    if (suffix != ' ') {
      fracIsZero = true;
      for (int i = 0; i < cleanFrac.length(); i++) {
        if (cleanFrac[i] != '0') { fracIsZero = false; break; }
      }
    }

    if (fracIsZero) {
      // ── Integer display (F/C suffix, no meaningful fractional part) ──
      dot = false;
      if (cleanInt.length() > 3) cleanInt = cleanInt.substring(cleanInt.length() - 3);
      int len = cleanInt.length();
      if (len >= 3) {
        d0 = cleanInt[0]; d1 = cleanInt[1]; d2 = cleanInt[2];
      } else if (len == 2) {
        d1 = cleanInt[0]; d2 = cleanInt[1];
      } else if (len == 1) {
        d2 = cleanInt[0];
      }
      d3 = suffix;
    } else {
      // ── Decimal display ──
      dot = (dotIdx >= 0);

      // Suffix present, no dot, 1-2 digits: pad with 0
      if (suffix != ' ' && dotIdx < 0) {
        cleanFrac = "0";
        dot = true;
      }

      // Round to 1 decimal when suffix present and 2+ frac digits
      if (suffix != ' ' && cleanFrac.length() >= 2) {
        int roundDigit = cleanFrac[1] - '0';
        int tenths = cleanFrac[0] - '0';
        if (roundDigit >= 5) {
          tenths++;
          if (tenths > 9) {
            tenths = 0;
            int intVal = 0;
            for (int i = 0; i < cleanInt.length(); i++)
              intVal = intVal * 10 + (cleanInt[i] - '0');
            intVal++;
            if (intVal > 99) intVal = 99;
            cleanInt = String(intVal);
          }
        }
        cleanFrac = String(tenths);
      }

      // Integer → d0, d1
      if (cleanInt.length() > 2) cleanInt = cleanInt.substring(cleanInt.length() - 2);
      if (cleanInt.length() >= 2) {
        d0 = cleanInt[cleanInt.length() - 2];
        d1 = cleanInt[cleanInt.length() - 1];
      } else if (cleanInt.length() == 1) {
        d1 = cleanInt[0];
      }

      // Fractional → d2
      if (cleanFrac.length() > 0) d2 = cleanFrac[0];
      else if (dot) d2 = '0';

      if (suffix != ' ') {
        d3 = suffix;
      } else {
        if (cleanFrac.length() > 1) d3 = cleanFrac[1];
      }
    }
  } else {
    // No dot, no suffix: up to 4 digits, right-aligned
    String clean = "";
    for (int i = 0; i < numPart.length(); i++)
      if (numPart[i] >= '0' && numPart[i] <= '9') clean += numPart[i];
    if (clean.length() > 4) clean = clean.substring(clean.length() - 4);

    int len = clean.length();
    if (len >= 4) {
      d0 = clean[0]; d1 = clean[1]; d2 = clean[2]; d3 = clean[3];
    } else if (len == 3) {
      d1 = clean[0]; d2 = clean[1]; d3 = clean[2];
    } else if (len == 2) {
      d2 = clean[0]; d3 = clean[1];
    } else if (len == 1) {
      d3 = clean[0];
    }
  }

  // Draw
  setDigit(0, d0, digitRed, digitGreen, digitBlue);
  setDigit(1, d1, digitRed, digitGreen, digitBlue);
  setDigit(2, d2, digitRed, digitGreen, digitBlue);
  setDigit(3, d3, digitRed, digitGreen, digitBlue);

  if (dot) {
    setColon(0, 0, 0);
    setDecimalPoint(digitRed, digitGreen, digitBlue);
  } else {
    setColon(0, 0, 0);
  }

  setDash(0, 0, 0);
  setAM(0, 0, 0);
  render();
}   

void printLocalTime() {
  time_t now;
  struct tm timeinfo;

  time(&now);
  localtime_r(&now, &timeinfo);

  if (timeinfo.tm_year < 70) {
    Serial.println("Waiting for NTP time sync broadcast...");
    return;
  }
  setDegree(0, 0, 0);

  if (display_state) {
    if (showValue) {
      renderDisplayValue();
      return;
    }

    if (TOP_PRESSED) {
      setDigit(0, minute_digit1, digitRed, digitGreen, digitBlue);
      setDigit(1, minute_digit2, digitRed, digitGreen, digitBlue);
      setDigit(2, second_digit1, digitRed, digitGreen, digitBlue);
      setDigit(3, second_digit2, digitRed, digitGreen, digitBlue);
      setDash(digitRed, digitGreen, digitBlue);
      setColon(0, 0, 0);
      setAM(0, 0, 0);
      render();
    } else {
      setDash(0, 0, 0);
      if (timeinfo.tm_hour >= 12) {
        setAM(0, 0, 0);
      } else {
        setAM(amRed, amGreen, amBlue);
      }

      if (timeinfo.tm_sec % 2 == 0) {
        setColon(colonRed, colonGreen, colonBlue);
      } else {
        setColon(0, 0, 0);
      }

      char hour_digit1 = '0';
      char hour_digit2 = '0';
      char min_digit1 = '0';
      char min_digit2 = '0';

      int hour12 = timeinfo.tm_hour;

      if (hour12 == 0) {
        hour12 = 12;
      } else if (hour12 > 12) {
        hour12 = hour12 - 12;
      }

      if (hour12 < 10) {
        hour_digit1 = ' ';
      } else {
        hour_digit1 = (hour12 / 10) + '0';
      }
      hour_digit2 = (hour12 % 10) + '0';

      min_digit1 = (timeinfo.tm_min / 10) + '0';
      min_digit2 = (timeinfo.tm_min % 10) + '0';

      setDigit(0, hour_digit1, digitRed, digitGreen, digitBlue);
      setDigit(1, hour_digit2, digitRed, digitGreen, digitBlue);
      setDigit(2, min_digit1, digitRed, digitGreen, digitBlue);
      setDigit(3, min_digit2, digitRed, digitGreen, digitBlue);

      render();
    }
  } else {
    clearPanel();
    render();
  }
}

// --- MQTT ---
void setupMQTTTopics() {
  mqtt_client_id = "ESP32_" + WiFi.macAddress();
  mqtt_client_id.replace(":", "");

  topic_led_cmd = saved_mqtt_topic + "/light/led/cmd";
  topic_led_state = saved_mqtt_topic + "/light/led/state";

  topic_buzzer_cmd = saved_mqtt_topic + "/switch/buzzer/cmd";
  topic_buzzer_state = saved_mqtt_topic + "/switch/buzzer/state";

  topic_stopwatch_limit_cmd = saved_mqtt_topic + "/number/stopwatch_limit/cmd";
  topic_stopwatch_limit_state = saved_mqtt_topic + "/number/stopwatch_limit/state";

  topic_activity = saved_mqtt_topic + "/activity";

  topic_timer_toggle_cmd = saved_mqtt_topic + "/button/timer_toggle/cmd";
  topic_timer_reset_cmd = saved_mqtt_topic + "/button/timer_reset/cmd";
  topic_timer_state = saved_mqtt_topic + "/sensor/timer_state/state";
  topic_timer_elapsed = saved_mqtt_topic + "/sensor/timer_elapsed/state";

  topic_show_timer_cmd = saved_mqtt_topic + "/switch/show_timer/cmd";
  topic_show_timer_state = saved_mqtt_topic + "/switch/show_timer/state";

  topic_show_value_cmd = saved_mqtt_topic + "/switch/show_value/cmd";
  topic_show_value_state = saved_mqtt_topic + "/switch/show_value/state";
  topic_display_value_cmd = saved_mqtt_topic + "/text/display_value/cmd";
  topic_display_value_state = saved_mqtt_topic + "/text/display_value/state";
}

void sendLightStateToHA() {
  //int current_ha_brightness = map(g_current, 0, 0xD, 10, 255);

  int current_ha_brightness = levelToValue(g_current);

  String response_payload = "{";
  response_payload += "\"state\":\"" + String(display_state ? "ON" : "OFF") + "\",";
  response_payload += "\"brightness\":" + String(current_ha_brightness) + ",";
  response_payload += "\"color_mode\":\"rgb\",";
  response_payload += "\"color\":{";
  response_payload += "\"r\":" + String(digitRed * 127) + ",";
  response_payload += "\"g\":" + String(digitGreen * 127) + ",";
  response_payload += "\"b\":" + String(digitBlue * 127);
  response_payload += "}";
  response_payload += "}";

  mqttClient.publish(topic_led_state.c_str(), response_payload.c_str(), true);
  Serial.println("Reported unified JSON light state back to HA.");
}

void publishHomeAssistantDiscovery() {
  // 1. LED Light
  String led_discovery_topic = "homeassistant/light/" + mqtt_client_id + "/board_led/config";
  String led_payload = "{";
  led_payload += "\"name\":\"Show Display\",";
  led_payload += "\"unique_id\":\"" + mqtt_client_id + "_led\",";
  led_payload += "\"cmd_t\":\"" + topic_led_cmd + "\",";
  led_payload += "\"stat_t\":\"" + topic_led_state + "\",";
  led_payload += "\"schema\":\"json\",";
  led_payload += "\"brightness\":true,";
  led_payload += "\"supported_color_modes\":[\"rgb\"],";
  led_payload += "\"device\":{";
  led_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  led_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  led_payload += "\"model\":\"ESP32 Smart Clock\",";
  led_payload += "\"mf\":\"Bitsmarter\"";
  led_payload += "}";
  led_payload += "}";

  Serial.println("Sending LED discovery payload...");
  if (mqttClient.publish(led_discovery_topic.c_str(), led_payload.c_str(), true)) {
    Serial.println("LED Discovery: SUCCESS");
  } else {
    Serial.println("LED Discovery: FAILED");
  }

  // 2. Buzzer Switch
  String buzzer_discovery_topic = "homeassistant/switch/" + mqtt_client_id + "/board_buzzer/config";
  String buzzer_payload = "{";
  buzzer_payload += "\"name\":\"Buzzer\",";
  buzzer_payload += "\"unique_id\":\"" + mqtt_client_id + "_buzzer\",";
  buzzer_payload += "\"cmd_t\":\"" + topic_buzzer_cmd + "\",";
  buzzer_payload += "\"stat_t\":\"" + topic_buzzer_state + "\",";
  buzzer_payload += "\"payload_on\":\"ON\",";
  buzzer_payload += "\"payload_off\":\"OFF\",";
  buzzer_payload += "\"device\":{";
  buzzer_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  buzzer_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  buzzer_payload += "\"model\":\"ESP32 Smart Clock\",";
  buzzer_payload += "\"mf\":\"Bitsmarter\"";
  buzzer_payload += "}";
  buzzer_payload += "}";

  Serial.println("Sending Buzzer discovery payload...");
  if (mqttClient.publish(buzzer_discovery_topic.c_str(), buzzer_payload.c_str(), true)) {
    Serial.println("Buzzer Discovery: SUCCESS");
  } else {
    Serial.println("Buzzer Discovery: FAILED");
  }

  // 3. Timer Limit Number
  String num_discovery_topic = "homeassistant/number/" + mqtt_client_id + "/stopwatch_limit/config";
  String num_payload = "{";
  num_payload += "\"name\":\"Timer (min)\",";
  num_payload += "\"unique_id\":\"" + mqtt_client_id + "_stopwatch_limit\",";
  num_payload += "\"cmd_t\":\"" + topic_stopwatch_limit_cmd + "\",";
  num_payload += "\"stat_t\":\"" + topic_stopwatch_limit_state + "\",";
  num_payload += "\"min\":1,";
  num_payload += "\"max\":59,";
  num_payload += "\"step\":1,";
  num_payload += "\"device\":{";
  num_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  num_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  num_payload += "\"model\":\"ESP32 Smart Clock\",";
  num_payload += "\"mf\":\"Bitsmarter\"";
  num_payload += "}";
  num_payload += "}";

  Serial.println("Sending Timer Limit discovery payload...");
  if (mqttClient.publish(num_discovery_topic.c_str(), num_payload.c_str(), true)) {
    Serial.println("Timer Limit Discovery: SUCCESS");
  } else {
    Serial.println("Timer Limit Discovery: FAILED");
  }

  // 4. Stopwatch Activity Sensor
  String activity_discovery_topic = "homeassistant/sensor/" + mqtt_client_id + "/stopwatch_activity/config";
  String activity_payload = "{";
  activity_payload += "\"name\":\"Timer Activity\",";
  activity_payload += "\"unique_id\":\"" + mqtt_client_id + "_stopwatch_activity\",";
  activity_payload += "\"state_topic\":\"" + topic_activity + "\",";
  activity_payload += "\"icon\":\"mdi:timer-alert\",";
  activity_payload += "\"device\":{";
  activity_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  activity_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  activity_payload += "\"model\":\"ESP32 Smart Clock\",";
  activity_payload += "\"mf\":\"Bitsmarter\"";
  activity_payload += "}";
  activity_payload += "}";

  Serial.println("Sending Stopwatch Activity discovery payload...");
  if (mqttClient.publish(activity_discovery_topic.c_str(), activity_payload.c_str(), true)) {
    Serial.println("Stopwatch Activity Discovery: SUCCESS");
  } else {
    Serial.println("Stopwatch Activity Discovery: FAILED");
  }

  // 5. Timer Toggle Button
  String toggle_discovery_topic = "homeassistant/button/" + mqtt_client_id + "/timer_toggle/config";
  String toggle_payload = "{";
  toggle_payload += "\"name\":\"Timer Start/Stop\",";
  toggle_payload += "\"unique_id\":\"" + mqtt_client_id + "_timer_toggle\",";
  toggle_payload += "\"cmd_t\":\"" + topic_timer_toggle_cmd + "\",";
  toggle_payload += "\"icon\":\"mdi:timer-play\",";
  toggle_payload += "\"device\":{";
  toggle_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  toggle_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  toggle_payload += "\"model\":\"ESP32 Smart Clock\",";
  toggle_payload += "\"mf\":\"Bitsmarter\"";
  toggle_payload += "}";
  toggle_payload += "}";

  Serial.println("Sending Timer Toggle discovery payload...");
  if (mqttClient.publish(toggle_discovery_topic.c_str(), toggle_payload.c_str(), true)) {
    Serial.println("Timer Toggle Discovery: SUCCESS");
  } else {
    Serial.println("Timer Toggle Discovery: FAILED");
  }

  // 6. Timer Reset Button
  String reset_discovery_topic = "homeassistant/button/" + mqtt_client_id + "/timer_reset/config";
  String reset_payload = "{";
  reset_payload += "\"name\":\"Timer Reset\",";
  reset_payload += "\"unique_id\":\"" + mqtt_client_id + "_timer_reset\",";
  reset_payload += "\"cmd_t\":\"" + topic_timer_reset_cmd + "\",";
  reset_payload += "\"icon\":\"mdi:timer-cog-outline\",";
  reset_payload += "\"device\":{";
  reset_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  reset_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  reset_payload += "\"model\":\"ESP32 Smart Clock\",";
  reset_payload += "\"mf\":\"Bitsmarter\"";
  reset_payload += "}";
  reset_payload += "}";

  Serial.println("Sending Timer Reset discovery payload...");
  if (mqttClient.publish(reset_discovery_topic.c_str(), reset_payload.c_str(), true)) {
    Serial.println("Timer Reset Discovery: SUCCESS");
  } else {
    Serial.println("Timer Reset Discovery: FAILED");
  }

  // 7. Timer State Sensor
  String tstate_discovery_topic = "homeassistant/sensor/" + mqtt_client_id + "/timer_state/config";
  String tstate_payload = "{";
  tstate_payload += "\"name\":\"Timer State\",";
  tstate_payload += "\"unique_id\":\"" + mqtt_client_id + "_timer_state\",";
  tstate_payload += "\"stat_t\":\"" + topic_timer_state + "\",";
  tstate_payload += "\"icon\":\"mdi:timer\",";
  tstate_payload += "\"device\":{";
  tstate_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  tstate_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  tstate_payload += "\"model\":\"ESP32 Smart Clock\",";
  tstate_payload += "\"mf\":\"Bitsmarter\"";
  tstate_payload += "}";
  tstate_payload += "}";

  Serial.println("Sending Timer State discovery payload...");
  if (mqttClient.publish(tstate_discovery_topic.c_str(), tstate_payload.c_str(), true)) {
    Serial.println("Timer State Discovery: SUCCESS");
  } else {
    Serial.println("Timer State Discovery: FAILED");
  }

  // 8. Timer Elapsed Sensor
  String telapsed_discovery_topic = "homeassistant/sensor/" + mqtt_client_id + "/timer_elapsed/config";
  String telapsed_payload = "{";
  telapsed_payload += "\"name\":\"Timer Elapsed\",";
  telapsed_payload += "\"unique_id\":\"" + mqtt_client_id + "_timer_elapsed\",";
  telapsed_payload += "\"stat_t\":\"" + topic_timer_elapsed + "\",";
  telapsed_payload += "\"icon\":\"mdi:timer-outline\",";
  telapsed_payload += "\"device\":{";
  telapsed_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  telapsed_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  telapsed_payload += "\"model\":\"ESP32 Smart Clock\",";
  telapsed_payload += "\"mf\":\"Bitsmarter\"";
  telapsed_payload += "}";
  telapsed_payload += "}";

  Serial.println("Sending Timer Elapsed discovery payload...");
  if (mqttClient.publish(telapsed_discovery_topic.c_str(), telapsed_payload.c_str(), true)) {
    Serial.println("Timer Elapsed Discovery: SUCCESS");
  } else {
    Serial.println("Timer Elapsed Discovery: FAILED");
  }

  // 9. Show Timer Switch
  String showtimer_discovery_topic = "homeassistant/switch/" + mqtt_client_id + "/show_timer/config";
  String showtimer_payload = "{";
  showtimer_payload += "\"name\":\"Show Timer\",";
  showtimer_payload += "\"unique_id\":\"" + mqtt_client_id + "_show_timer\",";
  showtimer_payload += "\"cmd_t\":\"" + topic_show_timer_cmd + "\",";
  showtimer_payload += "\"stat_t\":\"" + topic_show_timer_state + "\",";
  showtimer_payload += "\"payload_on\":\"ON\",";
  showtimer_payload += "\"payload_off\":\"OFF\",";
  showtimer_payload += "\"icon\":\"mdi:timer\",";
  showtimer_payload += "\"device\":{";
  showtimer_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  showtimer_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  showtimer_payload += "\"model\":\"ESP32 Smart Clock\",";
  showtimer_payload += "\"mf\":\"Bitsmarter\"";
  showtimer_payload += "}";
  showtimer_payload += "}";

  Serial.println("Sending Show Timer discovery payload...");
  if (mqttClient.publish(showtimer_discovery_topic.c_str(), showtimer_payload.c_str(), true)) {
    Serial.println("Show Timer Discovery: SUCCESS");
  } else {
    Serial.println("Show Timer Discovery: FAILED");
  }

  // 10. Show Value Switch
  String showval_discovery_topic = "homeassistant/switch/" + mqtt_client_id + "/show_value/config";
  String showval_payload = "{";
  showval_payload += "\"name\":\"Show Value\",";
  showval_payload += "\"unique_id\":\"" + mqtt_client_id + "_show_value\",";
  showval_payload += "\"cmd_t\":\"" + topic_show_value_cmd + "\",";
  showval_payload += "\"stat_t\":\"" + topic_show_value_state + "\",";
  showval_payload += "\"payload_on\":\"ON\",";
  showval_payload += "\"payload_off\":\"OFF\",";
  showval_payload += "\"icon\":\"mdi:numeric\",";
  showval_payload += "\"device\":{";
  showval_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  showval_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  showval_payload += "\"model\":\"ESP32 Smart Clock\",";
  showval_payload += "\"mf\":\"Bitsmarter\"";
  showval_payload += "}";
  showval_payload += "}";

  Serial.println("Sending Show Value discovery payload...");
  if (mqttClient.publish(showval_discovery_topic.c_str(), showval_payload.c_str(), true)) {
    Serial.println("Show Value Discovery: SUCCESS");
  } else {
    Serial.println("Show Value Discovery: FAILED");
  }
  // 11. Display Value Text
  String dispval_discovery_topic = "homeassistant/text/" + mqtt_client_id + "/display_value/config";
  String dispval_payload = "{";
  dispval_payload += "\"name\":\"Display Value\",";
  dispval_payload += "\"unique_id\":\"" + mqtt_client_id + "_display_value\",";
  dispval_payload += "\"cmd_t\":\"" + topic_display_value_cmd + "\",";
  dispval_payload += "\"stat_t\":\"" + topic_display_value_state + "\",";
  dispval_payload += "\"max_length\":5,";
  dispval_payload += "\"mode\":\"text\",";
  dispval_payload += "\"icon\":\"mdi:thermometer\",";
  dispval_payload += "\"device\":{";
  dispval_payload += "\"ids\":[\"" + mqtt_client_id + "\"],";
  dispval_payload += "\"name\":\"" + saved_mqtt_topic + "\",";
  dispval_payload += "\"model\":\"ESP32 Smart Clock\",";
  dispval_payload += "\"mf\":\"Bitsmarter\"";
  dispval_payload += "}";
  dispval_payload += "}";

  Serial.println("Sending Display Value discovery payload...");
  if (mqttClient.publish(dispval_discovery_topic.c_str(), dispval_payload.c_str(), true)) {
    Serial.println("Display Value Discovery: SUCCESS");
  } else {
    Serial.println("Display Value Discovery: FAILED");
  }   
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");

  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // 1. LED JSON
  if (String(topic) == topic_led_cmd) {
    if (message.indexOf("\"state\":\"ON\"") >= 0) {
      display_state = true;
    } else if (message.indexOf("\"state\":\"OFF\"") >= 0) {
      display_state = false;
    }

    int brKeyIndex = message.indexOf("\"brightness\":");
    if (brKeyIndex >= 0) {
      int startIdx = brKeyIndex + 13;
      int endIdx = message.indexOf('}', startIdx);
      if (message.indexOf(',', startIdx) >= 0 && message.indexOf(',', startIdx) < endIdx) {
        endIdx = message.indexOf(',', startIdx);
      }

      int incomingBrightness = message.substring(startIdx, endIdx).toInt();
      incomingBrightness = constrain(incomingBrightness, 0, 255);

      //uint8_t targetCurrentReg = (uint8_t)map(incomingBrightness, 0, 255, 0, 13);
      
      uint8_t targetCurrentReg = mapToLevel(incomingBrightness);
            
      setCurrentValue(targetCurrentReg);
      Serial.printf("Dimming adjusted via JSON -> Register: 0x%X\n", g_current);
    }

    int rKeyIndex = message.indexOf("\"r\":");
    int gKeyIndex = message.indexOf("\"g\":");
    int bKeyIndex = message.indexOf("\"b\":");

    if (rKeyIndex >= 0 && gKeyIndex >= 0 && bKeyIndex >= 0) {
      auto parseValue = [](const String& msg, int keyIdx) {
        int start = keyIdx + 4;
        int end = msg.indexOf(',', start);
        if (end < 0 || end > msg.indexOf('}', start)) {
          end = msg.indexOf('}', start);
        }
        return msg.substring(start, end).toInt();
      };

      int rawR = parseValue(message, rKeyIndex);
      int rawG = parseValue(message, gKeyIndex);
      int rawB = parseValue(message, bKeyIndex);

      int targetR = constrain(rawR / 86, 0, 2);
      int targetG = constrain(rawG / 86, 0, 2);
      int targetB = constrain(rawB / 86, 0, 2);

      setClockColors(targetR, targetG, targetB,
                     targetR, targetG, targetB,
                     targetR, targetG, targetB,
                     0, 0, 0);

      Serial.printf("Colors parsed and scaled via Wheel -> R:%d, G:%d, B:%d\n", targetR, targetG, targetB);
    }
    sendLightStateToHA();
  }

  // 2. Buzzer Switch
  else if (String(topic) == topic_buzzer_cmd) {
    if (message.equals("ON") || message.indexOf("\"state\":\"ON\"") >= 0) {
      buzzer_state = true;
      mqttClient.publish(topic_buzzer_state.c_str(), "ON", true);
      Serial.println("Buzzer tracking status: ON");
    } else if (message.equals("OFF") || message.indexOf("\"state\":\"OFF\"") >= 0) {
      buzzer_state = false;
      mqttClient.publish(topic_buzzer_state.c_str(), "OFF", true);
      Serial.println("Buzzer tracking status: OFF");
    }
  }

  // 3. Timer Limit
  else if (String(topic) == topic_stopwatch_limit_cmd) {
    int newVal = constrain(message.toInt(), 1, 59);
    if (newVal != stopWatchLimitMinutes) {
      stopWatchLimitMinutes = newVal;
      preferences.begin("app-cfg", false);
      preferences.putInt("stopwatch_limit", newVal);
      preferences.end();
      Serial.printf("Stopwatch limit set to %d minutes\n", stopWatchLimitMinutes);
    }
    mqttClient.publish(topic_stopwatch_limit_state.c_str(), String(stopWatchLimitMinutes).c_str(), true);
  }

  // 4. Timer Toggle (start/stop)
  else if (String(topic) == topic_timer_toggle_cmd) {
    if (stopWatchRunning) {
      stopWatchRunning = false;
      lastTick = millis();
    } else {
      if (stopWatchAtLimit) {
        stopWatchAtLimit = false;
        stopWatchSeconds = 0;
      }
      stopWatchRunning = true;
      lastTick = millis();
    }
    Serial.println(stopWatchRunning ? "Timer started via MQTT" : "Timer stopped via MQTT");
    publishTimerState();
  }

  // 5. Timer Reset (stop + zero)
  else if (String(topic) == topic_timer_reset_cmd) {
    stopWatchRunning = false;
    stopWatchSeconds = 0;
    stopWatchAtLimit = false;
    alarmActive = false;
    noTone(BUZZER_PIN);
    isCurrentlyPlaying = false;
    lastTick = millis();
    Serial.println("Timer reset via MQTT");
    publishTimerState();
  }

  // 6. Show Timer Switch
  else if (String(topic) == topic_show_timer_cmd) {
    if (message.equals("ON") || message.indexOf("\"state\":\"ON\"") >= 0) {
      TOP_PRESSED = true;
    } else if (message.equals("OFF") || message.indexOf("\"state\":\"OFF\"") >= 0) {
      TOP_PRESSED = false;
    }
    Serial.println(TOP_PRESSED ? "Show Timer: ON (via MQTT)" : "Show Timer: OFF (via MQTT)");
    publishShowTimerState();
  }

  // 7. Show Value Switch
  else if (String(topic) == topic_show_value_cmd) {
    if (message.equals("ON") || message.indexOf("\"state\":\"ON\"") >= 0) {
      showValue = true;
    } else if (message.equals("OFF") || message.indexOf("\"state\":\"OFF\"") >= 0) {
      showValue = false;
    }
    Serial.println(showValue ? "Show Value: ON" : "Show Value: OFF");
    publishShowValueState();
  }

  // 8. Display Value Text
  else if (String(topic) == topic_display_value_cmd) {
    displayValue = message;
    if (displayValue.length() > 5) displayValue = displayValue.substring(0, 5);
    Serial.printf("Display Value set to: %s\n", displayValue.c_str());
    publishShowValueState();
  }
}

void reconnectMQTT() {
  if (saved_mqtt_host.length() == 0) return;

  static unsigned long last_reconnect_attempt = 0;
  unsigned long now = millis();

  if (!mqttClient.connected() && (now - last_reconnect_attempt > 5000)) {
    last_reconnect_attempt = now;
    Serial.print("Attempting MQTT connection to ");
    Serial.print(saved_mqtt_host);
    Serial.println("...");

    bool success = false;
    if (saved_mqtt_user.length() > 0) {
      success = mqttClient.connect(mqtt_client_id.c_str(), saved_mqtt_user.c_str(), saved_mqtt_pass.c_str());
    } else {
      success = mqttClient.connect(mqtt_client_id.c_str());
    }

    if (success) {
      Serial.println("MQTT connected!");
      mqttClient.subscribe(topic_led_cmd.c_str());
      mqttClient.subscribe(topic_buzzer_cmd.c_str());
      mqttClient.subscribe(topic_stopwatch_limit_cmd.c_str());
      mqttClient.subscribe(topic_timer_toggle_cmd.c_str());
      mqttClient.subscribe(topic_timer_reset_cmd.c_str());
      mqttClient.subscribe(topic_show_timer_cmd.c_str());
      mqttClient.subscribe(topic_show_value_cmd.c_str());
      mqttClient.subscribe(topic_display_value_cmd.c_str());

      sendLightStateToHA();

      if (digitalRead(BUZZER_PIN) == HIGH) {
        mqttClient.publish(topic_buzzer_state.c_str(), "ON", true);
      } else {
        mqttClient.publish(topic_buzzer_state.c_str(), "OFF", true);
      }

      publishHomeAssistantDiscovery();
      publishTimerState();
      publishShowTimerState();
      publishShowValueState();
    } else {
      Serial.printf("MQTT connection failed, rc=%d. Will try again in 5s.\n", mqttClient.state());
    }
  }
}

// --- Buzzer ---

void handleTone(bool state) {
  if (alarmActive) return;
  unsigned long currentMillis = millis();

  if (isCurrentlyPlaying) {
    if (currentMillis - toneStartTime >= TONE_DURATION) {
      noTone(BUZZER_PIN);
      isCurrentlyPlaying = false;
    }
    return;
  }
  if (state) {
    tone(BUZZER_PIN, TONE_FREQ, TONE_DURATION / 2);
    toneStartTime = currentMillis;
    isCurrentlyPlaying = true;
  }
}

void handleAlarm() {
  if (!alarmActive) return;
  unsigned long now = millis();

  if (now - alarmStartTime >= ALARM_DURATION_MS) {
    alarmActive = false;
    noTone(BUZZER_PIN);
    isCurrentlyPlaying = false;
    Serial.println("Alarm finished.");

    Serial.printf("Publishing 'finished' to %s (connected=%d)\n", topic_activity.c_str(), mqttClient.connected());

    if (mqttClient.connected()) {
      mqttClient.publish(topic_activity.c_str(), "Limit reached", true);
    }

    return;
  }

  unsigned long elapsed = now - alarmStartTime;
  unsigned long cyclePos = elapsed % 1000;

  if (cyclePos < 500) {
    if (!isCurrentlyPlaying) {
      tone(BUZZER_PIN, TONE_FREQ, TONE_DURATION / 2);
      isCurrentlyPlaying = true;
    }
  } else {
    if (isCurrentlyPlaying) {
      noTone(BUZZER_PIN);
      isCurrentlyPlaying = false;
    }
  }
}

// --- Setup & Loop ---
void setup() {
  Serial.begin(115200);
  delay(200);

  // Buzzer PWM
  ledcSetup(4, 2000, 8);
  ledcAttachPin(BUZZER_PIN, 4);

  // GPIO
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(TOP_BUTTON, INPUT_PULLUP);
  pinMode(BOTTOM_BUTTON, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(TOP_BUTTON), topButtonISR, FALLING);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Display bus
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLK_1, OUTPUT);
  pinMode(PIN_DATA_1, OUTPUT);
  GPIO.out_w1tc = M_CLK | M_DATA | M_CLK_1 | M_DATA_1;

  // The part discards anything sent in the first 200us after power up.
  delay(10);

  clearPanel();
  render();

  // Scan timer
  scanTimer = timerBegin(3, 80, true);           // Timer 3, prescaler 80 → 1 MHz (1 µs/tick)
  timerAttachInterrupt(scanTimer, &scanTick, true);
  timerAlarmWrite(scanTimer, TICK_US, true);     // 301 µs, auto-reload
  timerAlarmEnable(scanTimer); 

  // Load preferences from NVS
  preferences.begin("app-cfg", false);
  saved_tz_index = preferences.getInt("tz_index", 0);
  saved_mqtt_host = preferences.getString("mqtt_host", "");
  saved_mqtt_port = preferences.getInt("mqtt_port", 1883);
  saved_mqtt_user = preferences.getString("mqtt_user", "");
  saved_mqtt_pass = preferences.getString("mqtt_pass", "");
  saved_mqtt_topic = preferences.getString("mqtt_topic", "LEDClock01");

  int saved_stopwatch_limit = preferences.getInt("stopwatch_limit", 60);
  stopWatchLimitMinutes = constrain(saved_stopwatch_limit, 1, 59);
  Serial.printf("Loaded saved stopwatch limit: %d min\n", stopWatchLimitMinutes);

  Serial.printf("Loaded saved timezone index from flash: %d\n", saved_tz_index);
  Serial.printf("Loaded saved MQTT Host: %s\n", saved_mqtt_host.c_str());
  Serial.printf("Loaded saved MQTT Port: %d\n", saved_mqtt_port);
  Serial.printf("Loaded saved MQTT Topic from flash: %s\n", saved_mqtt_topic.c_str());

  // WiFiManager portal
  String dropdown_html = "<br/><label for='tz_id'>Choose Australia Timezone</label><br/>";
  dropdown_html += "<select id='tz_select' class='button' onchange='document.getElementById(\"tz_id\").value = this.value;'>";

  for (int i = 0; i < 6; i++) {
    dropdown_html += "<option value='";
    dropdown_html += String(i);
    dropdown_html += "'";
    if (i == saved_tz_index)
      dropdown_html += " selected";
    dropdown_html += ">";
    if (i == 0) dropdown_html += "AEST / AEDT (Sydney, Melbourne, Canberra, Hobart)";
    else if (i == 1) dropdown_html += "AEST (Brisbane / Queensland)";
    else if (i == 2) dropdown_html += "ACST / ACDT (Adelaide)";
    else if (i == 3) dropdown_html += "ACST (Darwin / NT)";
    else if (i == 4) dropdown_html += "AWST (Perth / WA)";
    else if (i == 5) dropdown_html += "LHST / LHDT (Lord Howe Island)";
    dropdown_html += "</option>";
  }
  dropdown_html += "</select><br/><br/>";

  WiFiManagerParameter custom_html_injector(dropdown_html.c_str());
  wm.addParameter(&custom_html_injector);

  char default_val_str[8];
  itoa(saved_tz_index, default_val_str, 10);
  WiFiManagerParameter custom_tz_hidden("tz_id", "", default_val_str, 2, "type='hidden'");
  wm.addParameter(&custom_tz_hidden);
  WiFiManagerParameter custom_mqtt_host("mqtt_host_id", "MQTT Host IP", saved_mqtt_host.c_str(), 40);
  wm.addParameter(&custom_mqtt_host);
  char port_str[16];
  itoa(saved_mqtt_port, port_str, 10);
  WiFiManagerParameter custom_mqtt_port("mqtt_port_id", "MQTT Port", port_str, 6, "type='number'");
  wm.addParameter(&custom_mqtt_port);
  WiFiManagerParameter custom_mqtt_user("mqtt_user_id", "MQTT User", saved_mqtt_user.c_str(), 40);
  wm.addParameter(&custom_mqtt_user);
  WiFiManagerParameter custom_mqtt_pass("mqtt_pass_id", "MQTT Password", saved_mqtt_pass.c_str(), 40, "type='password'");
  wm.addParameter(&custom_mqtt_pass);
  const char *pass_toggle_html = " Show Password";
  WiFiManagerParameter custom_pass_toggle(pass_toggle_html);
  wm.addParameter(&custom_pass_toggle);
  WiFiManagerParameter custom_mqtt_input("mqtt_id", "Set MQTT root topic", saved_mqtt_topic.c_str(), 40);
  wm.addParameter(&custom_mqtt_input);

  Serial.println("Starting Open WiFiManager AP/Client routine...");
  if (!wm.autoConnect(AP_SSID)) {
    Serial.println("Failed to connect or portal timed out. Restarting...");
    delay(3000);
    ESP.restart();
  }

  // Parse portal values
  String selected_index_str = custom_tz_hidden.getValue();
  int current_selected_index = selected_index_str.toInt();
  String current_mqtt_host = custom_mqtt_host.getValue();
  int current_mqtt_port = String(custom_mqtt_port.getValue()).toInt();
  String current_mqtt_user = custom_mqtt_user.getValue();
  String current_mqtt_pass = custom_mqtt_pass.getValue();
  String current_mqtt_topic = custom_mqtt_input.getValue();
  if (current_selected_index < 0 || current_selected_index > 5) current_selected_index = 0;
  current_mqtt_host.trim();
  current_mqtt_user.trim();
  current_mqtt_pass.trim();
  current_mqtt_topic.trim();

  // Persist to flash if changed
  if (current_selected_index != saved_tz_index) {
    preferences.putInt("tz_index", current_selected_index);
    saved_tz_index = current_selected_index;
  }
  if (current_mqtt_host != saved_mqtt_host) {
    preferences.putString("mqtt_host", current_mqtt_host);
    saved_mqtt_host = current_mqtt_host;
  }
  if (current_mqtt_port != saved_mqtt_port && current_mqtt_port > 0) {
    preferences.putInt("mqtt_port", current_mqtt_port);
    saved_mqtt_port = current_mqtt_port;
  }
  if (current_mqtt_user != saved_mqtt_user) {
    preferences.putString("mqtt_user", current_mqtt_user);
    saved_mqtt_user = current_mqtt_user;
  }
  if (current_mqtt_pass != saved_mqtt_pass) {
    preferences.putString("mqtt_pass", current_mqtt_pass);
    saved_mqtt_pass = current_mqtt_pass;
  }
  if (current_mqtt_topic != saved_mqtt_topic && current_mqtt_topic.length() > 0) {
    preferences.putString("mqtt_topic", current_mqtt_topic);
    saved_mqtt_topic = current_mqtt_topic;
  }
  preferences.end();

  // Timezone & NTP
  const char *chosen_tz_string = australian_tz_strings[saved_tz_index];
  configTzTime(chosen_tz_string, NTP_SERVER);

  // MQTT
  setupMQTTTopics();
  if (saved_mqtt_host.length() > 0) {
    mqttClient.setServer(saved_mqtt_host.c_str(), saved_mqtt_port);
    mqttClient.setBufferSize(1024);
    mqttClient.setCallback(mqttCallback);
  }

  Serial.println("LED Clock Started");
  setClockColors(0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 0, 0);
  setCurrentValue(0xD);
}

void loop() {
  checkButtonForReset();

  if (!mqttClient.connected()) {
    reconnectMQTT();
  } else {
    mqttClient.loop();
  }

  static unsigned long last_print = 0;
  if (millis() - last_print > 500) {
    last_print = millis();
    printLocalTime();
  }
  
  handleTone(buzzer_state);
  handleAlarm();
  updateStopWatch();
}   
