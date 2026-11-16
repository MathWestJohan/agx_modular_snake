/*
  ============================================================================
   SnakeBot - MOTOR ESP32   (runs the Servo2 and the gait)
  ============================================================================
   Board : ESP32-DEV-38P ("ESP32 Dev Module"), Arduino core 2.x or 3.x
   Parts : M5Stack Servo2 (PCA9685) + 5 x Parallax Standard Servo
           5 modules, joints pitch-yaw-pitch-yaw-pitch
   Partner sketch: snake_sensors.ino (sensors + Wi-Fi dashboard)

   This board only moves the snake. It gets its orders from the sensor
   ESP32 over a UART wire, 20 times per second, and reports the joint
   angles back. If the orders stop arriving (cable loose, sensor board
   crashed), it fades the gait out and stops by itself.

   This is esp32_code/snake_motor.ino plus what the simulation needs. Every
   change is marked "SIM":
     - The waves are made exactly as in the simulation (sim_gait.h, checked
       against the simulation by firmware/test).
     - The caterpillar wave runs from the TAIL to the HEAD by default, which is
       what moves the snake head first in the simulation. The first sketch ran
       it from the head to the tail. See CATERPILLAR_WAVE below.
     - Orders carry one more number: a constant bend of the pitch joints (the
       simulation's hilltop gait). Orders without it still work.

   Libraries: none (the Servo2 chip is driven directly, see "PCA9685" below)

  ---------------------------------------------------------------------------
   WIRING  (all grounds connected together!)
  ---------------------------------------------------------------------------
   Servo2 (I2C):   GPIO21 -> SDA    GPIO22 -> SCL    3.3V, GND
   Servos:         Servo2 ports 0, 1, 2, 8, 9  (joint 1 = head end)
   Servo power:    battery 6-12 V into the Servo2 XT60, never into the ESP32
   Power-only ports (5V + GND used, signal pin left unconnected):
                   12, 13 -> Sharp IR sensors   15 -> motor ESP32 5V   7 -> sensor ESP32 5V

   UART link to the sensor ESP32 (cross TX and RX!):
     motor GPIO17 (TX) -> sensor GPIO16 (RX)
     motor GPIO16 (RX) <- sensor GPIO17 (TX)
     motor GND         -- sensor GND

  ---------------------------------------------------------------------------
   USB Serial Monitor (115200) for testing this board on its own:
     0 stop | 1 caterpillar | 2 serpentine | 3 sidewind
     +/- amplitude | f/s faster/slower | r reverse | ? status
     d  diagnostics: reads back the pulse the Servo2 is really sending
     p  pitch test (while stopped): every pitch joint to -15 deg for 3 s.
        Both ends of the snake must go DOWN. See DIRECTION below.
   (Once the sensor board is talking, its orders take over.)
  ============================================================================
*/

#include <Wire.h>
#include "sim_gait.h"                         // SIM: the simulation's wave formula

// ============================ CONFIGURE ME ==================================
// ---- Pins ----
const int SDA_PIN = 21, SCL_PIN = 22;
const int LINK_RX = 16, LINK_TX = 17;         // UART2 to the sensor ESP32
const uint32_t LINK_BAUD = 38400;             // MUST match snake_sensors.ino. Slower = more noise-proof
const uint32_t LINK_TIMEOUT_MS = 1000;        // no valid orders for this long -> stop
const int LED_PIN = 2;                        // lit while the snake moves

// ---- Servo2 (PCA9685) ----
const uint8_t SERVO2_ADDR = 0x40;
const float   PCA_OSC_HZ  = 27000000;

// ---- Joints: joint 1 is at the head ----
const int NUM_JOINTS = 5;
const uint8_t PORT[NUM_JOINTS] = {0, 1, 2, 8, 9};

// Servo2 ports used ONLY for their 5V + GND pins (power for other parts).
// Their signal pin is forced permanently LOW and no joint may use them.
//   12, 13 = Sharp IR sensors   15 = motor ESP32   7 = sensor ESP32
const uint8_t RESERVED_PORTS[] = {7, 12, 13, 15};

enum JointType { PITCH, YAW };
const JointType TYPE[NUM_JOINTS] = {PITCH, YAW, PITCH, YAW, PITCH};
const int DIRECTION[NUM_JOINTS] = {1, 1, 1, 1, 1};   // -1 for a servo that turns the "wrong" way
// SIM: the simulation's gaits need this sign on the PITCH joints: a NEGATIVE angle bends the
// snake like a hilltop (both ends down), a positive angle like a valley (both ends up).
// Check with the 'p' command: both ends must go DOWN. A pitch joint that lifts its end
// instead gets DIRECTION -1. (The sensor board's slope measurement relies on this too.)
int TRIM_US[NUM_JOINTS]         = {0, -200, -160, -150, -120};   // small corrections so 0 deg is truly straight

// ---- Servo limits (Parallax Standard Servo) ----
const float CENTER_US   = 1500.0;
const float US_PER_DEG  = 1500.0 / 180.0;     // ~8.3 us per degree
const float MIN_US      = 750.0;
const float MAX_US      = 2250.0;
const float MAX_AMP_DEG = 45.0;

// ---- Steering ----
// Steering bends all yaw joints the same way. Check: send +15 steering;
// if the snake turns LEFT, set STEER_SIGN = -1.
const float STEER_SIGN     = 1.0;
const float MAX_STEER_DEG  = 30.0;
const float STEER_RATE_DPS = 30.0;            // max change in steering per second

// ---- SIM: caterpillar wave direction and pitch offset ----
// +1 = the pitch wave runs from the tail to the head. In the simulation that moves the
//      snake HEAD FIRST (a caterpillar's hump also travels from its tail to its head).
// -1 = from the head to the tail, as esp32_code/snake_motor.ino did.
// Check on the floor: caterpillar with "forward" on the dashboard must crawl head first.
// If it crawls tail first, set this to -1 (and tell us: the simulation then disagrees).
const int   CATERPILLAR_WAVE     = 1;
const float MAX_PITCH_OFFSET_DEG = 20.0;      // limit for the pitch offset in an order
const float OFFSET_RATE_DPS      = 10.0;      // max change of the pitch offset per second
const float PITCH_TEST_DEG       = -15.0;     // 'p' command
const uint32_t PITCH_TEST_MS     = 3000;

// ---- Timing ----
const uint32_t CONTROL_MS = 20;               // 50 Hz servo update
const uint32_t REPORT_MS  = 50;               // 20 Hz report to the sensor board
const float RAMP_PER_SEC  = 0.5;              // how fast a gait fades in/out
// ============================================================================

HardwareSerial& linkPort = Serial2;   // UART2

enum Mode { STOP, VERTICAL, LATERAL, SIDEWIND };
Mode mode = STOP, requestedMode = STOP;

// Orders (from the sensor board, or from USB when testing alone)
float amplitudeDeg = 20.0;
float frequencyHz  = 0.5;
float phaseLagDeg  = 120.0;
int   travelDir    = 1;
float steerTargetDeg = 0.0;
float pitchOffsetTargetDeg = 0.0;             // SIM: constant bend of the pitch joints (- = hilltop shape)

// Gait state
float ramp = 0.0, phase = 0.0, steerDeg = 0.0;
float pitchOffsetDeg = 0.0;                   // SIM: pitch offset in use (follows the target slowly)
bool  pitchTest = false;                      // SIM: 'p' command running
uint32_t pitchTestStart = 0;
float jointDeg[NUM_JOINTS];

bool  servoOk = false;
float pcaUsPerTick = 20000.0 / 4096.0;

bool linkSeen = false;                         // have we ever heard the sensor board?
uint32_t lastLinkMs = 0, badFrames = 0, goodFrames = 0;
uint32_t lastControl = 0, lastReport = 0, lastProbe = 0, lastWatchdog = 0;
uint32_t pcaReinits = 0;
int      pcaReadFails = 0;

// ================================================ PCA9685 (Servo2 chip) =====
// Our own tiny driver instead of the Adafruit library. The library sets the
// chip up by reading MODE1, changing it and writing it back; one bad read
// there can leave the chip asleep with the external-clock bit stuck on
// (that is exactly what we saw: MODE1 = 0xF0, limp servos). Here every
// setting is written explicitly and then checked.
const uint8_t PCA_MODE1 = 0x00, PCA_MODE2 = 0x01, PCA_LED0 = 0x06, PCA_PRESCALE = 0xFE;
const uint8_t M1_RESTART = 0x80, M1_EXTCLK = 0x40, M1_AI = 0x20, M1_SLEEP = 0x10;
const uint8_t PCA_PRESCALE_50HZ = (uint8_t)(PCA_OSC_HZ / (50 * 4096.0f) + 0.5f - 1);   // = 131

bool pcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(SERVO2_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool pcaSetPWM(uint8_t ch, uint16_t on, uint16_t off) {
  Wire.beginTransmission(SERVO2_ADDR);
  Wire.write(PCA_LED0 + 4 * ch);
  Wire.write(on & 0xFF);  Wire.write(on >> 8);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  return Wire.endTransmission() == 0;
}

void writeJoint(int j, float angleDeg) {
  float us = CENTER_US + TRIM_US[j] + DIRECTION[j] * angleDeg * US_PER_DEG;
  us = constrain(us, MIN_US, MAX_US);
  // Never send 4096 or more: the chip treats that as "output fully OFF"
  uint16_t ticks = (uint16_t)constrain(us / pcaUsPerTick + 0.5f, 1.0f, 4095.0f);
  pcaSetPWM(PORT[j], 0, ticks);
}

// Read registers straight from the PCA9685
bool pcaRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(SERVO2_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(SERVO2_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// 'd' command: shows what the chip is REALLY outputting on each joint port
void printServoDiagnostics() {
  uint8_t mode1 = 0, pre = 0;
  if (!pcaRead(0x00, &mode1, 1) || !pcaRead(0xFE, &pre, 1)) {
    Serial.println("DIAG: cannot read the Servo2 chip over I2C");
    return;
  }
  Serial.printf("DIAG: MODE1=0x%02X%s%s | prescale=%u (expect %u) | %.3f us per tick | re-inits %lu\n",
                mode1, (mode1 & M1_SLEEP) ? "  <-- SLEEP: outputs stopped!" : "  (awake, good)",
                (mode1 & M1_EXTCLK) ? "  <-- EXTCLK stuck on!" : "",
                pre, PCA_PRESCALE_50HZ, pcaUsPerTick, (unsigned long)pcaReinits);
  for (int j = 0; j < NUM_JOINTS; j++) {
    uint8_t r[4];
    if (!pcaRead(0x06 + 4 * PORT[j], r, 4)) { Serial.printf("  port %d: read failed\n", PORT[j]); continue; }
    uint16_t on = r[0] | (r[1] << 8), off = r[2] | (r[3] << 8);
    if (off & 0x1000)     Serial.printf("  port %d: FULL OFF (no pulse - servo will be limp)\n", PORT[j]);
    else if (on & 0x1000) Serial.printf("  port %d: FULL ON (constant high - wrong)\n", PORT[j]);
    else Serial.printf("  port %d: pulse %.0f us (should be 750-2250)  [joint %d wants %.1f deg]\n",
                       PORT[j], ((off - on) & 0x0FFF) * (pre + 1) * 1e6f / PCA_OSC_HZ, j + 1, jointDeg[j]);
  }
}

// Full, explicit set-up of the chip. Returns true if it is awake at 50 Hz.
bool pcaInit() {
  // 1. Software reset (I2C "general call" 0x00, byte 0x06). This is the only
  //    command, apart from cutting power, that clears a stuck EXTCLK bit.
  Wire.beginTransmission(0x00);
  Wire.write(0x06);
  Wire.endTransmission();
  delay(10);                                              // chip is now asleep, defaults loaded

  // 2. Prescaler can only be written while asleep
  bool ok = pcaWrite(PCA_MODE1, M1_SLEEP | M1_AI)
         && pcaWrite(PCA_PRESCALE, PCA_PRESCALE_50HZ)
         && pcaWrite(PCA_MODE2, 0x04);                    // totem-pole outputs (servo standard)

  // 3. Ports that are power-only: signal permanently off. Joints: centre.
  for (uint8_t ch : RESERVED_PORTS) pcaSetPWM(ch, 0, 4096);       // 4096 = fully OFF
  for (int j = 0; j < NUM_JOINTS; j++) writeJoint(j, jointDeg[j]);

  // 4. Wake up: internal oscillator, auto-increment, NOT sleeping
  ok = ok && pcaWrite(PCA_MODE1, M1_AI);
  delayMicroseconds(600);                                 // oscillator needs 500 us to start

  // 5. Check it really worked
  uint8_t mode1 = 0xFF, pre = 0;
  ok = ok && pcaRead(PCA_MODE1, &mode1, 1) && pcaRead(PCA_PRESCALE, &pre, 1);
  if (!ok) { Serial.println("Servo2: I2C write/read failed during set-up"); return false; }
  if (mode1 & M1_EXTCLK) {
    Serial.printf("Servo2: MODE1=0x%02X - EXTCLK still stuck. Unplug ALL power (USB + battery) for 5 s.\n", mode1);
    return false;
  }
  if ((mode1 & M1_SLEEP) || pre != PCA_PRESCALE_50HZ) {
    Serial.printf("Servo2: set-up did not stick (MODE1=0x%02X, prescale=%u)\n", mode1, pre);
    return false;
  }
  Serial.printf("Servo2 awake: MODE1=0x%02X, prescale %u (50 Hz), centre = %.0f ticks\n",
                mode1, pre, 1500 / pcaUsPerTick);
  return true;
}

void probeServo2() {
  Wire.beginTransmission(SERVO2_ADDR);
  if (Wire.endTransmission() != 0) return;
  pcaUsPerTick = (PCA_PRESCALE_50HZ + 1) * 1e6f / PCA_OSC_HZ;     // one PWM tick in us
  for (int j = 0; j < NUM_JOINTS; j++) jointDeg[j] = 0;
  if (!pcaInit()) return;                                 // retried every 2 s by loop()
  servoOk = true;
  Serial.println("Servo2 found, joints straight");
}

// Called every second: if the chip ever falls asleep again, wake it and say so
void pcaWatchdog() {
  uint8_t mode1;
  if (!pcaRead(PCA_MODE1, &mode1, 1)) {
    if (++pcaReadFails >= 3) {
      servoOk = false;
      Serial.println("Servo2 not answering any more - check SDA/SCL/3.3V/GND. Retrying every 2 s");
    }
    return;
  }
  pcaReadFails = 0;
  if (mode1 & (M1_SLEEP | M1_EXTCLK)) {
    pcaReinits++;
    Serial.printf("Servo2 chip found asleep (MODE1=0x%02X) - re-initialising (#%lu)\n",
                  mode1, (unsigned long)pcaReinits);
    pcaInit();
  }
}

// ============================================================== Gait ========
const char* modeName(Mode m) {
  switch (m) {
    case VERTICAL: return "caterpillar";
    case LATERAL:  return "serpentine";
    case SIDEWIND: return "sidewind";
    default:       return "stop";
  }
}

// SIM: the wave formula now lives in sim_gait.h, so that it can be checked against the
// simulation on a PC. For pitch-yaw-pitch-yaw-pitch the serpentine and sidewind waves are
// exactly what they were; the caterpillar wave gets its direction from CATERPILLAR_WAVE
// and the pitch offset on top.
float gaitAngle(int j) {
  float amp = min(amplitudeDeg, MAX_AMP_DEG) * ramp;
  return simGaitAngle(mode, TYPE[j] == PITCH, j, amp, phaseLagDeg, phase, pitchOffsetDeg * ramp, CATERPILLAR_WAVE);
}

void updateGait(float dt) {
  if (requestedMode != mode) {                            // fade out, switch, fade in
    ramp -= RAMP_PER_SEC * dt;
    if (ramp <= 0.0f) { ramp = 0.0f; mode = requestedMode; phase = 0.0f; }
  } else if (mode != STOP && ramp < 1.0f) {
    ramp = min(ramp + RAMP_PER_SEC * dt, 1.0f);
  }

  phase += travelDir * TWO_PI * frequencyHz * dt;
  if (phase >  TWO_PI) phase -= TWO_PI;
  if (phase < -TWO_PI) phase += TWO_PI;

  float target = (requestedMode == STOP) ? 0.0f : steerTargetDeg;   // straighten when stopping
  float step = STEER_RATE_DPS * dt;
  steerDeg += constrain(target - steerDeg, -step, step);

  // SIM: pitch offset (caterpillar only), changed slowly so the servos do not jump
  float offTarget = (requestedMode == VERTICAL) ? pitchOffsetTargetDeg : 0.0f;
  float offStep = OFFSET_RATE_DPS * dt;
  pitchOffsetDeg += constrain(offTarget - pitchOffsetDeg, -offStep, offStep);

  if (pitchTest && (mode != STOP || requestedMode != STOP || millis() - pitchTestStart >= PITCH_TEST_MS)) {
    pitchTest = false;
    Serial.println("Pitch test done");
  }

  for (int j = 0; j < NUM_JOINTS; j++) {
    float a = pitchTest ? (TYPE[j] == PITCH ? PITCH_TEST_DEG : 0.0f) : gaitAngle(j);
    if (TYPE[j] == YAW) a += STEER_SIGN * steerDeg;
    jointDeg[j] = a;
    if (servoOk) writeJoint(j, a);
  }
}

// ========================================================== UART link =======
// Every message is one text line:   $BODY*CS\n
// CS = XOR of all characters in BODY, as two hex digits (same idea as GPS/NMEA).
// A line with a wrong checksum is thrown away, so noise can never move a servo.
//
//   sensor -> motor:  $S,mode,amp,freq,lag,dir,steer,pitchOffset*CS
//                     (SIM: pitchOffset is new. Without it, it counts as 0.)
//   motor -> sensor:  $J,activeMode,ramp,steer,servoOk,types,a1,a2,...*CS

uint8_t xorSum(const char* s) {
  uint8_t cs = 0;
  while (*s) cs ^= (uint8_t)*s++;
  return cs;
}

void linkSend(const char* body) {
  linkPort.printf("$%s*%02X\n", body, xorSum(body));
}

// Checks "$BODY*CS". On success, cuts the line down to BODY and returns it.
char* checkFrame(char* line) {
  if (line[0] != '$') return nullptr;
  char* star = strrchr(line, '*');
  if (!star || strlen(star) < 3) return nullptr;
  *star = 0;
  uint8_t cs = (uint8_t)strtoul(star + 1, nullptr, 16);
  return (xorSum(line + 1) == cs) ? line + 1 : nullptr;
}

void handleOrder(char* body) {
  // body = "S,mode,amp,freq,lag,dir,steer[,pitchOffset]"
  float v[7];
  int n = 0;
  char* tok = strtok(body, ",");
  if (!tok || strcmp(tok, "S") != 0) { badFrames++; return; }
  while (n < 7 && (tok = strtok(nullptr, ","))) v[n++] = atof(tok);
  if (n < 6) { badFrames++; return; }

  requestedMode  = (Mode)constrain((int)v[0], 0, 3);
  amplitudeDeg   = constrain(v[1], 0.0f, MAX_AMP_DEG);
  frequencyHz    = constrain(v[2], 0.1f, 2.0f);
  phaseLagDeg    = constrain(v[3], 0.0f, 180.0f);
  travelDir      = (v[4] < 0) ? -1 : 1;
  steerTargetDeg = constrain(v[5], -MAX_STEER_DEG, MAX_STEER_DEG);
  pitchOffsetTargetDeg = (n >= 7) ? constrain(v[6], -MAX_PITCH_OFFSET_DEG, MAX_PITCH_OFFSET_DEG) : 0.0f;   // SIM

  if (!linkSeen) Serial.println("Sensor board connected - it is in control now");
  linkSeen = true;
  lastLinkMs = millis();
}

// Prints a rejected line so we can SEE what the corruption looks like.
// Unprintable bytes are shown as <hex>. Max 2 per second, so it can't flood.
void logBadLine(const char* raw, int len) {
  static uint32_t lastLog = 0;
  if (millis() - lastLog < 500) return;
  lastLog = millis();
  Serial.printf("BAD LINE (%d bytes, total bad %lu / good %lu): ", len,
                (unsigned long)badFrames, (unsigned long)goodFrames);
  for (int i = 0; i < len; i++) {
    uint8_t c = raw[i];
    if (c >= 32 && c < 127) Serial.write(c); else Serial.printf("<%02X>", c);
  }
  Serial.println();
}

void linkRead() {
  static char buf[96], raw[96];
  static int len = 0;
  while (linkPort.available()) {
    char c = linkPort.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = 0;
      memcpy(raw, buf, len + 1);                          // checkFrame() cuts buf, keep a copy
      uint32_t before = badFrames;
      char* body = checkFrame(buf);
      if (body) handleOrder(body);
      else if (len > 0) badFrames++;
      if (badFrames != before) logBadLine(raw, len);
      else if (body) goodFrames++;
      len = 0;
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      badFrames++;
      logBadLine(buf, len);
      len = 0;                                            // line too long: drop it
    }
  }
}

void linkReport() {
  char body[128];
  int n = snprintf(body, sizeof(body), "J,%d,%.2f,%.1f,%d,", mode, ramp, steerDeg, servoOk);
  for (int j = 0; j < NUM_JOINTS; j++) n += snprintf(body + n, sizeof(body) - n, "%c", TYPE[j] == PITCH ? 'P' : 'Y');
  for (int j = 0; j < NUM_JOINTS; j++) n += snprintf(body + n, sizeof(body) - n, ",%.1f", jointDeg[j]);
  linkSend(body);
}

// ========================================================= USB serial =======
void printStatus() {
  Serial.printf("Mode %s (active %s) | amp %.0f | %.1f Hz | lag %.0f | %s | steer %.1f | pitch offset %.1f\n",
                modeName(requestedMode), modeName(mode), amplitudeDeg, frequencyHz, phaseLagDeg,
                travelDir > 0 ? "forward" : "backward", steerDeg, pitchOffsetDeg);
  Serial.printf("Servo2 %s | link %s | good frames %lu | bad frames %lu\n", servoOk ? "OK" : "MISSING",
                !linkSeen ? "never seen" : (millis() - lastLinkMs < LINK_TIMEOUT_MS ? "OK" : "LOST"),
                (unsigned long)goodFrames, (unsigned long)badFrames);
}

void handleUsb() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '0': requestedMode = STOP;     break;
      case '1': requestedMode = VERTICAL; break;
      case '2': requestedMode = LATERAL;  break;
      case '3': requestedMode = SIDEWIND; break;
      case '+': amplitudeDeg = min(amplitudeDeg + 5.0f, MAX_AMP_DEG); break;
      case '-': amplitudeDeg = max(amplitudeDeg - 5.0f, 0.0f);         break;
      case 'f': frequencyHz  = min(frequencyHz + 0.1f, 2.0f);          break;
      case 's': frequencyHz  = max(frequencyHz - 0.1f, 0.1f);          break;
      case 'r': travelDir = -travelDir; break;
      case '?': break;
      case 'd': printServoDiagnostics(); continue;
      case 'p':                                          // SIM: check the sign of the pitch joints
        if (mode != STOP || requestedMode != STOP) { Serial.println("Pitch test: stop the snake first (0)"); continue; }
        pitchTest = true;
        pitchTestStart = millis();
        Serial.printf("Pitch test: every pitch joint at %.0f deg for %lu s. Both ENDS of the snake must go DOWN\n"
                      "(hilltop shape). A pitch joint that lifts its end instead: give it DIRECTION -1.\n",
                      PITCH_TEST_DEG, (unsigned long)(PITCH_TEST_MS / 1000));
        continue;
      default: continue;
    }
    if (linkSeen && c != '?') Serial.println("(note: the sensor board will overwrite this)");
    printStatus();
  }
}

// ===================================================== Setup / loop ========
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nSnakeBot MOTOR board starting");
  pinMode(LED_PIN, OUTPUT);
  for (int j = 0; j < NUM_JOINTS; j++) jointDeg[j] = 0;

  // Safety check: a joint must never be put on a reserved (power-only) port
  for (int j = 0; j < NUM_JOINTS; j++)
    for (uint8_t ch : RESERVED_PORTS)
      if (PORT[j] == ch) {
        Serial.printf("CONFIG ERROR: joint %d is on reserved port %d. Fix PORT[] - servos disabled.\n", j + 1, ch);
        while (true) delay(1000);                      // refuse to run
      }

  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  Wire.setTimeOut(10);
  probeServo2();
  if (!servoOk) Serial.println("Servo2 NOT found at 0x40 - check SDA/SCL, 3.3V, GND. Retrying every 2 s");

  linkPort.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);
  Serial.println("Waiting for the sensor board on UART2... (USB commands: 0-3, +/-, f/s, r, ?, d, p)");
  lastControl = millis();
}

void loop() {
  linkRead();
  handleUsb();

  uint32_t now = millis();

  // Failsafe: orders stopped arriving -> fade out and stop
  if (linkSeen && requestedMode != STOP && now - lastLinkMs > LINK_TIMEOUT_MS) {
    requestedMode = STOP;
    Serial.println("Link to sensor board LOST - stopping");
  }

  if (now - lastControl >= CONTROL_MS) {
    float dt = (now - lastControl) / 1000.0f;
    lastControl = now;
    updateGait(dt);
    digitalWrite(LED_PIN, mode != STOP);
  }
  if (now - lastReport >= REPORT_MS) {
    lastReport = now;
    linkReport();
  }
  if (!servoOk && now - lastProbe >= 2000) {
    lastProbe = now;
    probeServo2();
  }
  if (servoOk && now - lastWatchdog >= 1000) {
    lastWatchdog = now;
    pcaWatchdog();
  }
}
