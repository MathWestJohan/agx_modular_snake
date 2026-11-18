/*
  ============================================================================
   SnakeBot - SENSOR ESP32   (sensors, decisions and the Wi-Fi dashboard)
  ============================================================================
   Board : ESP32-DEV-38P ("ESP32 Dev Module"), Arduino core 2.x or 3.x
   Parts : 2 x MC6470 accelerometer (front + rear)
           2 x Sharp GP2Y0A21YK0F IR distance (front left + front right)
           4 x Ohmite FSR06CE force sensors (underneath) - NOT USED now, see NUM_FSR
           M5Stack Timer Camera X (line following, over a serial wire)
   Partner sketches: snake_motor.ino (runs the Servo2)
                     snake_camera_line.ino (on the camera, USE_WIRE = true)

   Wi-Fi : join "Beanie" (password beanie0987) on your phone, then open
           http://192.168.4.1   - live data + controls.
           (Optional: the camera can join this network as 192.168.4.50.)

   This board decides WHAT the snake should do (mode, speed, steering,
   obstacle stop, line following) and sends it to the motor board 20 times
   per second over a UART wire. The motor board answers with the joint
   angles, which are shown on the phone.

   Libraries: "WebSockets" (Markus Sattler)

  ---------------------------------------------------------------------------
   WIRING  (all grounds connected together - including the Servo2 GND!)
  ---------------------------------------------------------------------------
   UART link to the motor ESP32 (cross TX and RX!):
     sensor GPIO17 (TX) -> motor GPIO16 (RX)
     sensor GPIO16 (RX) <- motor GPIO17 (TX)
     sensor GND         -- motor GND

   Camera wire (UART1, one direction: camera -> this board):
     camera Grove G4  -> sensor GPIO26
     camera Grove GND -> sensor GND      <-- REQUIRED, even with the camera on battery
     (GPIO27 is reserved as the unused TX pin - leave it free)

   Front IMU (I2C bus A):  GPIO21 SDA, GPIO22 SCL, VCC 3.3V, GND
   Rear IMU  (I2C bus B):  GPIO18 SDA, GPIO19 SCL, VCC 3.3V, GND
     long cable: twist SDA and SCL each with a GND wire.
     If the boot scan can't find it, add 2.2k pull-ups to 3.3V on bus B.
     The IMUs may be mounted any way up (flat, upright, ...). Calibrate once:
     snake flat + straight + still, STOP, press "Calibrate IMUs" (or USB 'c').
     Saved in flash - only redo it if an IMU is moved.

   Sharp IR (power from a FREE Servo2 port, e.g. 12 and 13 - 5V + GND only,
   leave the signal pin of that port unconnected):
     left  Vo -> GPIO34
     right Vo -> GPIO35
     100 uF + 100 nF across 5V/GND at each sensor (the servo rail is noisy)
     Servo2 GND MUST be connected to this board's GND, or the readings are garbage.

   FSR (underneath), each one as a voltage divider:
       3.3V --- FSR ---+--- GPIOxx
                       |
                    10 kOhm (+ optional 100 nF in parallel)
                       |
                      GND
     FSR1 -> GPIO32   FSR2 -> GPIO33   FSR3 -> GPIO36 (VP)   FSR4 -> GPIO39 (VN)

   All six analog inputs are on ADC1 (GPIO32-39): ADC2 stops working with Wi-Fi.

  ---------------------------------------------------------------------------
   LINE FOLLOWING (camera)
  ---------------------------------------------------------------------------
   The camera sends "$L,seen,steer,near,far,nearSeen,farSeen*CS" 20 times a
   second over the wire (GPIO26). It is also accepted by UDP on port 4210 if
   the camera is on Wi-Fi (backup). steer = -1 (line far left) .. +1 (far right).
   Tick "Follow line" on the dashboard (or USB 'l'), then press Caterpillar.
   Caterpillar is best: the head points forward instead of swinging.
   Line not seen: the snake keeps going. It holds its last steering for
   LINE_HOLD_MS (the line often just slips out of view in a curve), then
   straightens out and goes straight ahead until it sees the line again.
   Safety: it STOPS if the camera stops sending for CAM_TIMEOUT_MS (set
   STOP_IF_CAMERA_LOST = false to drive straight on instead).
   "Stop at obstacle" still works.
   Check: line to the RIGHT of the camera -> snake must turn RIGHT.
          If it turns away from the line, set LINE_SIGN = -1.

  ---------------------------------------------------------------------------
   USB Serial Monitor (115200): 0-3 mode | +/- amp | f/s speed | r reverse
                                l line follow on/off | c calibrate IMUs | ? status
                                g sim gait on/off

  ---------------------------------------------------------------------------
   SIM GAIT (gaits from the simulation) - every change is marked "SIM"
  ---------------------------------------------------------------------------
   This is esp32_code/snake_sensors.ino plus what the simulation needs.
   Tick "Sim gait" on the dashboard (or USB 'g'), then press Caterpillar:
     - Amplitude, phase lag and pitch offset come from the simulation's gait
       table (sim_gaits.h, made by firmware/sync_from_sim.py) instead of the
       sliders. Speed (Hz) stays on the slider. In the simulation this robot
       managed about 0.35 Hz.
     - The gait is chosen from the slope: flat, uphill, downhill, crest
       (hilltop: bigger wave + the pitch joints bent down) and valley.
       The slope is worked out from both IMUs and the joint angles the motor
       board reports. For that the IMUs must be calibrated, and the pitch
       joints must have the right sign (motor board, 'p' command).
     - With "Auto-steer (IR)" on and flat ground it uses the simulation's maze
       gait (a smaller wave, so the snake does not roll over between walls).
   With "Sim gait" off, everything works as before.
   Use snake_motor.ino from this folder with it: the old motor sketch ignores
   the pitch offset and runs the caterpillar wave the other way.

  ---------------------------------------------------------------------------
   UNTETHERED USE (no phone, no computer)
  ---------------------------------------------------------------------------
   Switch on, put the snake down: after 5 s it starts in caterpillar mode.
   (With START_LINE_FOLLOW = true it waits until the camera is connected.)
   BOOT button on this ESP32:  press = next mode
                               (stop > caterpillar > serpentine > sidewind > stop)
                               hold 1 s = STOP
                               press during the 5 s countdown = cancel auto-start
   Do NOT hold BOOT while switching on (that starts the board in upload mode).

   Blue LED:  short blip twice a second   counting down to auto-start
              fast flicker                motor board not answering
              steady on                   link OK, stopped
              1 / 2 / 3 blinks, pause     caterpillar / serpentine / sidewind
   Settings: see "Untethered control" in CONFIGURE ME below.
  ============================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Preferences.h>
#include "sim_gaits.h"      // SIM: gait tables copied from the simulation (firmware/sync_from_sim.py)
#include "sim_terrain.h"    // SIM: body slope from the IMUs + joint angles, and the gait choice

// ============================ CONFIGURE ME ==================================
// ---- Wi-Fi ----
const char* AP_SSID = "Beanie";
const char* AP_PASS = "beanie0987";           // at least 8 characters

// ---- Pins ----
const int BUS_A_SDA = 21, BUS_A_SCL = 22;     // front IMU
const int BUS_B_SDA = 18, BUS_B_SCL = 19;     // rear IMU (long cable)
const uint32_t I2C_HZ = 100000;
const int LINK_RX = 16, LINK_TX = 17;         // UART2 to the motor ESP32
const uint32_t LINK_BAUD = 38400;             // MUST match snake_motor.ino. Slower = more noise-proof
const uint32_t LINK_TIMEOUT_MS = 1000;        // no report for this long = link lost

const int IR_LEFT_PIN  = 34;
const int IR_RIGHT_PIN = 35;
const int NUM_FSR = 0;                        // 0 = FSRs not used (not read, not on the dashboard)
const int FSR_PINS[4] = {32, 33, 36, 39};
const int LED_PIN = 2;                        // on-board blue LED: status (see updateLed)

// ---- IMUs (MC6470 accelerometer part) ----
const uint8_t IMU_ADDR    = 0x4C;             // both IMUs use this, each on its own bus
const uint8_t IMU_RANGE_G = 2;                // 2, 4, 8 or 16
const uint8_t IMU_RATE    = 0x08;             // 0x08 = 64 Hz
// Mounting: which sensor axis points (roughly) towards the HEAD?
// {1,0,0} = +X, {-1,0,0} = -X, {0,1,0} = +Y, {0,0,1} = +Z, ...
// "Up" does NOT need setting: the calibration measures it.
// Check after calibrating: lift the head -> pitch must go POSITIVE.
//                          If it goes negative, flip the sign here.
const float FRONT_FWD[3] = {1, 0, 0};
const float REAR_FWD[3]  = {1, 0, 0};
const float TILT_ALPHA = 0.15;                // smoothing (lower = smoother)
const int   CAL_SAMPLES      = 50;            // 1 s at 50 Hz
const float CAL_MAX_WOBBLE_G = 0.04;          // more shaking than this = "moved", try again

// ---- Sharp IR ----
const float IR_MIN_CM = 10.0, IR_MAX_CM = 80.0;
const float IR_ALPHA  = 0.3;

// ---- FSR ----
const float FSR_CONTACT_MV = 400.0;           // above this = touching the ground
const float FSR_ALPHA      = 0.5;

// ---- Decisions ----
const float MAX_AMP_DEG      = 45.0;
const float MAX_STEER_DEG    = 30.0;
const float AUTO_KP          = 1.0;           // IR auto-steer: deg per cm left/right difference
const float OBSTACLE_STOP_CM = 15.0;

// ---- Camera line following ----
const int      CAM_RX_PIN     = 26;           // <- camera Grove G4
const int      CAM_TX_PIN     = 27;           // not connected (the UART driver wants a pin)
const uint32_t CAM_BAUD       = 115200;       // MUST match WIRE_BAUD in snake_camera_line.ino
const uint16_t LINE_UDP_PORT  = 4210;         // Wi-Fi backup, MUST match snake_camera_line.ino
const float    LINE_KP        = 50.0;         // deg of steering when the line is at the picture edge
const float    LINE_SIGN      = 1.0;          // -1 if the snake turns AWAY from the line
const float    LINE_SMOOTH    = 0.3;          // 0..1: lower = calmer steering, higher = quicker
const uint32_t LINE_HOLD_MS   = 300;          // line lost: keep the last steering this long ...
const float    LINE_RETURN    = 0.1;          // ... then ease back to straight (0..1, higher = faster)
const uint32_t CAM_TIMEOUT_MS = 500;          // no camera message this long = camera lost
const bool     STOP_IF_CAMERA_LOST = true;    // camera lost: true = STOP, false = keep going straight
const char*    CAMERA_URL     = "http://192.168.4.50";   // camera's own page (for the dashboard link)

// ---- SIM: gaits from the simulation ----
// The front IMU must sit in the head (in front of joint 1) and the rear IMU in the tail
// (behind the last joint): the slope is worked out from there through the joint angles.
const bool     START_SIM_GAIT   = false;      // true = "Sim gait" already on at power-up (course run!)
const bool     SIM_CREST_VALLEY = true;       // false = only flat / uphill / downhill
const float    SIM_BEND_TAU_S   = 20.0;       // crest/valley: the slope is compared with its own average
                                              // over about this many seconds (not tested in the simulation yet)
const bool     SIM_MAZE_WITH_IR = true;       // IR auto-steer on + flat ground -> the simulation's maze gait
const uint32_t TERRAIN_MS       = 250;        // gait choice 4 times a second, as in the simulation
const float    SIM_BIG_WAVE_STEER_DEG = 10.0; // steering limit in the crest / valley gaits: full lock on
                                              // top of their big wave rolled the simulated snake over

// ---- Timing ----
const uint32_t IMU_MS       = 20;             // 50 Hz
const uint32_t SENSOR_MS    = 40;             // 25 Hz IR + FSR
const uint32_t ORDER_MS     = 50;             // 20 Hz orders to the motor board
const uint32_t TELEMETRY_MS = 50;             // 20 Hz to the phone
const uint32_t REPROBE_MS   = 2000;

// ---- Untethered control: auto-start + BOOT button ----
const bool     AUTO_START          = true;    // false = always wait in STOP after power-on
const int      AUTO_START_MODE     = 1;       // 1 caterpillar, 2 serpentine, 3 sidewind
const uint32_t AUTO_START_DELAY_MS = 5000;    // time to put the snake down after switching on
const bool     START_OBSTACLE_STOP = false;   // true = "stop at obstacle" already on at power-up
const bool     START_AUTO_STEER    = false;   // true = IR auto-steer already on at power-up
const bool     START_LINE_FOLLOW   = false;   // true = line following on at power-up (course run!)
const int      MODE_BUTTON_PIN     = 0;       // the BOOT button on the ESP32 board (GPIO0)
const uint32_t LONG_PRESS_MS       = 1000;    // hold this long = STOP
// ============================================================================

WebServer        server(80);
WebSocketsServer webSocket(81);
HardwareSerial&  linkPort = Serial2;         // motor board
HardwareSerial&  camPort  = Serial1;         // camera wire
WiFiUDP          camUdp;
Preferences      prefs;                      // flash storage (IMU calibration)

// ---------------------------------------------------------------- orders
// What we WANT the snake to do. Sent to the motor board.
enum Mode { STOP, VERTICAL, LATERAL, SIDEWIND };
Mode  requestedMode = STOP;
float amplitudeDeg = 20.0;
float frequencyHz  = 0.5;
float phaseLagDeg  = 120.0;
int   travelDir    = 1;
float manualSteerDeg = 0.0;
bool  autoSteer    = false;
bool  obstacleStop = false;
bool  lineFollow   = false;
bool  simGait      = false;                   // SIM: amplitude, lag and pitch offset from the simulation

// ---------------------------------------------------------------- motor report
// What the motor board says it is ACTUALLY doing.
const int MAX_JOINTS = 8;
int   motorMode = 0;
float motorRamp = 0, motorSteer = 0;
bool  motorServoOk = false;
int   numJoints = 0;
char  jointTypes[MAX_JOINTS + 1] = "";
float jointDeg[MAX_JOINTS];
uint32_t lastReportMs = 0, badFrames = 0;
bool  everHeardMotor = false;

// ---------------------------------------------------------------- camera
bool     camEverHeard = false;
uint32_t lastCamMs = 0, lastLineSeenMs = 0, camPackets = 0, camBadPackets = 0;
bool     camSeen = false;                     // camera sees the line right now
float    camSteer = 0;                        // -1 .. +1 from the camera
float    lineSteerDeg = 0;                    // smoothed steering we ask for
bool     camViaWire = false;                  // last good camera message came by wire (else Wi-Fi)

// ---------------------------------------------------------------- SIM gait
const SimGaitTable* simTable = nullptr;       // gait table for the joint layout the motor board reports
int      simSituation = SIT_FLAT;             // the gait in use
simterrain::Terrain terrain;                  // filtered slope and bend, degrees
bool     terrainOk = false;                   // the slope is being measured
float    slopeSum = 0;                        // slope samples since the last gait choice
int      slopeN = 0;
uint32_t lastTerrain = 0, simSwitchedMs = 0;

// ---------------------------------------------------------------- sensors
float irLeftCm = NAN, irRightCm = NAN;
float fsrMv[4] = {0, 0, 0, 0};

char lastMsg[64] = "";
uint32_t lastImu = 0, lastSensor = 0, lastOrder = 0, lastTelemetry = 0, lastProbe = 0, maxLoopUs = 0;

bool     autoStartPending = AUTO_START;       // counting down to auto-start
uint32_t readyMs = 0;                         // when setup() finished

// ================================================================= IMU ======
// NOTE: keep all types (struct/enum) ABOVE the first function in this file.
// The Arduino IDE auto-inserts function declarations before the first
// function, and they must already know about types like Imu.
struct Imu {
  const char*  name;
  char         key;              // 'f' or 'r': name of its calibration in flash
  TwoWire*     bus;
  const float* fwdHint;          // sensor axis that points towards the head
  bool  ok;
  int   fails;
  float pitch, roll;             // degrees, NAN until first reading
  // Body directions in sensor coordinates (set by the calibration):
  float fwd[3], left[3], up[3];
  bool  calibrated;
  // Calibration in progress:
  int   calN;
  float calSum[3], calSq[3];
};
Imu frontImu = {"front IMU", 'f', &Wire,  FRONT_FWD};
Imu rearImu  = {"rear IMU",  'r', &Wire1, REAR_FWD};

bool     calRunning = false;
uint32_t calStartMs = 0;

const uint8_t REG_MODE = 0x07, REG_SRTFR = 0x08, REG_XOUT_L = 0x0D, REG_OUTCFG = 0x20;

bool i2cPresent(TwoWire* bus, uint8_t addr) {
  bus->beginTransmission(addr);
  return bus->endTransmission() == 0;
}

void imuWrite(Imu& m, uint8_t reg, uint8_t val) {
  m.bus->beginTransmission(IMU_ADDR);
  m.bus->write(reg);
  m.bus->write(val);
  m.bus->endTransmission();
}

bool imuRead(Imu& m, uint8_t reg, uint8_t* buf, uint8_t len) {
  m.bus->beginTransmission(IMU_ADDR);
  m.bus->write(reg);
  if (m.bus->endTransmission(false) != 0) return false;
  if (m.bus->requestFrom(IMU_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = m.bus->read();
  return true;
}

bool imuInit(Imu& m) {
  if (!i2cPresent(m.bus, IMU_ADDR)) return false;
  uint8_t rangeBits = (IMU_RANGE_G == 2) ? 0 : (IMU_RANGE_G == 4) ? 1 : (IMU_RANGE_G == 8) ? 2 : 3;
  uint8_t cfg = (rangeBits << 4) | 0x05;                  // 0x05 = 14-bit resolution
  imuWrite(m, REG_MODE, 0x00);                            // standby to configure
  imuWrite(m, REG_OUTCFG, cfg);
  imuWrite(m, REG_SRTFR, IMU_RATE);
  imuWrite(m, REG_MODE, 0x01);                            // wake
  uint8_t back = 0;
  return imuRead(m, REG_OUTCFG, &back, 1) && back == cfg;
}

// ---- small vector helpers ----
float dot3(const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
float norm3(const float* a) { return sqrtf(dot3(a, a)); }

// Work out forward/left/up (in sensor coordinates) from a measured "up" and
// the forward hint. Returns false if the hint points (almost) straight up.
bool imuSetFrame(Imu& m, const float* upMeasured) {
  float n = norm3(upMeasured);
  if (n < 0.1f) return false;
  for (int i = 0; i < 3; i++) m.up[i] = upMeasured[i] / n;
  float d = dot3(m.fwdHint, m.up);                        // remove the "up" part of the hint
  for (int i = 0; i < 3; i++) m.fwd[i] = m.fwdHint[i] - d * m.up[i];
  n = norm3(m.fwd);
  if (n < 0.3f) return false;
  for (int i = 0; i < 3; i++) m.fwd[i] /= n;
  m.left[0] = m.up[1] * m.fwd[2] - m.up[2] * m.fwd[1];    // left = up x forward
  m.left[1] = m.up[2] * m.fwd[0] - m.up[0] * m.fwd[2];
  m.left[2] = m.up[0] * m.fwd[1] - m.up[1] * m.fwd[0];
  return true;
}

// At start-up: use the saved calibration, or guess "Z is up" until calibrated
void imuLoadCal(Imu& m) {
  char k[4];
  float up[3] = {0, 0, 1};
  m.pitch = m.roll = NAN;                                 // "no reading yet"
  snprintf(k, sizeof(k), "%cc", m.key);
  m.calibrated = prefs.getBool(k, false);
  if (m.calibrated)
    for (int i = 0; i < 3; i++) { snprintf(k, sizeof(k), "%c%d", m.key, i); up[i] = prefs.getFloat(k, up[i]); }
  if (!imuSetFrame(m, up)) {                              // hint was along Z: guess differently
    float alt[3] = {0, 1, 0};
    imuSetFrame(m, alt);
    m.calibrated = false;
  }
  Serial.printf("%s: %s\n", m.name, m.calibrated ? "calibration loaded" : "NOT calibrated yet");
}

void imuUpdate(Imu& m) {
  if (!m.ok) return;
  uint8_t b[6];
  if (!imuRead(m, REG_XOUT_L, b, 6)) {
    if (++m.fails >= 5) {
      m.ok = false;
      m.pitch = m.roll = NAN;
      snprintf(lastMsg, sizeof(lastMsg), "%s lost - check cable", m.name);
      Serial.println(lastMsg);
    }
    return;
  }
  m.fails = 0;
  float k = 8192.0f / IMU_RANGE_G;                        // counts per g, 14-bit
  float a[3] = {(int16_t)(b[1] << 8 | b[0]) / k,          // raw sensor axes
                (int16_t)(b[3] << 8 | b[2]) / k,
                (int16_t)(b[5] << 8 | b[4]) / k};

  if (calRunning && m.calN < CAL_SAMPLES) {               // collecting for calibration
    for (int i = 0; i < 3; i++) { m.calSum[i] += a[i]; m.calSq[i] += a[i] * a[i]; }
    m.calN++;
  }

  // Turn sensor axes into snake axes (forward, left, up)
  float ax = dot3(a, m.fwd), ay = dot3(a, m.left), az = dot3(a, m.up);

  // Tilt from gravity (no gyro, so no heading). Smoothed, because the
  // servos shake the body.
  float p = degrees(atan2f(ax, sqrtf(ay * ay + az * az)));   // + = nose up
  float r = degrees(atan2f(ay, az));                          // + = rolled right
  if (isnan(m.pitch)) { m.pitch = p; m.roll = r; }
  else { m.pitch += TILT_ALPHA * (p - m.pitch); m.roll += TILT_ALPHA * (r - m.roll); }
}

// ---- Calibration: snake flat + straight + still, ~1 second ----
void startImuCal() {
  if (calRunning) return;
  if (requestedMode != STOP || motorRamp > 0.01f) {
    snprintf(lastMsg, sizeof(lastMsg), "Press STOP first (snake must be straight + still)");
    Serial.println(lastMsg);
    return;
  }
  if (!frontImu.ok && !rearImu.ok) {
    snprintf(lastMsg, sizeof(lastMsg), "No IMU found - nothing to calibrate");
    Serial.println(lastMsg);
    return;
  }
  Imu* all[2] = {&frontImu, &rearImu};
  for (Imu* m : all) { m->calN = 0; for (int i = 0; i < 3; i++) m->calSum[i] = m->calSq[i] = 0; }
  calRunning = true;
  calStartMs = millis();
  snprintf(lastMsg, sizeof(lastMsg), "Calibrating IMUs - keep the snake flat and still...");
  Serial.println(lastMsg);
}

// Returns a short result text
const char* imuFinishCal(Imu& m) {
  if (!m.ok) return "missing";
  if (m.calN < CAL_SAMPLES) return "no data";
  float mean[3], maxSd = 0;
  for (int i = 0; i < 3; i++) {
    mean[i] = m.calSum[i] / m.calN;
    float var = m.calSq[i] / m.calN - mean[i] * mean[i];
    maxSd = max(maxSd, sqrtf(max(var, 0.0f)));
  }
  float g = norm3(mean);
  if (maxSd > CAL_MAX_WOBBLE_G) return "moved, try again";
  if (g < 0.85f || g > 1.15f)   return "odd reading";
  if (!imuSetFrame(m, mean))    return "FWD axis points up";
  char k[4];
  for (int i = 0; i < 3; i++) { snprintf(k, sizeof(k), "%c%d", m.key, i); prefs.putFloat(k, mean[i]); }
  snprintf(k, sizeof(k), "%cc", m.key);
  prefs.putBool(k, true);
  m.calibrated = true;
  m.pitch = m.roll = NAN;                                 // restart smoothing from the new zero
  return "OK";
}

void updateImuCal() {
  if (!calRunning) return;
  bool frontDone = !frontImu.ok || frontImu.calN >= CAL_SAMPLES;
  bool rearDone  = !rearImu.ok  || rearImu.calN  >= CAL_SAMPLES;
  bool timeout   = millis() - calStartMs > 3000;
  if (!(frontDone && rearDone) && !timeout) return;
  calRunning = false;
  snprintf(lastMsg, sizeof(lastMsg), "IMU calibration: front %s, rear %s",
           imuFinishCal(frontImu), imuFinishCal(rearImu));
  Serial.println(lastMsg);
}

// =========================================================== Analog ========
float readMv(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogReadMilliVolts(pin);
  return sum / 8.0f;
}

float voltsToCm(float v) {                // GP2Y0A21 curve from sharp_ir_test.ino
  if (v < 0.15f) return NAN;
  return 27.728f * powf(v, -1.2045f);
}

float smoothNan(float prev, float now, float alpha) {
  if (isnan(now))  return NAN;
  if (isnan(prev)) return now;
  return prev + alpha * (now - prev);
}

void stopWith(const char* why) {
  requestedMode = STOP;
  snprintf(lastMsg, sizeof(lastMsg), "%s", why);
  Serial.println(lastMsg);
}

void readAnalogSensors() {
  irLeftCm  = smoothNan(irLeftCm,  voltsToCm(readMv(IR_LEFT_PIN)  / 1000.0f), IR_ALPHA);
  irRightCm = smoothNan(irRightCm, voltsToCm(readMv(IR_RIGHT_PIN) / 1000.0f), IR_ALPHA);
  for (int i = 0; i < NUM_FSR; i++) fsrMv[i] += FSR_ALPHA * (readMv(FSR_PINS[i]) - fsrMv[i]);

  if (obstacleStop && requestedMode != STOP && travelDir > 0) {
    float l = isnan(irLeftCm) ? 999 : irLeftCm, r = isnan(irRightCm) ? 999 : irRightCm;
    if (min(l, r) < OBSTACLE_STOP_CM) {
      char m[64];
      snprintf(m, sizeof(m), "Obstacle at %.0f cm - stopped", min(l, r));
      stopWith(m);
    }
  }
}

// ======================================================= Camera (UDP) =======
// The camera talks the same "$BODY*CS" format as the UART link (see below).
uint8_t xorSum(const char* s) {
  uint8_t cs = 0;
  while (*s) cs ^= (uint8_t)*s++;
  return cs;
}

char* checkFrame(char* line) {
  if (line[0] != '$') return nullptr;
  char* star = strrchr(line, '*');
  if (!star || strlen(star) < 3) return nullptr;
  *star = 0;
  uint8_t cs = (uint8_t)strtoul(star + 1, nullptr, 16);
  return (xorSum(line + 1) == cs) ? line + 1 : nullptr;
}

bool camOk() { return camEverHeard && millis() - lastCamMs < CAM_TIMEOUT_MS; }

void handleCamera(char* body) {
  // body = "L,seen,steer,near,far,nearSeen,farSeen"
  char* tok = strtok(body, ",");
  if (!tok || strcmp(tok, "L") != 0) { camBadPackets++; return; }
  float v[6];
  int n = 0;
  while (n < 6 && (tok = strtok(nullptr, ","))) v[n++] = atof(tok);
  if (n != 6) { camBadPackets++; return; }

  uint32_t now = millis();
  camSeen  = v[0] > 0.5f;
  camSteer = constrain(v[1], -1.0f, 1.0f);
  if (camSeen) {
    lastLineSeenMs = now;
    float target = constrain(LINE_SIGN * LINE_KP * camSteer, -MAX_STEER_DEG, MAX_STEER_DEG);
    lineSteerDeg += LINE_SMOOTH * (target - lineSteerDeg);
  } else if (now - lastLineSeenMs > LINE_HOLD_MS) {
    // Line gone for a while: straighten out gently and keep going forward
    lineSteerDeg += LINE_RETURN * (0.0f - lineSteerDeg);
  }
  // (Line just lost: keep the last steering - in a curve the line often
  //  slips out of view for a moment.)

  if (!camEverHeard) Serial.printf("Camera connected - line data arriving by %s\n", camViaWire ? "wire" : "Wi-Fi");
  camEverHeard = true;
  lastCamMs = now;
  camPackets++;
}

void readCamera() {
  // 1. The wire (main way)
  static char line[96];
  static int len = 0;
  while (camPort.available()) {
    char c = camPort.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = 0;
      char* body = checkFrame(line);
      if (body) { camViaWire = true; handleCamera(body); }
      else if (len > 0) camBadPackets++;
      len = 0;
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = c;
    } else {
      len = 0;                                // too long = garbage, start over
      camBadPackets++;
    }
  }
  // 2. Wi-Fi (backup, only if the camera has joined this network)
  while (camUdp.parsePacket() > 0) {
    char buf[96];
    int n = camUdp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = 0;
    char* nl = strchr(buf, '\n');
    if (nl) *nl = 0;
    char* body = checkFrame(buf);
    if (body) { camViaWire = false; handleCamera(body); }
    else camBadPackets++;
  }
}

// Safety while following the line: stop if the camera is gone.
// (A missing LINE is fine - we just keep going straight.)
void checkLineFollow() {
  if (!lineFollow || requestedMode == STOP || !STOP_IF_CAMERA_LOST) return;
  if (!camOk()) stopWith("Camera not sending - stopped");
}

// Steering to ask for: camera line, IR auto-steer, or the phone slider
float steerTarget() {
  if (lineFollow) {
    if (travelDir < 0 || !camOk()) return 0;     // line following only makes sense going forward
    return lineSteerDeg;
  }
  if (!autoSteer) return manualSteerDeg;
  float l = isnan(irLeftCm)  ? IR_MAX_CM : constrain(irLeftCm,  IR_MIN_CM, IR_MAX_CM);
  float r = isnan(irRightCm) ? IR_MAX_CM : constrain(irRightCm, IR_MIN_CM, IR_MAX_CM);
  return constrain(AUTO_KP * (r - l), -MAX_STEER_DEG, MAX_STEER_DEG);   // + = right side more open
}

// ============================================================ SIM gait ======
// The simulation has one gait table per joint layout. A layout with mostly pitch joints
// (ours: pitch-yaw-pitch-yaw-pitch) crawls with the pitch wave = caterpillar mode; a layout
// with mostly yaw joints uses the yaw wave = serpentine mode.
int simMode() {
  if (!simTable) return VERTICAL;
  return simTable->gaits[SIT_FLAT].amP != 0.0f ? VERTICAL : LATERAL;
}

// Sim gait is ticked AND the snake is in the mode the simulation's gaits are for
bool simActive() { return simGait && simTable != nullptr && requestedMode == simMode(); }

const SimGait* simNow() { return &simTable->gaits[simSituation]; }

// What goes to the motor board: from the simulation's table, or from the sliders
float orderAmp() {
  if (!simActive()) return amplitudeDeg;
  return fabsf(simMode() == VERTICAL ? simNow()->amP : simNow()->amY);
}
float orderLag() {
  if (!simActive()) return phaseLagDeg;
  return fabsf(simMode() == VERTICAL ? simNow()->phiP : simNow()->phiY);
}
float orderPitchOffset() { return simActive() ? simNow()->offsetP : 0.0f; }
float orderSteer() {
  float s = steerTarget();
  if (simActive() && (simSituation == SIT_CREST || simSituation == SIT_VALLEY))
    s = constrain(s, -SIM_BIG_WAVE_STEER_DEG, SIM_BIG_WAVE_STEER_DEG);
  return s;
}

// Which way is up, seen from the body part that carries this IMU (x forward, y left, z up).
// Needs a calibrated IMU: otherwise "level" is only a guess.
bool imuUp(const Imu& m, float up[3]) {
  if (!m.ok || !m.calibrated || isnan(m.pitch) || isnan(m.roll)) return false;
  float p = radians(m.pitch), r = radians(m.roll);
  up[0] = sinf(p);
  up[1] = cosf(p) * sinf(r);
  up[2] = cosf(p) * cosf(r);
  return true;
}

// Called at the IMU rate. The simulation's slope is the line from the last module to the
// first. Each IMU only knows the tilt of its own end, and the ends rock with the wave, so
// the line is worked out from each IMU through the joint angles. Two IMUs = two answers,
// which are averaged.
void sampleSlope() {
  if (!linkOk() || numJoints == 0) return;
  float up[3], one, sum = 0;
  int n = 0;
  if (imuUp(frontImu, up) && simterrain::bodySlopeDeg(jointTypes, jointDeg, numJoints, up, 0, travelDir, one)) { sum += one; n++; }
  if (imuUp(rearImu, up) && simterrain::bodySlopeDeg(jointTypes, jointDeg, numJoints, up, numJoints, travelDir, one)) { sum += one; n++; }
  if (n) { slopeSum += sum / n; slopeN++; }
}

// Called every TERRAIN_MS: filter the slope and choose the gait, like GaitController.pre()
void updateTerrain(float dt) {
  terrainOk = slopeN > 0;
  if (terrainOk) terrain.update(slopeSum / slopeN, dt, SIM_BEND_TAU_S);
  else terrain.reset();
  slopeSum = 0;
  slopeN = 0;

  int wanted = SIT_FLAT;                                  // also when "Sim gait" is off
  if (simActive()) {
    bool crestValley = SIM_CREST_VALLEY && simTable->gaits[SIT_CREST].valid && simTable->gaits[SIT_VALLEY].valid;
    if (terrainOk) wanted = terrain.choose(simSituation, crestValley);
    if (wanted == SIT_FLAT && SIM_MAZE_WITH_IR && autoSteer) wanted = SIT_MAZE;
    if (!simTable->gaits[wanted].valid) wanted = SIT_FLAT;
  }
  if (wanted == simSituation) return;
  if (simActive() && millis() - simSwitchedMs < SIM_MIN_DWELL_MS) return;   // keep a gait for a while
  simSituation = wanted;
  simSwitchedMs = millis();
  if (simActive())
    Serial.printf("Sim gait: %s (slope %+.1f, bend %+.1f)\n", SIM_SITUATION_NAMES[simSituation],
                  terrainOk ? terrain.slope : NAN, terrainOk ? terrain.bend : NAN);
}

// ========================================================== UART link =======
// Same format as in snake_motor.ino:   $BODY*CS\n   (CS = XOR of BODY, hex)
//   sensor -> motor:  $S,mode,amp,freq,lag,dir,steer,pitchOffset*CS      (SIM: pitchOffset is new)
//   motor -> sensor:  $J,activeMode,ramp,steer,servoOk,types,a1,a2,...*CS

bool linkOk() { return everHeardMotor && millis() - lastReportMs < LINK_TIMEOUT_MS; }

void linkSend(const char* body) {
  linkPort.printf("$%s*%02X\n", body, xorSum(body));
}

void sendOrders() {
  char body[80];
  snprintf(body, sizeof(body), "S,%d,%.1f,%.2f,%.0f,%d,%.1f,%.1f",                // SIM: from the table if on
           requestedMode, orderAmp(), frequencyHz, orderLag(), travelDir, orderSteer(), orderPitchOffset());
  linkSend(body);
}

void handleReport(char* body) {
  // body = "J,activeMode,ramp,steer,servoOk,types,a1,...,aN"
  char* tok = strtok(body, ",");
  if (!tok || strcmp(tok, "J") != 0) { badFrames++; return; }
  char* f[5];
  for (int i = 0; i < 5; i++) if (!(f[i] = strtok(nullptr, ","))) { badFrames++; return; }

  int n = min((int)strlen(f[4]), MAX_JOINTS);
  float a[MAX_JOINTS];
  for (int i = 0; i < n; i++) {
    if (!(tok = strtok(nullptr, ","))) { badFrames++; return; }
    a[i] = atof(tok);
  }
  motorMode    = constrain(atoi(f[0]), 0, 3);
  motorRamp    = atof(f[1]);
  motorSteer   = atof(f[2]);
  motorServoOk = atoi(f[3]) != 0;
  numJoints    = n;
  memcpy(jointTypes, f[4], n);
  jointTypes[n] = 0;
  memcpy(jointDeg, a, sizeof(float) * n);
  simTable = simterrain::tableFor(jointTypes);            // SIM: the gait table for this joint layout

  if (!everHeardMotor) Serial.printf("Motor board connected (%d joints: %s)\n", n, jointTypes);
  everHeardMotor = true;
  lastReportMs = millis();
}

void linkRead() {
  static char buf[160];
  static int len = 0;
  while (linkPort.available()) {
    char c = linkPort.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = 0;
      char* body = checkFrame(buf);
      if (body) handleReport(body);
      else if (len > 0) badFrames++;
      len = 0;
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;
    }
  }
}

// ========================================================= Commands ========
// From the phone ("key=value") and the USB Serial Monitor
void applyCommand(const char* key, float v) {
  if (!strcmp(key, "cal")) { startImuCal(); return; }      // keeps its own message
  if (!strcmp(key, "sim")) {                               // SIM
    simGait = v > 0.5f;
    if (!simGait)          snprintf(lastMsg, sizeof(lastMsg), "Sim gait off - sliders in control");
    else if (simActive())  snprintf(lastMsg, sizeof(lastMsg), "Sim gait on");
    else                   snprintf(lastMsg, sizeof(lastMsg), "Sim gait on - it works in %s mode", modeName(simMode()));
    Serial.println(lastMsg);
    sendOrders();
    return;
  }
  if      (!strcmp(key, "mode"))  requestedMode  = (Mode)constrain((int)v, 0, 3);
  else if (!strcmp(key, "amp"))   amplitudeDeg   = constrain(v, 0.0f, MAX_AMP_DEG);
  else if (!strcmp(key, "freq"))  frequencyHz    = constrain(v, 0.1f, 2.0f);
  else if (!strcmp(key, "lag"))   phaseLagDeg    = constrain(v, 0.0f, 180.0f);
  else if (!strcmp(key, "steer")) manualSteerDeg = constrain(v, -MAX_STEER_DEG, MAX_STEER_DEG);
  else if (!strcmp(key, "auto"))  { autoSteer  = v > 0.5f; if (autoSteer)  lineFollow = false; }
  else if (!strcmp(key, "line"))  { lineFollow = v > 0.5f; if (lineFollow) autoSteer  = false; lineSteerDeg = 0; }
  else if (!strcmp(key, "obs"))   obstacleStop   = v > 0.5f;
  else if (!strcmp(key, "rev"))   travelDir      = -travelDir;
  else return;
  lastMsg[0] = 0;
  if (simActive() && (!strcmp(key, "amp") || !strcmp(key, "lag")))   // SIM
    snprintf(lastMsg, sizeof(lastMsg), "Sim gait is on - it sets amplitude and phase lag");
  checkLineFollow();                                      // e.g. "go" with no camera -> stop
  sendOrders();                                           // pass it on right away
}

const char* modeName(int m) {
  static const char* names[] = {"stop", "caterpillar", "serpentine", "sidewind"};
  return names[constrain(m, 0, 3)];
}

// ================================================ Untethered control ======
// BOOT button: short press = next mode (stop > caterpillar > serpentine >
// sidewind > stop). Hold 1 s = STOP. A press during the auto-start countdown
// cancels the auto-start.
void shortPress() {
  if (autoStartPending) {
    autoStartPending = false;
    Serial.println("Button: auto-start cancelled");
    snprintf(lastMsg, sizeof(lastMsg), "Auto-start cancelled by button");
    return;
  }
  int next = (requestedMode + 1) % 4;
  applyCommand("mode", next);
  Serial.printf("Button: mode -> %s\n", modeName(next));
  if (requestedMode == next) snprintf(lastMsg, sizeof(lastMsg), "Button: %s", modeName(next));
}

void longPress() {
  autoStartPending = false;
  applyCommand("mode", 0);
  Serial.println("Button held: STOP");
  snprintf(lastMsg, sizeof(lastMsg), "Button held: STOP");
}

void handleButton() {
  static bool lastRaw = HIGH, stable = HIGH, longDone = false;
  static uint32_t changedAt = 0, pressedAt = 0;
  uint32_t now = millis();
  bool raw = digitalRead(MODE_BUTTON_PIN);                // LOW = pressed
  if (raw != lastRaw) { lastRaw = raw; changedAt = now; }
  if (now - changedAt < 30) return;                       // debounce: wait until it settles
  if (raw != stable) {
    stable = raw;
    if (stable == LOW) { pressedAt = now; longDone = false; }
    else if (!longDone) shortPress();                     // released before 1 s
  }
  if (stable == LOW && !longDone && now - pressedAt >= LONG_PRESS_MS) {
    longDone = true;
    longPress();
  }
}

void handleAutoStart() {
  if (!autoStartPending || millis() - readyMs < AUTO_START_DELAY_MS) return;
  // Following the line: wait until the camera is connected
  if (lineFollow && STOP_IF_CAMERA_LOST && !camOk()) {
    static uint32_t lastNote = 0;
    if (millis() - lastNote > 2000) {
      lastNote = millis();
      snprintf(lastMsg, sizeof(lastMsg), "Auto-start waiting for the camera");
      Serial.println(lastMsg);
    }
    return;
  }
  autoStartPending = false;
  applyCommand("mode", AUTO_START_MODE);
  Serial.printf("Auto-start: %s\n", modeName(AUTO_START_MODE));
  snprintf(lastMsg, sizeof(lastMsg), "Auto-start: %s", modeName(AUTO_START_MODE));
}

// Blue LED tells you what is going on without a phone or computer:
//   short blip twice a second   counting down to auto-start
//   fast flicker                motor board not answering (check link wires/power)
//   steady on                   link OK, snake stopped
//   1 / 2 / 3 blinks, pause     running caterpillar / serpentine / sidewind
void updateLed() {
  uint32_t now = millis();
  bool on;
  if (autoStartPending)           on = (now % 500) < 80;
  else if (!linkOk())             on = (now % 200) < 100;
  else if (requestedMode == STOP) on = true;
  else {
    uint32_t t = now % 2000;
    on = (t < (uint32_t)requestedMode * 300) && ((t % 300) < 150);
  }
  digitalWrite(LED_PIN, on);
}

void printStatus() {
  const char* src = lineFollow ? " (line)" : (autoSteer ? " (IR auto)" : "");
  Serial.printf("Asked: %s | amp %.0f | %.1f Hz | lag %.0f | %s | steer %.1f%s\n",
                modeName(requestedMode), orderAmp(), frequencyHz, orderLag(),
                travelDir > 0 ? "forward" : "backward", steerTarget(), src);
  Serial.printf("Sim gait: %s | gait %s | pitch offset %.0f | slope %.1f | bend %.1f%s\n",   // SIM
                !simGait ? "off" : (simActive() ? "ON" : "on (not in its mode)"), SIM_SITUATION_NAMES[simSituation],
                orderPitchOffset(), terrainOk ? terrain.slope : NAN, terrainOk ? terrain.bend : NAN,
                terrainOk ? "" : "  (no slope: needs the motor link and a calibrated IMU)");
  Serial.printf("Motor: link %s | active %s | ramp %.0f%% | Servo2 %s | bad frames %lu\n",
                linkOk() ? "OK" : "LOST", modeName(motorMode), motorRamp * 100,
                motorServoOk ? "OK" : "MISSING", (unsigned long)badFrames);
  Serial.printf("Camera: %s%s | line %s | pos %+.2f | packets %lu (bad %lu) | follow %s\n",
                camOk() ? "OK" : (camEverHeard ? "LOST" : "never heard"),
                camOk() ? (camViaWire ? " (wire)" : " (Wi-Fi)") : "", camSeen ? "seen" : "not seen",
                camSteer, (unsigned long)camPackets, (unsigned long)camBadPackets, lineFollow ? "ON" : "off");
  Serial.printf("IMU front %s%s pitch %.1f | rear %s%s pitch %.1f | IR L %.1f R %.1f cm",
                frontImu.ok ? "OK" : "MISSING", frontImu.calibrated ? "" : " (not calibrated)", frontImu.pitch,
                rearImu.ok ? "OK" : "MISSING", rearImu.calibrated ? "" : " (not calibrated)", rearImu.pitch,
                irLeftCm, irRightCm);
  if (NUM_FSR > 0) {
    Serial.print(" | FSR mV:");
    for (int i = 0; i < NUM_FSR; i++) Serial.printf(" %.0f", fsrMv[i]);
  }
  Serial.println();
}

void handleUsb() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '0': applyCommand("mode", 0); break;
      case '1': applyCommand("mode", 1); break;
      case '2': applyCommand("mode", 2); break;
      case '3': applyCommand("mode", 3); break;
      case '+': applyCommand("amp",  amplitudeDeg + 5); break;
      case '-': applyCommand("amp",  amplitudeDeg - 5); break;
      case 'f': applyCommand("freq", frequencyHz + 0.1f); break;
      case 's': applyCommand("freq", frequencyHz - 0.1f); break;
      case 'r': applyCommand("rev",  0); break;
      case 'l': applyCommand("line", lineFollow ? 0 : 1); break;
      case 'g': applyCommand("sim",  simGait ? 0 : 1); break;      // SIM
      case 'c': applyCommand("cal", 0); continue;        // result is printed when done
      case '?': break;
      default: continue;
    }
    printStatus();
  }
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED)    Serial.printf("Phone #%u connected\n", num);
  if (type == WStype_DISCONNECTED) Serial.printf("Phone #%u disconnected\n", num);
  if (type == WStype_TEXT) {
    char msg[32];
    len = min(len, sizeof(msg) - 1);
    memcpy(msg, payload, len);
    msg[len] = 0;
    char* eq = strchr(msg, '=');
    float v = 0;
    if (eq) { *eq = 0; v = atof(eq + 1); }
    applyCommand(msg, v);
  }
}

// ========================================================= Telemetry =======
int putNum(char* buf, int n, int size, const char* key, float v, int decimals) {
  if (isnan(v)) return n + snprintf(buf + n, size - n, "\"%s\":null,", key);
  return n + snprintf(buf + n, size - n, "\"%s\":%.*f,", key, decimals, v);
}

void sendTelemetry() {
  static char buf[1200];
  const int S = sizeof(buf);
  bool lk = linkOk(), co = camOk();
  int n = snprintf(buf, S, "{\"t\":%lu,\"mode\":%d,\"act\":%d,\"ramp\":%.2f,\"dir\":%d,"
                   "\"amp\":%.0f,\"freq\":%.1f,\"lag\":%.0f,\"steer\":%.1f,\"msteer\":%.0f,"
                   "\"auto\":%d,\"obs\":%d,\"ostop\":%.0f,\"lk\":%d,\"sv\":%d,\"fi\":%d,\"ri\":%d,"
                   "\"lf\":%d,\"cam\":%d,\"cs\":%d,\"cw\":%d,\"fic\":%d,\"ric\":%d,\"cal\":%d,"
                   "\"sg\":%d,\"sa\":%d,\"sit\":%d,\"sm\":%d,",                           // SIM
                   (unsigned long)millis(), requestedMode, lk ? motorMode : 0, lk ? motorRamp : 0.0f,
                   travelDir, orderAmp(), frequencyHz, orderLag(), motorSteer, manualSteerDeg,
                   autoSteer, obstacleStop, OBSTACLE_STOP_CM, lk, lk && motorServoOk,
                   frontImu.ok, rearImu.ok, lineFollow, co, co && camSeen, camViaWire,
                   frontImu.calibrated, rearImu.calibrated, calRunning,
                   simGait, simActive(), simSituation, simMode());
  n = putNum(buf, n, S, "slp",  terrainOk ? terrain.slope : NAN, 1);      // SIM
  n = putNum(buf, n, S, "bnd",  terrainOk ? terrain.bend  : NAN, 1);
  n = putNum(buf, n, S, "poff", orderPitchOffset(), 0);
  n = putNum(buf, n, S, "fp",  frontImu.pitch, 1);
  n = putNum(buf, n, S, "fr",  frontImu.roll,  1);
  n = putNum(buf, n, S, "rp",  rearImu.pitch,  1);
  n = putNum(buf, n, S, "rr",  rearImu.roll,   1);
  n = putNum(buf, n, S, "irl", irLeftCm,  1);
  n = putNum(buf, n, S, "irr", irRightCm, 1);
  n = putNum(buf, n, S, "cpos", (co && camSeen) ? camSteer : NAN, 2);
  // Line steering is always worked out while the camera talks, so you can
  // check it on the dashboard BEFORE ticking "Follow line" (preview).
  n = putNum(buf, n, S, "lsd",  co ? lineSteerDeg : NAN, 1);

  n += snprintf(buf + n, S - n, "\"types\":\"%s\",\"j\":[", jointTypes);
  for (int i = 0; i < numJoints; i++) n += snprintf(buf + n, S - n, "%s%.1f", i ? "," : "", jointDeg[i]);
  snprintf(buf + n, S - n, "],\"lp\":%lu,\"msg\":\"%s\"}", (unsigned long)maxLoopUs, lastMsg);

  webSocket.broadcastTXT(buf);
}

// ====================================================== Web page ===========
// Everything is inline: the phone has no internet while on the snake's Wi-Fi.
// %CAMERA_URL% is replaced with CAMERA_URL when the page is sent.
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SnakeBot</title>
<style>
 *{box-sizing:border-box}
 body{font-family:system-ui,sans-serif;background:#121212;color:#eee;margin:0 auto;padding:12px;max-width:720px}
 h2{margin:4px 0 12px;font-size:20px}
 .card{background:#1e1e1e;border-radius:10px;padding:10px 12px;margin-bottom:12px}
 .card h3{margin:0 0 8px;font-size:12px;color:#9a9a9a;font-weight:600;text-transform:uppercase;letter-spacing:.05em}
 .row{display:flex;align-items:center;margin:6px 0;gap:8px}
 .lbl{width:92px;font-size:14px;white-space:nowrap}
 .bar{flex:1;height:16px;background:#333;position:relative;border-radius:4px;overflow:hidden}
 .fill{position:absolute;top:0;bottom:0;left:0;width:0}
 .mid{position:absolute;left:50%;top:0;bottom:0;width:1px;background:#888;z-index:1}
 .val{width:70px;text-align:right;font-variant-numeric:tabular-nums;font-size:14px}
 canvas{width:100%;display:block}
 #status{font-size:12px;padding:2px 8px;border-radius:10px;vertical-align:middle;margin-left:6px;background:#555}
 .btns{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-bottom:6px}
 button{font:inherit;font-size:14px;padding:12px 2px;border:0;border-radius:8px;background:#333;color:#eee}
 button.on{background:#42a5f5;color:#000;font-weight:600}
 #stop{background:#c62828;color:#fff;font-weight:700}
 input[type=range]{flex:1;min-width:0}
 .tog{display:flex;gap:14px;flex-wrap:wrap;align-items:center;font-size:14px;margin-top:8px}
 .tog input{transform:scale(1.3);margin-right:6px}
 .tog button{padding:8px 14px}
 .dot{display:inline-block;width:11px;height:11px;border-radius:50%;background:#444;margin-left:6px}
 .stats{display:flex;flex-wrap:wrap;gap:6px 18px;font-size:14px}
 .stats b{font-variant-numeric:tabular-nums}
 #msg{color:#ffca28;font-size:14px;min-height:1.2em;margin-top:6px}
 .ok{color:#66bb6a}.bad{color:#ef5350}
 .note{color:#9a9a9a;font-size:14px}
 a{color:#42a5f5}
 #camimg{width:100%;display:none;border-radius:6px;margin-top:8px;background:#000}
</style></head><body>
<h2>SnakeBot <span id="status">connecting…</span></h2>

<div class="card"><h3>Control</h3>
 <div class="btns">
  <button id="stop" onclick="send('mode=0')">STOP</button>
  <button id="m1" onclick="send('mode=1')">Caterpillar</button>
  <button id="m2" onclick="send('mode=2')">Serpentine</button>
  <button id="m3" onclick="send('mode=3')">Sidewind</button>
 </div>
 <div class="row"><div class="lbl">Amplitude</div><input type="range" id="amp" min="0" max="45" step="1"><div class="val" id="ampv">-</div></div>
 <div class="row"><div class="lbl">Speed (Hz)</div><input type="range" id="freq" min="0.1" max="2" step="0.1"><div class="val" id="freqv">-</div></div>
 <div class="row"><div class="lbl">Phase lag</div><input type="range" id="lag" min="0" max="180" step="5"><div class="val" id="lagv">-</div></div>
 <div class="row"><div class="lbl">Steering</div><input type="range" id="steer" min="-30" max="30" step="1"><div class="val" id="steerv">-</div></div>
 <div class="tog">
  <label><input type="checkbox" id="line">Follow line (camera)</label>
  <label><input type="checkbox" id="sim">Sim gait</label>
  <label><input type="checkbox" id="auto">Auto-steer (IR)</label>
  <label><input type="checkbox" id="obs">Stop at obstacle</label>
  <button onclick="send('rev')">Reverse: <span id="dir">-</span></button>
 </div>
 <div class="note" id="simst" style="margin-top:8px">-</div>
 <div id="msg"></div>
</div>

<div class="card"><h3>Camera line <span id="camst" class="bad">-</span></h3>
 <div id="cam"></div>
 <div class="tog">
  <label><input type="checkbox" id="camv">Show camera video</label>
  <a href="%CAMERA_URL%" target="_blank">Open camera page (tuning)</a>
 </div>
 <img id="camimg" alt="">
</div>

<div class="card"><h3>Snake shape (top = yaw joints, bottom = pitch joints, ● = head)</h3><canvas id="shape" style="height:200px"></canvas></div>
<div class="card"><h3>Joint angles (from motor board)</h3><div id="joints"></div></div>
<div class="card"><h3>Joint history (last 10 s)</h3><canvas id="chart" style="height:180px"></canvas></div>
<div class="card"><h3>Tilt from IMUs (degrees) <span id="icst"></span></h3><div id="imus"></div>
 <div class="tog"><button id="calb" onclick="send('cal')">Calibrate IMUs</button>
  <span class="note">Snake flat, straight and still, STOP pressed. Takes 1 s.</span></div>
</div>
<div class="card"><h3>Front distance (Sharp IR)</h3><div id="ir"></div></div>
<div class="card stats">
 <span>Mode <b id="mode">-</b></span><span>Ramp <b id="ramp">-</b></span>
 <span>Motor link <b id="lk">-</b></span><span>Servo2 <b id="sv">-</b></span>
 <span>Camera <b id="ca">-</b></span>
 <span>Front IMU <b id="fi">-</b></span><span>Rear IMU <b id="ri">-</b></span>
 <span>Max loop <b id="lp">-</b> µs</span><span>Uptime <b id="up">-</b> s</span>
</div>

<script>
const C=['#ef5350','#42a5f5','#66bb6a','#ffca28','#ab47bc','#26c6da','#ff7043','#8d6e63'];
const HIST=200; let hist=[], built=false, nj=-1, ws=null, last=null;
const $=id=>document.getElementById(id);
function send(m){ if(ws&&ws.readyState===1) ws.send(m); }

[['amp',0],['freq',1],['lag',0],['steer',0]].forEach(([id,dec])=>{
  const s=$(id); let t=0;
  s.oninput=()=>{ $(id+'v').textContent=(+s.value).toFixed(dec);
    const n=Date.now(); if(n-t>120){t=n; send(id+'='+s.value);} };
  s.onchange=()=>send(id+'='+s.value);
});
$('auto').onchange=e=>send('auto='+(e.target.checked?1:0));
$('line').onchange=e=>send('line='+(e.target.checked?1:0));
$('sim').onchange =e=>send('sim=' +(e.target.checked?1:0));
$('obs').onchange =e=>send('obs=' +(e.target.checked?1:0));
// Camera video comes straight from the camera (port 81). Only one viewer at a time!
$('camv').onchange=e=>{ const i=$('camimg');
  i.src=e.target.checked?'%CAMERA_URL%:81/stream?t='+Date.now():'data:,';
  i.style.display=e.target.checked?'block':'none'; };

function bar(id,label,color,centered){
  return `<div class="row"><div class="lbl" style="color:${color}">${label}</div><div class="bar">`+
    (centered?'<div class="mid"></div>':'')+`<div class="fill" id="${id}f" style="background:${color}"></div></div>`+
    `<div class="val" id="${id}v">-</div></div>`;
}
function buildJoints(d){
  $('joints').innerHTML=d.j.length ? d.j.map((_,i)=>bar('j'+i,'J'+(i+1)+' '+(d.types[i]=='P'?'pitch':'yaw'),C[i%8],1)).join('')
                                   : '<div class="note">Waiting for the motor board…</div>';
  nj=d.j.length; hist=[];
}
function build(d){
  $('cam').innerHTML=bar('cp','Line position','#ffca28',1)+bar('ls','<span id="lsl">Line steering</span>','#42a5f5',1);
  $('imus').innerHTML=bar('fp','Front pitch','#66bb6a',1)+bar('fr','Front roll','#26c6da',1)+
                      bar('rp','Rear pitch','#66bb6a',1)+bar('rr','Rear roll','#26c6da',1);
  $('ir').innerHTML=bar('irl','Left','#ab47bc',0)+bar('irr','Right','#ab47bc',0);
  built=true;
}
function setC(id,v,range,unit,dec){
  const f=$(id+'f');
  if(v===null){ f.style.width='0'; $(id+'v').textContent='n/a'; return; }
  const p=Math.max(-range,Math.min(range,v))/range*50;
  f.style.left=(p>=0?50:50+p)+'%'; f.style.width=Math.abs(p)+'%';
  $(id+'v').textContent=v.toFixed(dec===undefined?1:dec)+unit;
}
function setL(id,v,max,text,color){
  const f=$(id+'f'); f.style.width=Math.max(0,Math.min(1,v/max))*100+'%';
  if(color) f.style.background=color;
  $(id+'v').textContent=text;
}
function okTxt(id,v){ const e=$(id); e.textContent=v?'OK':'MISSING'; e.className=v?'ok':'bad'; }
function sync(id,v,dec){ const s=$(id); if(document.activeElement!==s){ s.value=v; $(id+'v').textContent=(+v).toFixed(dec);} }

function drawShape(d){
  const c=$('shape'), r=devicePixelRatio||1;
  const w=c.width=c.clientWidth*r, h=c.height=c.clientHeight*r, x=c.getContext('2d');
  if(!d.j.length) return;
  const n=d.j.length+1, L=(w-40*r)/n, deg=Math.PI/180;
  x.font=(11*r)+'px system-ui'; x.fillStyle='#777';
  x.fillText('top view',4*r,14*r); x.fillText('side view',4*r,h/2+14*r);
  x.strokeStyle='#333'; x.lineWidth=r; x.beginPath(); x.moveTo(0,h/2); x.lineTo(w,h/2); x.stroke();
  [['Y',h/4],['P',3*h/4]].forEach(([type,cy])=>{
    x.save(); x.beginPath(); x.rect(0,cy-h/4,w,h/2); x.clip();
    let px=20*r, py=cy, a=0, pts=[[px,py]];
    for(let i=0;i<n;i++){ if(i>0 && d.types[i-1]==type) a+=d.j[i-1]*deg;
      px+=L*Math.cos(a); py+=L*Math.sin(a)*(type=='P'?-1:1); pts.push([px,py]); }
    x.lineCap='round';
    for(let i=0;i<n;i++){ x.strokeStyle=i%2?'#888':'#bbb'; x.lineWidth=8*r; x.beginPath();
      x.moveTo(pts[i][0],pts[i][1]); x.lineTo(pts[i+1][0],pts[i+1][1]); x.stroke(); }
    for(let i=1;i<n;i++){ x.fillStyle=C[(i-1)%8]; x.beginPath(); x.arc(pts[i][0],pts[i][1],4*r,0,7); x.fill(); }
    x.fillStyle='#fff'; x.beginPath(); x.arc(pts[0][0],pts[0][1],6*r,0,7); x.fill();
    x.restore();
  });
}
function drawChart(){
  const c=$('chart'), r=devicePixelRatio||1;
  const w=c.width=c.clientWidth*r, h=c.height=c.clientHeight*r, x=c.getContext('2d');
  x.strokeStyle='#444'; x.lineWidth=r;
  [-45,0,45].forEach(v=>{const y=h/2-v/60*h/2; x.beginPath(); x.moveTo(0,y); x.lineTo(w,y); x.stroke();});
  if(!hist.length) return;
  for(let j=0;j<hist[0].length;j++){
    x.strokeStyle=C[j%8]; x.lineWidth=2*r; x.beginPath();
    hist.forEach((s,k)=>{const px=k/(HIST-1)*w, py=h/2-s[j]/60*h/2; k?x.lineTo(px,py):x.moveTo(px,py);});
    x.stroke();
  }
}
let pending=false;
function render(){
  pending=false; const d=last; if(!d) return;
  if(!built) build(d);
  if(d.j.length!==nj) buildJoints(d);
  d.j.forEach((a,i)=>setC('j'+i,a,60,'°'));
  setC('fp',d.fp,45,'°'); setC('fr',d.fr,45,'°'); setC('rp',d.rp,45,'°'); setC('rr',d.rr,45,'°');
  setC('cp',d.cpos,1,'',2); setC('ls',d.lsd,30,'°');
  const ic=$('icst');
  ic.textContent=d.cal?'calibrating…':((d.fic&&d.ric)?'calibrated':'NOT calibrated: front '+(d.fic?'ok':'no')+', rear '+(d.ric?'ok':'no'));
  ic.className=(d.fic&&d.ric)?'ok':'bad'; $('calb').disabled=!!d.cal;
  $('lsl').textContent=d.lf?'Line steering':'Steer (preview)';
  const cs=$('camst');
  cs.textContent=!d.cam?'camera not connected':((d.cs?'line seen':(d.lf?'no line - going straight':'no line'))+(d.cw?' (wire)':' (Wi-Fi)'));
  cs.className=d.cam&&d.cs?'ok':'bad';
  [['irl',d.irl],['irr',d.irr]].forEach(([id,v])=>{
    if(v===null) setL(id,80,80,'nothing','#555');
    else setL(id,v,80,v.toFixed(0)+' cm', v<10?'#ef5350':(v<d.ostop?'#ffca28':'#ab47bc'));
  });
  for(let k=1;k<=3;k++) $('m'+k).className=(d.mode==k?'on':'');
  sync('amp',d.amp,0); sync('freq',d.freq,1); sync('lag',d.lag,0); sync('steer',d.msteer,0);
  $('auto').checked=!!d.auto; $('line').checked=!!d.lf; $('obs').checked=!!d.obs;
  $('sim').checked=!!d.sg;
  $('simst').textContent='Sim gait '+(!d.sg?'off':(d.sa?'ON: '+['flat','uphill','downhill','crest','valley','maze'][d.sit]+
      (d.poff?' (pitch offset '+d.poff+'°)':''):'on - works in '+['','caterpillar','serpentine'][d.sm]+' mode'))+
    ' | slope '+(d.slp===null?'n/a (needs motor link + calibrated IMU)':d.slp.toFixed(1)+'° | bend '+d.bnd.toFixed(1)+'°');
  $('dir').textContent=d.dir>0?'forward':'backward';
  $('msg').textContent=d.lk?d.msg:'Motor board not answering - check the UART wires and that it is powered';
  $('mode').textContent=['stop','caterpillar','serpentine','sidewind'][d.act];
  $('ramp').textContent=Math.round(d.ramp*100)+'%';
  okTxt('lk',d.lk); okTxt('sv',d.sv); okTxt('ca',d.cam); okTxt('fi',d.fi); okTxt('ri',d.ri);
  $('lp').textContent=d.lp; $('up').textContent=(d.t/1000).toFixed(0);
  drawShape(d); drawChart();
}
function status(t,col){ const s=$('status'); s.textContent=t; s.style.background=col; }
function connect(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=()=>status('live','#2e7d32');
  ws.onclose=()=>{ status('disconnected','#c62828'); setTimeout(connect,1000); };
  ws.onmessage=e=>{ last=JSON.parse(e.data);
    if(last.j.length===nj){ hist.push(last.j); if(hist.length>HIST) hist.shift(); }
    if(!pending){ pending=true; requestAnimationFrame(render);} };
}
connect();
</script></body></html>
)rawliteral";

// ===================================================== Setup / loop ========
void scanBus(TwoWire* bus, const char* name) {
  Serial.printf("I2C %s:", name);
  int found = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++)
    if (i2cPresent(bus, a)) { Serial.printf(" 0x%02X", a); found++; }
  Serial.println(found ? "" : " nothing found - check wiring");
}

void probeImus(bool verbose) {
  if (!frontImu.ok) { frontImu.ok = imuInit(frontImu); frontImu.fails = 0; }
  if (!rearImu.ok)  { rearImu.ok  = imuInit(rearImu);  rearImu.fails  = 0; }
  if (verbose) Serial.printf("Front IMU (bus A): %s | rear IMU (bus B): %s\n",
                             frontImu.ok ? "OK" : "NOT FOUND", rearImu.ok ? "OK" : "NOT FOUND");
}

void sendDashboard() {
  String page = FPSTR(DASHBOARD_HTML);
  page.replace("%CAMERA_URL%", CAMERA_URL);
  server.send(200, "text/html", page);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nSnakeBot SENSOR board starting");
  pinMode(LED_PIN, OUTPUT);

  prefs.begin("snakebot", false);
  imuLoadCal(frontImu);
  imuLoadCal(rearImu);

  Wire.begin(BUS_A_SDA, BUS_A_SCL, I2C_HZ);
  Wire1.begin(BUS_B_SDA, BUS_B_SCL, I2C_HZ);
  Wire.setTimeOut(10);                    // ms: a broken cable must not freeze the board
  Wire1.setTimeOut(10);
  scanBus(&Wire,  "bus A (expect 0x4C)");
  scanBus(&Wire1, "bus B (expect 0x4C)");
  probeImus(true);

  analogReadResolution(12);
  analogSetPinAttenuation(IR_LEFT_PIN,  ADC_11db);
  analogSetPinAttenuation(IR_RIGHT_PIN, ADC_11db);
  for (int i = 0; i < NUM_FSR; i++) analogSetPinAttenuation(FSR_PINS[i], ADC_11db);

  linkPort.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);
  camPort.begin(CAM_BAUD, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
  Serial.printf("Listening for the camera on the wire (GPIO%d, %lu baud)\n", CAM_RX_PIN, (unsigned long)CAM_BAUD);

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("Join Wi-Fi \"%s\" (password %s) and open http://%s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  server.on("/", sendDashboard);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWsEvent);
  camUdp.begin(LINE_UDP_PORT);
  Serial.printf("... and on Wi-Fi UDP port %u (backup)\n", LINE_UDP_PORT);

  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  obstacleStop = START_OBSTACLE_STOP;
  autoSteer    = START_AUTO_STEER && !START_LINE_FOLLOW;
  lineFollow   = START_LINE_FOLLOW;
  simGait      = START_SIM_GAIT;                  // SIM

  Serial.println("Ready. USB: 0-3 mode, +/- amp, f/s speed, r reverse, l line follow, g sim gait, c calibrate IMUs, ? status");
  Serial.println("BOOT button: press = next mode, hold 1 s = STOP");
  if (AUTO_START)
    Serial.printf("Auto-start in %lu s: %s%s (press BOOT to cancel)\n",
                  (unsigned long)(AUTO_START_DELAY_MS / 1000), modeName(AUTO_START_MODE),
                  lineFollow ? ", following the line" : "");
  readyMs = millis();
}

void loop() {
  uint32_t start = micros();

  server.handleClient();
  webSocket.loop();
  linkRead();
  readCamera();
  checkLineFollow();
  handleUsb();
  handleButton();
  handleAutoStart();
  updateLed();

  uint32_t now = millis();
  if (now - lastImu >= IMU_MS) {
    lastImu = now;
    imuUpdate(frontImu);
    imuUpdate(rearImu);
    updateImuCal();
    sampleSlope();                          // SIM
  }
  if (now - lastTerrain >= TERRAIN_MS) {    // SIM: choose the simulation's gait from the slope
    float dt = (now - lastTerrain) / 1000.0f;
    lastTerrain = now;
    updateTerrain(dt);
  }
  if (now - lastSensor >= SENSOR_MS) {
    lastSensor = now;
    readAnalogSensors();
  }
  if (now - lastOrder >= ORDER_MS) {      // also acts as the heartbeat for the motor board
    lastOrder = now;
    sendOrders();
  }
  if (now - lastProbe >= REPROBE_MS) {
    lastProbe = now;
    if (!frontImu.ok || !rearImu.ok) probeImus(false);
  }
  if (now - lastTelemetry >= TELEMETRY_MS) {
    lastTelemetry = now;
    sendTelemetry();
    maxLoopUs = 0;
  }

  uint32_t dt = micros() - start;
  if (dt > maxLoopUs) maxLoopUs = dt;
}
