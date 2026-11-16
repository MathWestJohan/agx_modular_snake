/*
  ============================================================================
   SnakeCam LINE - finds a coloured tape line on the floor (Timer Camera X)
  ============================================================================
   Step 1 of line following: the camera finds the line by itself and shows
   on your phone what it sees. Test this on the course floor BEFORE it is
   connected to the robot.

   Board : M5Stack Timer Camera X (ESP32 + 8 MB PSRAM + OV3660 camera)
   Arduino IDE settings: same as snake_camera_test.ino
     Board "ESP32 Dev Module" (or "M5Stack-Timer-CAM"), PSRAM "Enabled",
     Partition Scheme "Huge APP (3MB No OTA/1MB SPIFFS)"
   Libraries: none extra

   HOW TO TEST
     1. Upload. Join Wi-Fi "SnakeCam" (password snake1234), open http://192.168.4.1
     2. Hold the camera like it will sit on the snake head: tilted down
        30-45 deg, looking at the floor 10-40 cm ahead.
     3. Tick "Tap to pick colour" and tap the TAPE in the video.
        The colour is saved and survives power-off.
     4. Yellow = pixels the camera thinks are tape. Red lines = where it thinks
        the line is (top band = further ahead, bottom band = close).
        Adjust "Tolerance" until the tape is yellow and the grey floor isn't.
     5. Blue LED on the camera = line seen. Handy when the phone isn't watching.

   HOW IT WORKS
     ~10 times a second a 320x240 frame is checked on an 80x60 grid. A pixel
     counts as "tape" if it is colourful enough (not grey) and its colour mix
     is close to the colour you picked. Brightness is ignored, so shadows
     matter less. Result: line position -1 (far left) .. 0 (centre) .. +1 (far right).

   ON THE ROBOT: the line position goes to the sensor ESP32 over a WIRE
   (serial/UART), 20 times a second:
       $L,seen,steer,near,far,nearSeen,farSeen*CS      (CS = XOR, like the motor link)
   WIRING (2 wires, from the camera's Grove port):
       camera G4  (Grove signal)  ->  sensor ESP32 GPIO26
       camera GND (Grove black)   ->  sensor ESP32 GND      <-- REQUIRED, even on battery!
       (G13 and 5V on the Grove port: leave unconnected)
   Both boards are 3.3 V, so the wire goes straight across - no resistors needed.

   Wi-Fi is then only needed for the tuning page:
     CAMERA_WIFI = true,  JOIN_ROBOT_WIFI = false -> own "SnakeCam" network (tuning)
     CAMERA_WIFI = true,  JOIN_ROBOT_WIFI = true  -> joins "Beanie" as 192.168.4.50,
                                                     also sends by UDP (backup)
     CAMERA_WIFI = false                          -> Wi-Fi off: wire only, saves battery
   When joining Beanie: blue LED fast flicker = not joined yet, it keeps retrying.
  ============================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================ CONFIGURE ME ==================================
// ---- Wire to the sensor ESP32 (UART on the Grove port) ----
const bool     USE_WIRE    = true;
const int      WIRE_TX_PIN = 4;               // Grove G4 -> sensor ESP32 GPIO26
const int      WIRE_RX_PIN = 13;              // Grove G13, not connected (the UART driver wants a pin)
const uint32_t WIRE_BAUD   = 115200;          // MUST match CAM_BAUD in snake_sensors.ino

// ---- Wi-Fi ----
const bool  CAMERA_WIFI     = true;           // false = Wi-Fi completely off (wire only, saves battery)
const bool  JOIN_ROBOT_WIFI = true;          // true = join the sensor ESP32's network (+ UDP), false = own "SnakeCam" network
const char* OWN_AP_SSID   = "SnakeCam";       // used when not joining (stand-alone test)
const char* OWN_AP_PASS   = "snake1234";
const char* ROBOT_SSID    = "Beanie";         // AP_SSID in snake_sensors.ino
const char* ROBOT_PASS    = "beanie0987";     // AP_PASS in snake_sensors.ino
const IPAddress CAM_IP  (192, 168, 4, 50);    // camera's fixed address on the robot network
const IPAddress ROBOT_IP(192, 168, 4, 1);     // the sensor ESP32
const IPAddress CAM_MASK(255, 255, 255, 0);
const uint32_t JOIN_TIMEOUT_MS = 10000;       // wait this long at start-up before going on
const uint32_t JOIN_RETRY_MS   = 8000;        // not joined: try again this often
const uint16_t LINE_UDP_PORT   = 4210;        // the sensor ESP32 listens here (step 2)
const uint32_t SEND_MS         = 50;          // line data to the robot: 20 Hz (wire and/or UDP)

// ---- Picture ----
const int  START_QUALITY = 12;                // JPEG: 10 = best ... 63 = worst
const bool START_VFLIP   = true;              // OV3660 is mounted upside down on most units
const bool START_HMIRROR = false;             // NOTE: mirror flips left/right of the line too

// ---- Line detection (Tolerance and Min colour can be changed on the page) ----
const float START_TOLERANCE = 0.10;           // how close to the picked colour (0.02 strict .. 0.40 loose)
const int   START_MIN_SAT   = 30;             // how colourful a pixel must be (grey floor ~0-20)
const int   MIN_BRIGHTNESS  = 60;             // r+g+b below this = too dark to judge
const int   FAR_ROW0  = 15;                   // grid rows 0-14 are ignored (too far / horizon)
const int   NEAR_ROW0 = 35;                   // rows 15-34 = "far" band, 35-59 = "near" band
const int   MIN_BAND_PIXELS = 6;              // fewer tape pixels than this in a band = not seen
const float NEAR_WEIGHT = 0.6;                // steer = 0.6 * near + 0.4 * far
// ============================================================================

// ---- Timer Camera X pins (from the M5Stack pin map, do not change) ----
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  15
#define XCLK_GPIO_NUM   27
#define SIOD_GPIO_NUM   25
#define SIOC_GPIO_NUM   23
#define Y9_GPIO_NUM     19
#define Y8_GPIO_NUM     36
#define Y7_GPIO_NUM     18
#define Y6_GPIO_NUM     39
#define Y5_GPIO_NUM      5
#define Y4_GPIO_NUM     34
#define Y3_GPIO_NUM     35
#define Y2_GPIO_NUM     32
#define VSYNC_GPIO_NUM  22
#define HREF_GPIO_NUM   26
#define PCLK_GPIO_NUM   21
const int LED_PIN      = 2;                   // blue LED = line seen
const int BAT_HOLD_PIN = 33;                  // must be HIGH or it switches off on battery
const int BAT_ADC_PIN  = 38;

// ---- Image / grid sizes ----
const int IMG_W = 320, IMG_H = 240;           // QVGA, fixed for line following
const int STEP  = 4;                          // check every 4th pixel
const int GW = IMG_W / STEP, GH = IMG_H / STEP;   // 80 x 60 grid

// ---------------------------------------------------------------- state
httpd_handle_t webServer = NULL, streamServer = NULL;
Preferences prefs;
WiFiUDP udp;
bool joinedRobot = false;

volatile uint32_t framesSent = 0, lastFrameBytes = 0;
volatile bool streaming = false;
float streamFps = 0, detectFps = 0;
uint32_t detectCount = 0, detectMs = 0;

uint8_t* rgb = NULL;                          // decoded 320x240 picture (RGB888, in PSRAM)
uint8_t rawMask[GW * GH], mask[GW * GH];
uint8_t maskBits[GW * GH / 8];                // packed copy for the web page

// Colour to look for (as a mix: the three parts add up to 1)
bool  targetSet = false;
float target[3] = {0.33, 0.33, 0.33};
float tolerance = START_TOLERANCE;
int   minSat    = START_MIN_SAT;

volatile bool pickRequested = false;
volatile float pickX = 0.5, pickY = 0.5;

// Line position in one band of the picture. NOTE: keep all struct types up
// here, above the first function (the Arduino IDE needs that, see snake_sensors.ino).
struct Band { bool seen; float pos; int count; };

// Results
bool  lineSeen = false, nearSeen = false, farSeen = false;
float nearPos = 0, farPos = 0, steerHint = 0;
char  lastMsg[64] = "";

// ====================================================== Web page ===========
const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SnakeCam LINE</title>
<style>
 *{box-sizing:border-box}
 body{font-family:system-ui,sans-serif;background:#121212;color:#eee;margin:0 auto;padding:12px;max-width:720px}
 h2{margin:4px 0 12px;font-size:20px}
 .card{background:#1e1e1e;border-radius:10px;padding:10px 12px;margin-bottom:12px}
 #wrap{position:relative;width:100%;aspect-ratio:4/3;background:#000;border-radius:6px;overflow:hidden}
 #view,#ov{position:absolute;inset:0;width:100%;height:100%}
 #wrap.pick{outline:3px solid #ffca28;cursor:crosshair}
 .row{display:flex;align-items:center;margin:8px 0;gap:8px;font-size:14px}
 .lbl{width:92px}
 input[type=range]{flex:1;min-width:0}
 .val{width:40px;text-align:right}
 .tog{display:flex;gap:16px;flex-wrap:wrap;font-size:14px;margin:8px 0}
 .tog input{transform:scale(1.3);margin-right:6px}
 button{font:inherit;font-size:14px;padding:10px 14px;border:0;border-radius:8px;background:#333;color:#eee}
 #st{font-size:12px;padding:2px 8px;border-radius:10px;vertical-align:middle;margin-left:6px;background:#555}
 #big{font-size:22px;font-weight:700;margin:4px 0 8px}
 .bar{height:18px;background:#333;position:relative;border-radius:4px;overflow:hidden}
 .mid{position:absolute;left:50%;top:0;bottom:0;width:1px;background:#888}
 #steerf{position:absolute;top:0;bottom:0;background:#42a5f5}
 .small{font-size:12px;color:#9a9a9a;font-variant-numeric:tabular-nums}
</style></head><body>
<h2>SnakeCam LINE <span id="st">connecting…</span></h2>
<div class="card">
 <div id="wrap"><img id="view" alt=""><canvas id="ov" width="320" height="240"></canvas></div>
 <div id="big" style="margin-top:8px">-</div>
 <div class="bar"><div class="mid"></div><div id="steerf"></div></div>
 <div class="small" id="info" style="margin-top:6px">-</div>
 <div id="msg" style="color:#ffca28;font-size:14px;min-height:1.2em;margin-top:4px"></div>
</div>
<div class="card">
 <div class="tog">
  <label><input type="checkbox" id="pick">Tap to pick colour</label>
  <label><input type="checkbox" id="show" checked>Show detection</label>
  <button id="go">Stop video</button>
 </div>
 <div class="row"><div class="lbl">Tolerance</div><input type="range" id="tol" min="2" max="40"><div class="val" id="tolv">-</div></div>
 <div class="row"><div class="lbl">Min colour</div><input type="range" id="sat" min="0" max="100"><div class="val" id="satv">-</div></div>
 <div class="small">Tape not fully yellow → raise Tolerance or lower Min colour. Floor turning yellow → the opposite.</div>
</div>
<div class="card">
 <div class="row"><div class="lbl">JPEG quality</div><input type="range" id="quality" min="10" max="40"><div class="val" id="qualityv">-</div></div>
 <div class="tog">
  <label><input type="checkbox" id="vflip">Flip</label>
  <label><input type="checkbox" id="hmirror">Mirror</label>
 </div>
 <div class="small" id="info2">-</div>
</div>
<script>
const $=id=>document.getElementById(id);
const streamUrl='http://'+location.hostname+':81/stream';
const cx=$('ov').getContext('2d');
let on=false, first=true, firstLine=true;
function setStream(v){ on=v; $('view').src=v?streamUrl+'?t='+Date.now():'data:,'; $('go').textContent=v?'Stop video':'Start video'; }
function set(k,v){ fetch('/control?var='+k+'&val='+v); }
$('go').onclick=()=>setStream(!on);
['quality','tol','sat'].forEach(id=>{ const s=$(id);
  s.oninput=()=>$(id+'v').textContent=s.value; s.onchange=()=>set(id,s.value); });
['vflip','hmirror'].forEach(id=>$(id).onchange=e=>set(id,e.target.checked?1:0));
$('pick').onchange=e=>$('wrap').classList.toggle('pick',e.target.checked);
$('wrap').onclick=e=>{
  if(!$('pick').checked) return;
  const r=$('wrap').getBoundingClientRect();
  fetch('/pick?x='+((e.clientX-r.left)/r.width).toFixed(3)+'&y='+((e.clientY-r.top)/r.height).toFixed(3));
  $('pick').checked=false; $('wrap').classList.remove('pick');
};

function drawOverlay(d){
  cx.clearRect(0,0,320,240);
  if(!$('show').checked) return;
  const s=320/d.gw;
  cx.fillStyle='rgba(255,235,59,0.6)';
  for(let b=0;b<d.mask.length/2;b++){
    const v=parseInt(d.mask.substr(b*2,2),16); if(!v) continue;
    for(let k=0;k<8;k++) if(v>>k&1){ const i=b*8+k; cx.fillRect((i%d.gw)*s,Math.floor(i/d.gw)*s,s,s); }
  }
  cx.setLineDash([6,6]); cx.strokeStyle='rgba(255,255,255,0.7)'; cx.lineWidth=1;
  [d.y0,d.y1].forEach(y=>{ cx.beginPath(); cx.moveTo(0,y*s); cx.lineTo(320,y*s); cx.stroke(); });
  cx.setLineDash([]); cx.strokeStyle='#ff1744'; cx.lineWidth=4;
  if(d.fs){ const x=(d.far+1)/2*320; cx.beginPath(); cx.moveTo(x,d.y0*s); cx.lineTo(x,d.y1*s); cx.stroke(); }
  if(d.ns){ const x=(d.near+1)/2*320; cx.beginPath(); cx.moveTo(x,d.y1*s); cx.lineTo(x,240); cx.stroke(); }
}
async function pollLine(){
  try{
    const d=await (await fetch('/line')).json();
    if(firstLine){ firstLine=false; ['tol','sat'].forEach(k=>{ $(k).value=d[k]; $(k+'v').textContent=d[k]; }); }
    drawOverlay(d);
    let t;
    if(!d.set){ t='Tick "Tap to pick colour", then tap the tape'; $('big').style.color='#ffca28'; }
    else if(!d.seen){ t='NO LINE'; $('big').style.color='#ef5350'; }
    else { const a=Math.abs(d.steer); t='Line '+(a<0.1?'CENTRE':(d.steer<0?'LEFT ':'RIGHT ')+a.toFixed(2)); $('big').style.color='#66bb6a'; }
    $('big').textContent=t;
    const p=Math.max(-1,Math.min(1,d.seen?d.steer:0))*50, f=$('steerf');
    f.style.left=(p>=0?50:50+p)+'%'; f.style.width=Math.abs(p)+'%';
    $('info').textContent='near '+(d.ns?d.near.toFixed(2):'-')+' | far '+(d.fs?d.far.toFixed(2):'-')+
      ' | detection '+d.dfps.toFixed(1)+' fps ('+d.ms+' ms)'+(d.udp?' | sending to robot':'');
    // (d.udp = sending by wire and/or Wi-Fi)
    $('msg').textContent=d.msg;
    $('st').textContent='online'; $('st').style.background='#2e7d32';
  }catch(e){ $('st').textContent='offline'; $('st').style.background='#c62828'; }
}
async function pollStatus(){
  try{
    const d=await (await fetch('/status')).json();
    if(first){ first=false; $('quality').value=d.quality; $('qualityv').textContent=d.quality;
      $('vflip').checked=!!d.vflip; $('hmirror').checked=!!d.hmirror; }
    $('info2').textContent='video '+d.fps.toFixed(1)+' fps · '+d.kb+' kB/frame · battery '+d.bat.toFixed(2)+' V'+
      (d.rssi?' · signal '+d.rssi+' dBm':'');
  }catch(e){}
}
setInterval(pollLine,200); setInterval(pollStatus,1000); pollStatus(); setStream(true);
</script></body></html>
)rawliteral";

// ============================================================ Camera ========
bool cameraInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk  = XCLK_GPIO_NUM;
  c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href  = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_QVGA;            // 320x240: enough for a line, and fast
  c.jpeg_quality = START_QUALITY;
  c.fb_count     = 3;                         // video stream + line detection each hold one
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  if (!psramFound()) {
    Serial.println("No PSRAM found - set Tools -> PSRAM -> Enabled. Line detection needs it.");
    return false;
  }
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED (error 0x%x)\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  Serial.printf("Camera sensor found: PID 0x%04X (OV3660 = 0x3660)\n", s->id.PID);
  s->set_vflip(s, START_VFLIP);
  s->set_hmirror(s, START_HMIRROR);
  s->set_brightness(s, 1);
  s->set_saturation(s, 1);                    // a bit MORE colour helps tell tape from grey floor

  rgb = (uint8_t*)ps_malloc(IMG_W * IMG_H * 3);
  if (!rgb) { Serial.println("Out of PSRAM for the picture buffer"); return false; }
  return true;
}

float batteryVolts() {
  return analogReadMilliVolts(BAT_ADC_PIN) * 1.51f / 1000.0f;
}

void setMsg(const char* m) {
  strncpy(lastMsg, m, sizeof(lastMsg) - 1);
  Serial.println(m);
}

// ==================================================== Line detection ========
// Is this pixel tape-coloured? p points at 3 colour bytes.
// We compare the colour MIX (each part divided by the total), so the same
// tape in shadow or bright light still matches. The byte order of the
// decoder doesn't matter, because the picked colour is measured the same way.
bool isTape(const uint8_t* p) {
  int a = p[0], b = p[1], c = p[2];
  int sum = a + b + c;
  if (sum < MIN_BRIGHTNESS) return false;                      // too dark to judge
  int mx = max(a, max(b, c)), mn = min(a, min(b, c));
  if (mx - mn < minSat) return false;                          // greyish = floor
  float d = fabsf(a / (float)sum - target[0]) + fabsf(b / (float)sum - target[1])
          + fabsf(c / (float)sum - target[2]);
  return d < tolerance;
}

// Phone tapped the picture: measure the colour around that spot (5x5 average)
void pickColour() {
  pickRequested = false;
  int cx = constrain((int)(pickX * IMG_W), 2, IMG_W - 3);
  int cy = constrain((int)(pickY * IMG_H), 2, IMG_H - 3);
  long s0 = 0, s1 = 0, s2 = 0;
  for (int dy = -2; dy <= 2; dy++)
    for (int dx = -2; dx <= 2; dx++) {
      const uint8_t* p = rgb + ((cy + dy) * IMG_W + cx + dx) * 3;
      s0 += p[0]; s1 += p[1]; s2 += p[2];
    }
  int a = s0 / 25, b = s1 / 25, c = s2 / 25;
  int sum = a + b + c;
  int sat = max(a, max(b, c)) - min(a, min(b, c));
  if (sum < MIN_BRIGHTNESS || sat < minSat) {
    char m[64];
    snprintf(m, sizeof(m), "That spot is grey/dark (colour %d) - tap the tape", sat);
    setMsg(m);
    return;
  }
  target[0] = a / (float)sum; target[1] = b / (float)sum; target[2] = c / (float)sum;
  targetSet = true;
  prefs.putBool("set", true);
  prefs.putFloat("t0", target[0]); prefs.putFloat("t1", target[1]); prefs.putFloat("t2", target[2]);
  char m[64];
  snprintf(m, sizeof(m), "Colour picked (colour strength %d) and saved", sat);
  setMsg(m);
}

// Where is the tape in rows y0..y1-1? pos -1 = left edge, +1 = right edge
Band findInBand(int y0, int y1) {
  long sumX = 0;
  int n = 0;
  for (int y = y0; y < y1; y++)
    for (int x = 0; x < GW; x++)
      if (mask[y * GW + x]) { sumX += x; n++; }
  Band b;
  b.count = n;
  b.seen  = n >= MIN_BAND_PIXELS;
  b.pos   = b.seen ? ((sumX / (float)n) + 0.5f) / GW * 2.0f - 1.0f : 0.0f;
  return b;
}

void detectLine() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;
  uint32_t t0 = millis();
  bool ok = fb->width == IMG_W && fb->height == IMG_H && fmt2rgb888(fb->buf, fb->len, fb->format, rgb);
  esp_camera_fb_return(fb);                   // we have our own copy now
  if (!ok) { setMsg("Could not decode the picture"); return; }

  if (pickRequested) pickColour();

  // 1. Mark tape pixels on the 80x60 grid
  for (int gy = 0; gy < GH; gy++)
    for (int gx = 0; gx < GW; gx++) {
      const uint8_t* p = rgb + ((gy * STEP + STEP / 2) * IMG_W + gx * STEP + STEP / 2) * 3;
      rawMask[gy * GW + gx] = targetSet && isTape(p);
    }

  // 2. Remove lonely specks: keep a pixel only if 2+ of its 4 neighbours are tape too
  for (int gy = 0; gy < GH; gy++)
    for (int gx = 0; gx < GW; gx++) {
      int i = gy * GW + gx, n = 0;
      if (gx > 0      && rawMask[i - 1])  n++;
      if (gx < GW - 1 && rawMask[i + 1])  n++;
      if (gy > 0      && rawMask[i - GW]) n++;
      if (gy < GH - 1 && rawMask[i + GW]) n++;
      mask[i] = rawMask[i] && n >= 2;
    }

  // 3. Where is the line, close by and further ahead?
  Band farB  = findInBand(FAR_ROW0, NEAR_ROW0);
  Band nearB = findInBand(NEAR_ROW0, GH);
  bool wasSeen = lineSeen;
  nearSeen = nearB.seen; nearPos = nearB.pos;
  farSeen  = farB.seen;  farPos  = farB.pos;
  lineSeen = nearSeen || farSeen;
  if (nearSeen && farSeen) steerHint = NEAR_WEIGHT * nearPos + (1 - NEAR_WEIGHT) * farPos;
  else if (nearSeen)       steerHint = nearPos;
  else if (farSeen)        steerHint = farPos;
  else                     steerHint = 0;
  if (targetSet && wasSeen != lineSeen) Serial.println(lineSeen ? "Line found" : "Line LOST");

  // 4. Packed copy for the phone
  memset(maskBits, 0, sizeof(maskBits));
  for (int i = 0; i < GW * GH; i++) if (mask[i]) maskBits[i >> 3] |= 1 << (i & 7);

  detectMs = millis() - t0;
  detectCount++;
}

// ================================================ Line data to the robot ====
uint8_t xorSum(const char* s) {
  uint8_t cs = 0;
  while (*s) cs ^= (uint8_t)*s++;
  return cs;
}

// Same message over the wire (always, if USE_WIRE) and over Wi-Fi (if joined)
void sendLine() {
  char body[64];
  snprintf(body, sizeof(body), "L,%d,%.3f,%.3f,%.3f,%d,%d",
           lineSeen && targetSet, steerHint, nearPos, farPos, nearSeen, farSeen);
  if (USE_WIRE) Serial1.printf("$%s*%02X\n", body, xorSum(body));
  if (joinedRobot && WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(ROBOT_IP, LINE_UDP_PORT);
    udp.printf("$%s*%02X\n", body, xorSum(body));
    udp.endPacket();
  }
}

// ======================================================== Web handlers =====
#define PART_BOUNDARY "snakecamframe"
static const char* STREAM_TYPE     = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART     = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t streamHandler(httpd_req_t* req) {
  char part[64];
  esp_err_t res = httpd_resp_set_type(req, STREAM_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  streaming = true;
  Serial.println("Video started");
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    size_t hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)fb->len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    lastFrameBytes = fb->len;
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
    framesSent++;
  }
  streaming = false;
  Serial.println("Video stopped");
  return res;
}

bool queryValue(httpd_req_t* req, const char* key, char* out, size_t outLen) {
  char query[64];
  return httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
         httpd_query_key_value(query, key, out, outLen) == ESP_OK;
}

// /control?var=tol&val=12
static esp_err_t controlHandler(httpd_req_t* req) {
  char var[16], val[16];
  if (!queryValue(req, "var", var, sizeof(var)) || !queryValue(req, "val", val, sizeof(val))) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  int v = atoi(val);
  sensor_t* s = esp_camera_sensor_get();
  int r = 0;
  if      (!strcmp(var, "quality")) r = s->set_quality(s, constrain(v, 10, 63));
  else if (!strcmp(var, "vflip"))   r = s->set_vflip(s, v);
  else if (!strcmp(var, "hmirror")) r = s->set_hmirror(s, v);
  else if (!strcmp(var, "tol"))     { tolerance = constrain(v, 2, 40) / 100.0f; prefs.putFloat("tol", tolerance); }
  else if (!strcmp(var, "sat"))     { minSat = constrain(v, 0, 100); prefs.putInt("sat", minSat); }
  else { httpd_resp_send_404(req); return ESP_FAIL; }
  Serial.printf("Set %s = %d%s\n", var, v, r ? "  (camera refused!)" : "");
  return httpd_resp_send(req, r ? "FAIL" : "OK", HTTPD_RESP_USE_STRLEN);
}

// /pick?x=0.5&y=0.8  (0..1 across / down the picture)
static esp_err_t pickHandler(httpd_req_t* req) {
  char xs[12], ys[12];
  if (!queryValue(req, "x", xs, sizeof(xs)) || !queryValue(req, "y", ys, sizeof(ys))) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  pickX = constrain((float)atof(xs), 0.0f, 1.0f);
  pickY = constrain((float)atof(ys), 0.0f, 1.0f);
  pickRequested = true;                       // done by detectLine() on the next frame
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t lineHandler(httpd_req_t* req) {
  static char json[1800];
  const int S = sizeof(json);
  int n = snprintf(json, S,
      "{\"set\":%d,\"seen\":%d,\"ns\":%d,\"fs\":%d,\"near\":%.3f,\"far\":%.3f,\"steer\":%.3f,"
      "\"dfps\":%.1f,\"ms\":%lu,\"tol\":%d,\"sat\":%d,\"udp\":%d,\"gw\":%d,\"gh\":%d,\"y0\":%d,\"y1\":%d,"
      "\"msg\":\"%s\",\"mask\":\"",
      targetSet, lineSeen, nearSeen, farSeen, nearPos, farPos, steerHint,
      detectFps, (unsigned long)detectMs, (int)(tolerance * 100 + 0.5f), minSat,
      USE_WIRE || joinedRobot, GW, GH, FAR_ROW0, NEAR_ROW0, lastMsg);
  for (int i = 0; i < (int)sizeof(maskBits) && n < S - 4; i++)
    n += snprintf(json + n, S - n, "%02x", maskBits[i]);
  snprintf(json + n, S - n, "\"}");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t statusHandler(httpd_req_t* req) {
  sensor_t* s = esp_camera_sensor_get();
  char json[200];
  snprintf(json, sizeof(json),
           "{\"fps\":%.1f,\"kb\":%u,\"quality\":%d,\"vflip\":%d,\"hmirror\":%d,\"rssi\":%d,\"bat\":%.2f}",
           streaming ? streamFps : 0.0f, (unsigned)(lastFrameBytes / 1024), s->status.quality,
           s->status.vflip, s->status.hmirror, joinedRobot ? (int)WiFi.RSSI() : 0, batteryVolts());
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

void addPage(httpd_handle_t h, const char* uri, esp_err_t (*fn)(httpd_req_t*)) {
  httpd_uri_t u = {};
  u.uri = uri;
  u.method = HTTP_GET;
  u.handler = fn;
  u.user_ctx = NULL;
  httpd_register_uri_handler(h, &u);
}

void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port   = 32768;
  if (httpd_start(&webServer, &cfg) == ESP_OK) {
    addPage(webServer, "/",        indexHandler);
    addPage(webServer, "/control", controlHandler);
    addPage(webServer, "/pick",    pickHandler);
    addPage(webServer, "/line",    lineHandler);
    addPage(webServer, "/status",  statusHandler);
  } else Serial.println("Web server (port 80) failed to start");

  cfg.server_port = 81;
  cfg.ctrl_port   = 32769;
  if (httpd_start(&streamServer, &cfg) == ESP_OK) addPage(streamServer, "/stream", streamHandler);
  else Serial.println("Stream server (port 81) failed to start");
}

// ============================================================== Wi-Fi =======
void joinRobot() {
  WiFi.config(CAM_IP, ROBOT_IP, CAM_MASK);
  WiFi.begin(ROBOT_SSID, ROBOT_PASS);
}

void startWifi() {
  if (JOIN_ROBOT_WIFI) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    joinRobot();
    Serial.printf("Joining \"%s\"", ROBOT_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < JOIN_TIMEOUT_MS) { delay(250); Serial.print("."); }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED)
      Serial.printf("\"%s\" not found yet - is the sensor ESP32 on? I keep trying every %lu s\n",
                    ROBOT_SSID, (unsigned long)(JOIN_RETRY_MS / 1000));
    return;                                   // checkWifi() in loop() does the rest
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(OWN_AP_SSID, OWN_AP_PASS);
  WiFi.setSleep(false);
  Serial.printf("Join Wi-Fi \"%s\" (password %s) and open http://%s\n",
                OWN_AP_SSID, OWN_AP_PASS, WiFi.softAPIP().toString().c_str());
}

// Robot network: notice joining/losing it, and keep retrying while not joined
void checkWifi() {
  if (!CAMERA_WIFI || !JOIN_ROBOT_WIFI) return;
  static uint32_t lastTry = 0;
  bool up = WiFi.status() == WL_CONNECTED;
  if (up != joinedRobot) {
    joinedRobot = up;
    if (up) {
      WiFi.setSleep(false);                   // Wi-Fi power saving makes video stutter
      Serial.printf("Joined \"%s\". Open http://%s - sending line data to %s:%u\n", ROBOT_SSID,
                    WiFi.localIP().toString().c_str(), ROBOT_IP.toString().c_str(), LINE_UDP_PORT);
    } else {
      Serial.printf("Lost \"%s\" - trying to re-join\n", ROBOT_SSID);
    }
    lastTry = millis();
  }
  if (!up && millis() - lastTry >= JOIN_RETRY_MS) {
    lastTry = millis();
    Serial.printf("Trying to join \"%s\" again...\n", ROBOT_SSID);
    WiFi.disconnect();
    joinRobot();
  }
}

// Blue LED: robot network wanted but not joined = fast flicker, otherwise on = line seen
void updateLed() {
  bool on = (CAMERA_WIFI && JOIN_ROBOT_WIFI && !joinedRobot) ? (millis() % 200) < 100 : lineSeen;
  digitalWrite(LED_PIN, on);
}

// ===================================================== Setup / loop ========
void setup() {
  pinMode(BAT_HOLD_PIN, OUTPUT);
  digitalWrite(BAT_HOLD_PIN, HIGH);           // stay on when running from battery
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // M5Stack's advice for camera start-up
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(300);
  Serial.println("\nSnakeCam LINE starting");

  // Saved settings from last time
  prefs.begin("snakecam", false);
  targetSet = prefs.getBool("set", false);
  target[0] = prefs.getFloat("t0", 0.33f);
  target[1] = prefs.getFloat("t1", 0.33f);
  target[2] = prefs.getFloat("t2", 0.33f);
  tolerance = prefs.getFloat("tol", START_TOLERANCE);
  minSat    = prefs.getInt("sat", START_MIN_SAT);
  Serial.printf("Tape colour: %s | tolerance %.2f | min colour %d\n",
                targetSet ? "saved one loaded" : "NOT picked yet", tolerance, minSat);
  if (!targetSet) strcpy(lastMsg, "No colour yet - tick 'Tap to pick colour' and tap the tape");

  if (!cameraInit()) {
    Serial.println("Stopping here. Blue LED blinks = camera problem.");
    while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(150); }
  }
  if (USE_WIRE) {
    Serial1.begin(WIRE_BAUD, SERIAL_8N1, WIRE_RX_PIN, WIRE_TX_PIN);
    Serial.printf("Sending line data on the wire: G%d, %lu baud (remember GND!)\n",
                  WIRE_TX_PIN, (unsigned long)WIRE_BAUD);
  }
  if (CAMERA_WIFI) {
    startWifi();
    startServers();
  } else {
    WiFi.mode(WIFI_OFF);
    Serial.println("Wi-Fi is OFF (CAMERA_WIFI = false) - no tuning page, wire only");
  }
  Serial.println("Ready.");
}

void loop() {
  static uint32_t lastRate = 0, lastPrint = 0, lastSend = 0, lastFrames = 0, lastDetect = 0;

  detectLine();                               // as often as possible (~10 per second)
  checkWifi();
  updateLed();

  uint32_t now = millis();
  if (now - lastSend >= SEND_MS) { lastSend = now; sendLine(); }

  if (now - lastRate >= 1000) {
    float dt = (now - lastRate) / 1000.0f;
    streamFps = (framesSent - lastFrames) / dt;
    detectFps = (detectCount - lastDetect) / dt;
    lastFrames = framesSent;
    lastDetect = detectCount;
    lastRate = now;
  }
  if (now - lastPrint >= 2000) {
    lastPrint = now;
    if (!targetSet) Serial.println("No tape colour picked yet");
    else if (lineSeen) Serial.printf("Line: steer %+.2f (near %s, far %s) | %.1f fps, %lu ms\n", steerHint,
                                     nearSeen ? "yes" : "no", farSeen ? "yes" : "no", detectFps,
                                     (unsigned long)detectMs);
    else Serial.printf("No line | %.1f fps\n", detectFps);
  }
  delay(1);                                   // let the Wi-Fi/web tasks breathe
}
