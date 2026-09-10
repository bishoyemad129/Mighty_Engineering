/*
  Hydrophlax Wi-Fi pump controller -- ESP32-C3, Arduino-ESP32 3.x

  GPIO4 -> JZ-MOS PWM; 10k pull-down at module PWM -> common GND.
  GPIO5 -> 1N4148 anode; striped cathode -> Pololu OFF.
  Pololu VOUT -> 5V buck -> board 5V input. Common grounds.
  Button remains on Pololu A/B: it switches controller POWER, not software.
  Pump power does NOT pass through the small Pololu logic power switch.
  Verify module 3.3V input/PWM compatibility and flyback diode before testing.
  Firmware cannot hold GPIOs safe before boot: keep the hardware pull-down.
  PWM duty is a duty-cycle percentage only -- NOT a calibrated RPM, flow, or
  power reading.

  Every power-on runs ONE cycle: pump held off during STARTUP_DELAY_MS, then
  auto-starts at the last-saved fader position (100% on the very first-ever
  boot) for DEFAULT_RUNTIME_SECONDS. That fader position is stored in NVS
  flash (via the Preferences library) and survives a full power cycle; total
  run time never persists and always resets to DEFAULT_RUNTIME_SECONDS
  (5 minutes) on every power-on.

  The fader shown to the user runs 0-100%, but that is NOT the raw PWM duty:
  0% turns the pump off (same as STOP); 1% maps to the pump's practical
  minimum duty (PWM_MIN_PERCENT, 50%); 100% maps to full duty
  (PWM_MAX_PERCENT, 100%); values in between are linearly interpolated. See
  mapUserPercentToDuty(). Settings apply only to the current cycle's run
  time; Wi-Fi is optional -- the timed cycle runs the same whether or not
  anyone is connected, and closing the page never stops it.

  STOP pauses: output drops to 0% and the run timer freezes (setting the
  fader itself to 0% and applying does the same thing automatically).
  CONTINUE resumes at the selected speed for whatever time is left. Total run time is measured
  against accumulated RUNNING time only -- time spent paused is never counted
  down. Settings (PWM / total time) can be changed while running or paused;
  changing total time below the time already run completes the cycle
  immediately. POWER OFF stops
  the pump, ends the cycle, and triggers the existing Pololu shutdown pulse --
  it is not a pause and cannot be undone from the page.

  NOTE: this file has been reviewed for logic but has NOT been compiled or
  run on real hardware -- verify on your bench before relying on it.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "Install Arduino-ESP32 3.x: this sketch uses the pin-based LEDC API."
#endif
#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "Select an ESP32-C3 board, e.g. ESP32C3 Dev Module."
#endif

// ---------------- USER SETTINGS (unchanged from existing build) ----------------
const char* AP_NAME = "Pump Control";
const char* AP_PASSWORD = "WaterPump123"; // Change before regular use (8+ chars).
const uint8_t PUMP_PIN = 4;
const uint8_t SHUTDOWN_PIN = 5;
const uint32_t PWM_FREQUENCY_HZ = 1000; // Starting test value, not a verified pump spec.
const uint8_t PWM_BITS = 8;
const uint32_t DEFAULT_RUNTIME_SECONDS = 300; // Five minutes on every power-on.
const int DEFAULT_PWM_PERCENT = 100; // Fallback fader position, used on the very first-ever boot.
const int PWM_MIN_PERCENT = 50;  // Actual duty floor once running (fader's 1% maps here).
const int PWM_MAX_PERCENT = 100; // Actual duty ceiling (fader's 100% maps here).
const int USER_SPEED_MIN_PERCENT = 0;   // Fader floor shown to the user -- 0 means pump off.
const int USER_SPEED_MAX_PERCENT = 100; // Fader ceiling shown to the user.
const uint32_t STARTUP_DELAY_MS = 1000; // One second after initialization.
const uint32_t MAX_RUNTIME_SECONDS = 3600;
// Optional full-duty startup boost, counted inside the total run time and
// only applied once at the very start of the cycle (not on every resume).
const uint32_t START_BOOST_MS = 0; // e.g. 300 to enable.
// LED disabled because Mini boards differ (plain LED vs addressable RGB).
// Set to 8 ONLY for a confirmed ordinary GPIO8 LED, not an addressable LED.
const int STATUS_LED_PIN = -1;
const bool LED_ACTIVE_LOW = true;
// ---------------------------------------------------------------------------

enum State : uint8_t { STARTING, RUNNING, PAUSED, COMPLETED, SHUTTING_DOWN, FAULT };

WebServer server(80);
Preferences prefs;
const char* PREFS_NAMESPACE = "pump";
const char* PREFS_KEY_PERCENT = "percent";

State state = STARTING;
bool pwmReady = false;
bool wifiReady = false;
int lastSavedPercent = DEFAULT_PWM_PERCENT; // mirrors what's currently in NVS

uint32_t bootMs = 0;
uint32_t runSegmentStartMs = 0; // valid only while state == RUNNING
uint32_t accumulatedRunMs = 0;  // running time banked from earlier segments
uint32_t totalRuntimeMs = DEFAULT_RUNTIME_SECONDS * 1000UL;

uint32_t shutdownStartedAt = 0;
bool shutdownPulseDone = false;

int requestedPercent = DEFAULT_PWM_PERCENT; // user-facing fader value, 0-100 -- see mapUserPercentToDuty()
int appliedPercent = 0; // actual PWM duty currently being driven to the hardware

// ---------------------------------------------------------
// Timing helpers (overflow-safe: always subtract, never compare millis() directly)
// ---------------------------------------------------------
uint32_t elapsedRunningMs() {
  if (state == RUNNING) return accumulatedRunMs + (millis() - runSegmentStartMs);
  return accumulatedRunMs;
}

uint32_t remainingMs() {
  uint32_t elapsed = elapsedRunningMs();
  return (elapsed >= totalRuntimeMs) ? 0UL : (totalRuntimeMs - elapsed);
}

// ---------------------------------------------------------
// Output
// ---------------------------------------------------------
void setOutput(int percent) {
  percent = constrain(percent, 0, 100);
  if (!pwmReady) percent = 0;
  if (pwmReady) ledcWrite(PUMP_PIN, (uint32_t)((percent * 255UL + 50UL) / 100UL));
  else digitalWrite(PUMP_PIN, LOW);
  appliedPercent = percent;
  if (STATUS_LED_PIN >= 0) {
    digitalWrite(STATUS_LED_PIN, (percent > 0) != LED_ACTIVE_LOW ? HIGH : LOW);
  }
}

// Maps the user-facing 0-100% fader value to the actual PWM duty applied to
// the pump. 0 means off. 1-100 linearly spans PWM_MIN_PERCENT..PWM_MAX_PERCENT
// (50-100%) so the pump never runs below its practical minimum duty.
int mapUserPercentToDuty(int userPercent) {
  if (userPercent <= 0) return 0;
  userPercent = constrain(userPercent, 1, USER_SPEED_MAX_PERCENT);
  long span = (long)(PWM_MAX_PERCENT - PWM_MIN_PERCENT);          // 50
  long steps = (long)(USER_SPEED_MAX_PERCENT - 1);                // 99
  long numerator = (long)(userPercent - 1) * span;
  return PWM_MIN_PERCENT + (int)((numerator + steps / 2) / steps); // rounded to nearest
}

void applyOutputForState() {
  if (state != RUNNING) { setOutput(0); return; }
  uint32_t elapsed = elapsedRunningMs();
  int duty = (elapsed < START_BOOST_MS) ? PWM_MAX_PERCENT : mapUserPercentToDuty(requestedPercent);
  setOutput(duty);
}

// ---------------------------------------------------------
// Core state machine -- called every loop() and again at the top of every
// handler, so a stop/completion condition is never missed behind a request.
// ---------------------------------------------------------
void update() {
  const uint32_t now = millis();

  if (state == SHUTTING_DOWN) {
    setOutput(0);
    uint32_t elapsed = now - shutdownStartedAt;
    if (!shutdownPulseDone) {
      if (elapsed >= 1000UL) {
        digitalWrite(SHUTDOWN_PIN, LOW);
        shutdownPulseDone = true;
      } else if (elapsed >= 500UL) {
        digitalWrite(SHUTDOWN_PIN, HIGH);
      }
    }
    return;
  }

  if (state == STARTING) {
    setOutput(0);
    if (now - bootMs >= STARTUP_DELAY_MS) {
      if (!pwmReady) { state = FAULT; return; }
      state = RUNNING;
      runSegmentStartMs = now;
      accumulatedRunMs = 0;
    }
    return;
  }

  if (state == RUNNING) {
    uint32_t elapsed = elapsedRunningMs();
    if (elapsed >= totalRuntimeMs) {
      accumulatedRunMs = totalRuntimeMs;
      state = COMPLETED;
      setOutput(0);
      return;
    }
    if (requestedPercent <= 0) {
      // 0% on the fader means "pump off" -- treat it as an automatic pause.
      accumulatedRunMs = elapsed;
      state = PAUSED;
      setOutput(0);
      return;
    }
    applyOutputForState();
    return;
  }

  // PAUSED, COMPLETED, FAULT: pump stays off.
  setOutput(0);
}

// ---------------------------------------------------------
// Request helpers
// ---------------------------------------------------------
bool readNumber(const char* name, uint32_t low, uint32_t high, uint32_t &value) {
  if (!server.hasArg(name)) return false;
  String s = server.arg(name);
  if (s.length() == 0 || s.length() > 6) return false;
  value = 0;
  for (size_t i = 0; i < s.length(); ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    value = value * 10 + (s[i] - '0');
  }
  return value >= low && value <= high;
}

void reply(int code, const char* message) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "text/plain", message);
}

// ---------------------------------------------------------
// Dashboard (single self-contained page, no external assets)
// ---------------------------------------------------------
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pump Speed Control</title><style>
:root{color-scheme:dark}
*{box-sizing:border-box}
body{font:17px/1.4 system-ui,-apple-system,sans-serif;background:#101c29;color:#edf7ff;margin:0;padding:20px}
main{max-width:480px;margin:0 auto}
h1{margin:4px 0 2px;font-size:1.6em}
.muted{color:#9fb7cc;font-size:14px}
header{display:flex;justify-content:space-between;align-items:baseline;flex-wrap:wrap;gap:6px}
#conn{font-size:13px;padding:4px 10px;border-radius:20px;background:#2b3f52}
#conn.ok{color:#7be0a0}
#conn.bad{color:#ff9b93;background:#3a2530}
section{background:#1b2d40;border-radius:18px;padding:22px;margin-top:16px}
#state{font-size:22px;font-weight:700}
#state.running{color:#5be7c4}
#state.paused{color:#ffcf6b}
#state.completed{color:#9fb7cc}
#state.shutting{color:#ff9b93}
#state.fault{color:#ff6b6b}
#remaining{font-size:44px;font-weight:700;margin:6px 0;letter-spacing:1px}
label{display:block;margin:20px 0 8px;font-weight:600}
input[type=number]{width:100%;padding:14px;border-radius:10px;border:1px solid #33475c;background:#0f1b28;color:#edf7ff;font:inherit;min-height:48px}
input[type=range]{width:100%;accent-color:#36cbd6;height:32px}
.row{display:flex;gap:12px}
.row>div{flex:1}
.btnrow{display:flex;flex-wrap:wrap;gap:10px;margin-top:18px}
button{font:inherit;font-weight:700;padding:16px;border:0;border-radius:12px;cursor:pointer;flex:1;min-height:52px;min-width:120px}
button:disabled{opacity:.35;cursor:not-allowed}
#apply{background:#36cbd6;color:#00232a}
#stop{background:#ffcf6b;color:#2a2000}
#cont{background:#5be7c4;color:#00291f}
#msg{min-height:22px;margin-top:12px;font-size:14px}
#msg.err{color:#ff9b93}
#msg.ok{color:#7be0a0}
.hint{margin-top:16px}
</style></head><body><main>
<header><h1>Pump Speed Control</h1><span id="conn">Connecting&hellip;</span></header>

<section>
  <div id="state">--</div>
  <div id="remaining">--:--</div>
</section>

<section>
  <label for="speed">Set Speed: <span id="value">--</span>%</label>
  <input id="speed" type="range" min="0" max="100" value="100">

  <label>Total run time</label>
  <div class="row">
    <div>
      <label for="minutes" class="muted">Minutes</label>
      <input id="minutes" type="number" inputmode="numeric" min="0" max="60" value="5">
    </div>
    <div>
      <label for="seconds" class="muted">Seconds</label>
      <input id="seconds" type="number" inputmode="numeric" min="0" max="59" value="0">
    </div>
  </div>

  <div class="btnrow"><button id="apply">APPLY SETTINGS</button></div>
  <div class="btnrow">
    <button id="stop">STOP</button>
    <button id="cont">CONTINUE</button>
  </div>

  <div id="msg" role="status" aria-live="polite"></div>


</main>
<script>
const $ = id => document.getElementById(id);
let dirty = false, editingCount = 0, commandBusy = false, pollBusy = false;

function setMsg(text, ok) {
  $('msg').textContent = text;
  $('msg').className = ok ? 'ok' : 'err';
}

async function post(path, args) {
  const r = await fetch(path, {method:'POST', body:new URLSearchParams(args||{}), signal:AbortSignal.timeout(2500)});
  const text = await r.text();
  if (!r.ok) throw new Error(text || 'Request failed');
  return text;
}

async function action(path, args, clearDirty) {
  if (commandBusy) return;
  commandBusy = true;
  try {
    const msg = await post(path, args);
    setMsg(msg, true);
    if (clearDirty) dirty = false;
  } catch (e) {
    setMsg(e.message, false);
  } finally {
    commandBusy = false;
    poll();
  }
}

$('speed').addEventListener('input', () => { dirty = true; $('value').textContent = $('speed').value; });
for (const id of ['speed', 'minutes', 'seconds']) {
  $(id).addEventListener('focus', () => { editingCount++; });
  $(id).addEventListener('blur', () => { editingCount--; });
  $(id).addEventListener('input', () => { dirty = true; });
}

$('apply').addEventListener('click', () => action('/settings', {
  percent: $('speed').value, minutes: $('minutes').value, seconds: $('seconds').value
}, true));
$('stop').addEventListener('click', () => action('/stop'));
$('cont').addEventListener('click', () => action('/continue'));

const STATE_CLASS = {Starting:'', Running:'running', Paused:'paused', Completed:'completed', 'Shutting down':'shutting', Fault:'fault'};

function render(s) {
  const el = $('state');
  el.textContent = s.state;
  el.className = STATE_CLASS[s.state] || '';

  const m = Math.floor(s.remainingSeconds / 60), sec = s.remainingSeconds % 60;
  $('remaining').textContent = String(m).padStart(2,'0') + ':' + String(sec).padStart(2,'0');

  if (!dirty && editingCount === 0 && !commandBusy) {
    $('speed').value = s.percentRequested;
    $('value').textContent = s.percentRequested;
    $('minutes').value = s.totalMinutes;
    $('seconds').value = s.totalSeconds;
  }

  $('apply').disabled = !s.canApply || commandBusy;
  $('stop').disabled = !s.canStop || commandBusy;
  $('cont').disabled = !s.canContinue || commandBusy;
  $('stop').style.display = s.state === 'Paused' ? 'none' : '';
  $('cont').style.display = s.state === 'Paused' ? '' : 'none';
}

async function poll() {
  if (pollBusy) return;
  pollBusy = true;
  try {
    const r = await fetch('/status', {cache:'no-store', signal:AbortSignal.timeout(2500)});
    if (!r.ok) throw new Error();
    const s = await r.json();
    $('conn').textContent = 'Connected';
    $('conn').className = 'ok';
    render(s);
  } catch (e) {
    $('conn').textContent = 'Disconnected';
    $('conn').className = 'bad';
    $('apply').disabled = $('stop').disabled = $('cont').disabled = true;
  } finally {
    pollBusy = false;
  }
}
setInterval(poll, 500);
poll();
</script>
</body></html>
)HTML";

// ---------------------------------------------------------
// Setup
// ---------------------------------------------------------
void setup() {
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW);
  if (STATUS_LED_PIN >= 0) pinMode(STATUS_LED_PIN, OUTPUT);

  pwmReady = ledcAttach(PUMP_PIN, PWM_FREQUENCY_HZ, PWM_BITS);
  setOutput(0);

  prefs.begin(PREFS_NAMESPACE, false); // NVS-backed: survives a full power cycle
  int savedPercent = prefs.getInt(PREFS_KEY_PERCENT, DEFAULT_PWM_PERCENT);
  savedPercent = constrain(savedPercent, USER_SPEED_MIN_PERCENT, USER_SPEED_MAX_PERCENT); // guard stale/out-of-range data
  requestedPercent = savedPercent;
  lastSavedPercent = savedPercent;

  totalRuntimeMs = DEFAULT_RUNTIME_SECONDS * 1000UL; // run time never persists
  accumulatedRunMs = 0;

  Serial.begin(115200); // Never wait for USB/Serial connection.

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  wifiReady = WiFi.softAP(AP_NAME, AP_PASSWORD);
  if (!wifiReady) Serial.println("Wi-Fi unavailable; timed pump cycle still operates.");

  server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html", PAGE); });

  server.on("/settings", HTTP_POST, [](){
    update();
    if (!(state == RUNNING || state == PAUSED)) {
      reply(409, "Settings can only be applied while running or paused");
      return;
    }
    uint32_t percent, minutes, seconds;
    if (!readNumber("percent", (uint32_t)USER_SPEED_MIN_PERCENT, (uint32_t)USER_SPEED_MAX_PERCENT, percent)) {
      reply(400, "Speed must be a whole number 0-100");
      return;
    }
    if (!readNumber("minutes", 0, MAX_RUNTIME_SECONDS / 60UL, minutes)) { reply(400, "Minutes must be 0-60"); return; }
    if (!readNumber("seconds", 0, 59, seconds)) { reply(400, "Seconds must be 0-59"); return; }

    uint32_t totalSeconds = minutes * 60UL + seconds;
    if (totalSeconds < 1 || totalSeconds > MAX_RUNTIME_SECONDS) {
      reply(400, "Total run time must be between 1 second and 60 minutes");
      return;
    }

    requestedPercent = (int)percent;
    if (requestedPercent != lastSavedPercent) {
      prefs.putInt(PREFS_KEY_PERCENT, requestedPercent);
      lastSavedPercent = requestedPercent;
    }
    uint32_t newTotalMs = totalSeconds * 1000UL;
    uint32_t elapsed = elapsedRunningMs();

    if (newTotalMs <= elapsed) {
      accumulatedRunMs = elapsed;
      totalRuntimeMs = newTotalMs;
      state = COMPLETED;
      setOutput(0);
      reply(200, "New run time already elapsed - cycle completed");
      return;
    }

    totalRuntimeMs = newTotalMs;
    update(); // re-evaluate immediately against the newly applied settings
    reply(200, "Settings applied");
  });

  server.on("/stop", HTTP_POST, [](){
    update();
    if (state == RUNNING) {
      accumulatedRunMs = elapsedRunningMs();
      state = PAUSED;
      setOutput(0);
      reply(200, "Paused");
    } else if (state == PAUSED) {
      reply(200, "Already paused");
    } else {
      reply(409, "Nothing to stop right now");
    }
  });

  server.on("/continue", HTTP_POST, [](){
    update();
    if (state == PAUSED) {
      if (requestedPercent <= 0) { reply(409, "Set a speed above 0% before continuing"); return; }
      if (remainingMs() == 0) { reply(409, "No time remaining"); return; }
      runSegmentStartMs = millis();
      state = RUNNING;
      applyOutputForState();
      reply(200, "Resumed");
    } else if (state == RUNNING) {
      reply(200, "Already running");
    } else {
      reply(409, "Cannot continue right now");
    }
  });

  server.on("/poweroff", HTTP_POST, [](){
    if (state != SHUTTING_DOWN) {
      accumulatedRunMs = elapsedRunningMs();
      state = SHUTTING_DOWN;
      shutdownStartedAt = millis();
      shutdownPulseDone = false;
      digitalWrite(SHUTDOWN_PIN, LOW);
      setOutput(0);
      reply(200, "Power-off requested");
    } else {
      reply(200, "Power-off already in progress");
    }
  });

  server.on("/status", HTTP_GET, [](){
    update();
    uint32_t remSec = (remainingMs() + 999UL) / 1000UL;
    uint32_t totSec = totalRuntimeMs / 1000UL;
    uint32_t totMin = totSec / 60UL;
    uint32_t totSecRem = totSec % 60UL;

    const char* stateName =
      state == STARTING ? "Starting" :
      state == RUNNING ? "Running" :
      state == PAUSED ? "Paused" :
      state == COMPLETED ? "Completed" :
      state == SHUTTING_DOWN ? "Shutting down" : "Fault";

    bool canApply = (state == RUNNING || state == PAUSED);
    bool canStop = (state == RUNNING);
    bool canContinue = (state == PAUSED && requestedPercent > 0 && remainingMs() > 0);
    bool canPowerOff = (state != SHUTTING_DOWN);

    String json = "{";
    json += "\"state\":\""; json += stateName; json += "\",";
    json += "\"ready\":"; json += (pwmReady ? "true" : "false"); json += ",";
    json += "\"percentRequested\":"; json += String(requestedPercent); json += ",";
    json += "\"percentApplied\":"; json += String(appliedPercent); json += ",";
    json += "\"totalMinutes\":"; json += String(totMin); json += ",";
    json += "\"totalSeconds\":"; json += String(totSecRem); json += ",";
    json += "\"remainingSeconds\":"; json += String(remSec); json += ",";
    json += "\"canApply\":"; json += (canApply ? "true" : "false"); json += ",";
    json += "\"canStop\":"; json += (canStop ? "true" : "false"); json += ",";
    json += "\"canContinue\":"; json += (canContinue ? "true" : "false"); json += ",";
    json += "\"canPowerOff\":"; json += (canPowerOff ? "true" : "false");
    json += "}";

    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
  });

  server.onNotFound([](){ reply(404, "Not found"); });
  if (wifiReady) server.begin();

  bootMs = millis();
  state = STARTING;
  Serial.println("Automatic cycle armed. Settings: http://192.168.4.1");
}

// ---------------------------------------------------------
// Main loop -- non-blocking. update() runs before AND after HTTP handling so
// a stop/timer condition is never delayed behind a pending request.
// ---------------------------------------------------------
void loop() {
  update();
  if (wifiReady) server.handleClient();
  update();
  delay(1);
}
