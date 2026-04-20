#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <chrono>
#include <atomic>
#include <functional>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cstring>
#include <csignal>
#include <libwebsockets.h>
#include <gpiod.h>

#define CHIP             "/dev/gpiochip0"
#define DT_PIN           5
#define SCK_PIN          11
#define SOLENOID_PIN     18
#define HTTP_PORT        8080
#define RAW_TRIGGER      3000

#define RATE_PER_MIN     0.10f
#define MIN_CHARGE       1.0f

class Solenoid {
public:
    Solenoid() : chip(nullptr), req(nullptr), ready(false), locked(false) {}

    bool init() {
        chip = gpiod_chip_open(CHIP);
        if (!chip) { std::cerr << "ERROR: Cannot open GPIO chip for solenoid\n"; return false; }
        unsigned int off = SOLENOID_PIN;
        struct gpiod_line_settings* cfg = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(cfg, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(cfg, GPIOD_LINE_VALUE_INACTIVE);
        struct gpiod_line_config* lc = gpiod_line_config_new();
        gpiod_line_config_add_line_settings(lc, &off, 1, cfg);
        struct gpiod_request_config* rc = gpiod_request_config_new();
        gpiod_request_config_set_consumer(rc, "solenoid");
        req = gpiod_chip_request_lines(chip, rc, lc);
        gpiod_line_config_free(lc);
        gpiod_line_settings_free(cfg);
        gpiod_request_config_free(rc);
        if (!req) { std::cerr << "ERROR: Cannot request solenoid GPIO line\n"; gpiod_chip_close(chip); chip = nullptr; return false; }
        ready = true;
        unlock();
        std::cout << "Solenoid initialised OK — UNLOCKED\n";
        return true;
    }

    void lock() {
        if (!ready) return;
        gpiod_line_request_set_value(req, SOLENOID_PIN, GPIOD_LINE_VALUE_ACTIVE);
        locked = true;
        std::cout << "Solenoid: LOCKED\n";
    }

    void unlock() {
        if (!ready) return;
        gpiod_line_request_set_value(req, SOLENOID_PIN, GPIOD_LINE_VALUE_INACTIVE);
        locked = false;
        std::cout << "Solenoid: UNLOCKED\n";
    }

    void shutdown() {
        unlock();
        if (req)  { gpiod_line_request_release(req);  req  = nullptr; }
        if (chip) { gpiod_chip_close(chip);            chip = nullptr; }
        ready = false;
    }

private:
    struct gpiod_chip*         chip;
    struct gpiod_line_request* req;
    bool                       ready;
    bool                       locked;
};

class WeightSensor {
public:
    using Callback = std::function<void(long)>;
    WeightSensor() : running(false), rawValue(0), zeroOffset(-489500), chip(nullptr), req(nullptr), ready(false) {}

    bool init() {
        chip = gpiod_chip_open(CHIP);
        if (!chip) { std::cerr << "ERROR: Cannot open GPIO chip for HX711\n"; return false; }
        unsigned int dt_off = DT_PIN, sck_off = SCK_PIN;
        struct gpiod_line_settings* dt_cfg = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(dt_cfg, GPIOD_LINE_DIRECTION_INPUT);
        struct gpiod_line_settings* sck_cfg = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(sck_cfg, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(sck_cfg, GPIOD_LINE_VALUE_INACTIVE);
        struct gpiod_line_config* lc = gpiod_line_config_new();
        gpiod_line_config_add_line_settings(lc, &dt_off, 1, dt_cfg);
        gpiod_line_config_add_line_settings(lc, &sck_off, 1, sck_cfg);
        struct gpiod_request_config* rc = gpiod_request_config_new();
        gpiod_request_config_set_consumer(rc, "hx711");
        req = gpiod_chip_request_lines(chip, rc, lc);
        gpiod_line_config_free(lc);
        gpiod_line_settings_free(dt_cfg);
        gpiod_line_settings_free(sck_cfg);
        gpiod_request_config_free(rc);
        if (!req) { std::cerr << "ERROR: Cannot request HX711 GPIO lines\n"; gpiod_chip_close(chip); chip = nullptr; return false; }
        ready = true;
        std::cout << "HX711 GPIO initialised OK\n";
        return true;
    }

    void calibrateZero() {
        std::cout << "Calibrating zero — keep scale empty...\n";
        long sum = 0; int count = 0;
        for (int i = 0; i < 20; i++) {
            long r = readOnce();
            if (r == -1 || r > -100000) continue;
            sum += r; count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (count > 0) { zeroOffset = sum / count; std::cout << "Zero offset: " << zeroOffset << "\n"; }
        else { std::cout << "Calibration failed, using default: " << zeroOffset << "\n"; }
    }

    void setCallback(Callback cb) { onChange = cb; }
    void start() { running = true; worker = std::thread(&WeightSensor::loop, this); }
    void stop() {
        running = false;
        if (worker.joinable()) worker.join();
        if (req)  { gpiod_line_request_release(req);  req  = nullptr; }
        if (chip) { gpiod_chip_close(chip);            chip = nullptr; }
    }
    long getRaw()  { std::lock_guard<std::mutex> l(mtx); return rawValue; }
    long getZero() { return zeroOffset; }

private:
    std::atomic<bool> running;
    long rawValue, zeroOffset;
    std::mutex mtx;
    std::thread worker;
    Callback onChange;
    struct gpiod_chip* chip;
    struct gpiod_line_request* req;
    bool ready;

    void loop() {
        while (running) {
            long r = ready ? readOnce() : 0;
            if (r == -1 || r > -100000) continue;
            { std::lock_guard<std::mutex> l(mtx); rawValue = r; }
            if (onChange) onChange(r);
        }
    }

    long readOnce() {
        int timeout = 0;
        while (gpiod_line_request_get_value(req, DT_PIN) != GPIOD_LINE_VALUE_INACTIVE) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            if (++timeout > 5000) return -1;
        }
        long raw = 0;
        for (int i = 0; i < 24; i++) {
            gpiod_line_request_set_value(req, SCK_PIN, GPIOD_LINE_VALUE_ACTIVE);
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            int bit = (gpiod_line_request_get_value(req, DT_PIN) == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;
            raw = (raw << 1) | bit;
            gpiod_line_request_set_value(req, SCK_PIN, GPIOD_LINE_VALUE_INACTIVE);
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        gpiod_line_request_set_value(req, SCK_PIN, GPIOD_LINE_VALUE_ACTIVE);
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        gpiod_line_request_set_value(req, SCK_PIN, GPIOD_LINE_VALUE_INACTIVE);
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        if (raw & 0x800000) raw |= ~0xFFFFFF;
        return raw;
    }
};

enum class State { IDLE, SET_PASSWORD, RUNNING, UNLOCK_PIN, ENDED };

class LockerController {
public:
    LockerController(WeightSensor& s, Solenoid& sol)
        : sensor(s), solenoid(sol), state(State::IDLE),
          rentAmount(0.0f), elapsedSecs(0),
          startTimeStr("--"), finalRent(""), pinMsg(""), itemDetectCount(0) {
        sensor.setCallback([this](long r){ onRaw(r); });
    }

    bool setPin(const std::string& pin) {
        std::lock_guard<std::mutex> l(mtx);
        if (state != State::SET_PASSWORD) return false;
        if (pin.size() != 4) { pinMsg = "PIN must be 4 digits"; return false; }
        for (char c : pin) if (c < '0' || c > '9') { pinMsg = "Digits only"; return false; }
        storedPin = pin; state = State::RUNNING;
        sessionStart = std::chrono::steady_clock::now();
        startTimeStr = currentTime();
        rentAmount = MIN_CHARGE; elapsedSecs = 0; finalRent = ""; pinMsg = "";
        solenoid.lock();
        std::cout << "PIN set. Session started: " << startTimeStr << "\n";
        return true;
    }

    bool verifyPin(const std::string& pin) {
        std::lock_guard<std::mutex> l(mtx);
        if (state != State::UNLOCK_PIN) return false;
        if (pin == storedPin) {
            auto now = std::chrono::steady_clock::now();
            elapsedSecs = std::chrono::duration_cast<std::chrono::seconds>(now - sessionStart).count();
            float minutes = elapsedSecs / 60.0f;
            rentAmount = minutes * RATE_PER_MIN;
            if (rentAmount < MIN_CHARGE) rentAmount = MIN_CHARGE;
            std::ostringstream r;
            r << std::fixed << std::setprecision(2) << rentAmount;
            finalRent = r.str(); storedPin = ""; state = State::ENDED; pinMsg = "";
            solenoid.unlock();
            std::cout << "PIN correct. Session ended. Duration=" << elapsedSecs << "s  Rent=£" << rentAmount << "\n";
            return true;
        } else {
            pinMsg = "Wrong PIN — try again";
            return false;
        }
    }

    void requestUnlock() {
        std::lock_guard<std::mutex> l(mtx);
        if (state == State::RUNNING) { state = State::UNLOCK_PIN; pinMsg = ""; }
    }

    void resetSession() {
        std::lock_guard<std::mutex> l(mtx);
        if (state == State::ENDED) { state = State::IDLE; finalRent = ""; pinMsg = ""; elapsedSecs = 0; }
    }

    std::string getJSON() {
        std::lock_guard<std::mutex> l(mtx);
        if (state == State::RUNNING) {
            auto now = std::chrono::steady_clock::now();
            elapsedSecs = std::chrono::duration_cast<std::chrono::seconds>(now - sessionStart).count();
            float minutes = elapsedSecs / 60.0f;
            rentAmount = minutes * RATE_PER_MIN;
            if (rentAmount < MIN_CHARGE) rentAmount = MIN_CHARGE;
        }
        long long h = elapsedSecs / 3600, m = (elapsedSecs % 3600) / 60, s = elapsedSecs % 60;
        std::ostringstream timer;
        timer << std::setfill('0') << std::setw(2) << h << ":"
              << std::setfill('0') << std::setw(2) << m << ":"
              << std::setfill('0') << std::setw(2) << s;
        std::string stateStr, statusMsg;
        switch (state) {
            case State::IDLE:         stateStr="idle";       statusMsg="Locker open — place an item inside"; break;
            case State::SET_PASSWORD: stateStr="set_pin";    statusMsg="Item detected — set a 4-digit PIN"; break;
            case State::RUNNING:      stateStr="running";    statusMsg="Item secured — timer running"; break;
            case State::UNLOCK_PIN:   stateStr="unlock_pin"; statusMsg="Enter PIN to retrieve your item"; break;
            case State::ENDED:        stateStr="ended";      statusMsg="Session ended — thank you"; break;
        }
        std::ostringstream o;
        o << std::fixed << std::setprecision(2);
        o << "{\"state\":\""        << stateStr    << "\""
          << ",\"status\":\""       << statusMsg   << "\""
          << ",\"rent\":"           << ((state==State::RUNNING) ? rentAmount : 0.0f)
          << ",\"elapsed\":\""      << ((state==State::RUNNING) ? timer.str() : "00:00:00") << "\""
          << ",\"started\":\""      << ((state==State::RUNNING) ? startTimeStr : "--") << "\""
          << ",\"finalRent\":\""    << finalRent   << "\""
          << ",\"finalElapsed\":\"" << timer.str() << "\""
          << ",\"pinMsg\":\""       << pinMsg      << "\""
          << "}";
        return o.str();
    }

private:
    WeightSensor& sensor;
    Solenoid&     solenoid;
    std::mutex    mtx;
    State         state;
    float         rentAmount;
    long long     elapsedSecs;
    std::string   startTimeStr, finalRent, storedPin, pinMsg;
    int           itemDetectCount;
    std::chrono::steady_clock::time_point sessionStart;

    std::string currentTime() {
        auto t = std::time(nullptr); auto tm = *std::localtime(&t);
        std::ostringstream o;
        o << std::setfill('0') << std::setw(2) << tm.tm_hour << ":"
          << std::setfill('0') << std::setw(2) << tm.tm_min  << ":"
          << std::setfill('0') << std::setw(2) << tm.tm_sec;
        return o.str();
    }

    void onRaw(long r) {
        std::lock_guard<std::mutex> l(mtx);
        long diff = sensor.getZero() - r;
        bool hasItem = (diff > RAW_TRIGGER);
        if (state == State::IDLE) {
            if (hasItem) { itemDetectCount++; if (itemDetectCount >= 10) { state = State::SET_PASSWORD; itemDetectCount = 0; std::cout << "Item detected\n"; } }
            else { itemDetectCount = 0; }
        }
    }
};

static LockerController* g_locker   = nullptr;
static Solenoid*         g_solenoid = nullptr;

void onShutdown(int) {
    std::cout << "\nShutting down — unlocking solenoid...\n";
    if (g_solenoid) g_solenoid->unlock();
    exit(0);
}

static const char DASHBOARD_HTML[] = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Locker</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@300;500;700&display=swap');
  *{margin:0;padding:0;box-sizing:border-box}
  :root{--green:#00ff9d;--red:#ff4d6d;--yellow:#ffe66d;--blue:#4da6ff;--bg:#0a0e17;--card:#111827;--border:#1f2d40}
  body{background:var(--bg);color:#e0e0e0;font-family:'Rajdhani',sans-serif;min-height:100vh;display:flex;flex-direction:column;align-items:center;justify-content:center;padding:2rem}
  h1{font-size:1.8rem;letter-spacing:.3em;text-transform:uppercase;color:var(--green);margin-bottom:2rem;font-weight:300}
  .grid{display:grid;grid-template-columns:1fr;gap:1.4rem;width:100%;max-width:440px}
  .card{background:var(--card);border:1px solid var(--border);border-radius:16px;padding:1.8rem;position:relative;overflow:hidden;transition:all .4s}
  .card::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:var(--green);transition:background .4s}
  .card.red::before{background:var(--red)}
  .card.yellow::before{background:var(--yellow)}
  .card.blue::before{background:var(--blue)}
  .label{font-size:.72rem;letter-spacing:.25em;text-transform:uppercase;color:#445;margin-bottom:.7rem}
  .timer-val{font-family:'Share Tech Mono',monospace;font-size:3.8rem;color:var(--red);line-height:1;letter-spacing:.08em}
  .rent-val{font-family:'Share Tech Mono',monospace;font-size:3.2rem;color:var(--yellow);line-height:1}
  .sub{font-size:.92rem;color:#667;margin-top:.5rem}
  .status-text{font-size:1.3rem;font-weight:500;letter-spacing:.04em}
  .dot{display:inline-block;width:11px;height:11px;border-radius:50%;background:var(--green);margin-right:.55rem;animation:pulse 2s infinite;vertical-align:middle}
  .dot.red{background:var(--red);animation:none}
  .dot.yellow{background:var(--yellow);animation:none}
  .dot.blue{background:var(--blue)}
  @keyframes pulse{0%,100%{opacity:1;transform:scale(1)}50%{opacity:.4;transform:scale(1.4)}}
  .btn{width:100%;padding:1rem;border:none;border-radius:12px;font-family:'Rajdhani',sans-serif;font-size:1.15rem;font-weight:700;letter-spacing:.12em;text-transform:uppercase;cursor:pointer;transition:all .2s;margin-top:.3rem}
  .btn:active{transform:scale(.97)}
  .btn:disabled{opacity:.3;cursor:not-allowed;transform:none}
  .btn-green{background:var(--green);color:#0a0e17}
  .btn-red{background:var(--red);color:#fff}
  .btn-blue{background:var(--blue);color:#0a0e17}
  .pin-wrap{display:flex;gap:.6rem;justify-content:center;margin:1rem 0}
  .pin-box{width:54px;height:64px;background:#0d1421;border:2px solid var(--border);border-radius:10px;font-family:'Share Tech Mono',monospace;font-size:2rem;color:var(--green);text-align:center;outline:none;transition:border .2s}
  .pin-box:focus{border-color:var(--green)}
  .pin-box.filled{border-color:var(--blue)}
  .pin-msg{text-align:center;font-size:.9rem;color:var(--red);min-height:1.2rem;letter-spacing:.05em}
  .conn{position:fixed;top:1rem;right:1.2rem;font-size:.7rem;letter-spacing:.15em;text-transform:uppercase;color:#334}
  .conn.ok{color:var(--green)}
  .final-box{background:#1a1a2e;border:2px solid var(--yellow);border-radius:16px;padding:2rem;text-align:center}
  .final-amount{font-family:'Share Tech Mono',monospace;font-size:4.5rem;color:var(--yellow);margin:.5rem 0}
  .final-label{font-size:.8rem;letter-spacing:.2em;text-transform:uppercase;color:#667}
  .final-sub{font-size:.95rem;color:#556;margin-top:.4rem}
  .history{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:1.2rem;max-height:100px;overflow-y:auto}
  .history .label{margin-bottom:.5rem}
  .log-line{font-family:'Share Tech Mono',monospace;font-size:.74rem;color:#556;margin-bottom:.3rem;border-bottom:1px solid var(--border);padding-bottom:.3rem}
  .log-line span{color:#889}
  .rate-note{text-align:center;font-size:.76rem;color:#334;letter-spacing:.1em}
  .screen{display:none}
  .screen.show{display:contents}
</style>
</head>
<body>
<h1>Smart Locker</h1>
<div class="grid">

  <div class="card" id="status-card">
    <div class="label">Status</div>
    <div style="padding:.3rem 0">
      <span class="dot" id="dot"></span>
      <span class="status-text" id="status-val">Loading...</span>
    </div>
  </div>

  <div class="screen" id="screen-idle">
    <div class="card blue">
      <div class="label">Instruction</div>
      <div style="font-size:1.1rem;color:#aaa;line-height:1.6">
        Place your item inside the locker.<br>The door will prompt you to set a PIN.
      </div>
    </div>
  </div>

  <div class="screen" id="screen-set-pin">
    <div class="card yellow">
      <div class="label">Set your 4-digit PIN</div>
      <div class="pin-wrap">
        <input class="pin-box" id="sp1" maxlength="1" type="password" inputmode="numeric" oninput="pinInput('sp',1)">
        <input class="pin-box" id="sp2" maxlength="1" type="password" inputmode="numeric" oninput="pinInput('sp',2)">
        <input class="pin-box" id="sp3" maxlength="1" type="password" inputmode="numeric" oninput="pinInput('sp',3)">
        <input class="pin-box" id="sp4" maxlength="1" type="password" inputmode="numeric" oninput="pinInput('sp',4)">
      </div>
      <div class="pin-msg" id="sp-msg"></div>
      <button class="btn btn-red" onclick="submitSetPin()">Lock &amp; Start Timer</button>
    </div>
  </div>

  <div class="screen" id="screen-running">
    <div class="card red" style="margin-bottom:.2rem">
      <div class="label">Duration (HH : MM : SS)</div>
      <div class="timer-val" id="elapsed-val">00:00:00</div>
      <div class="sub" id="started-val">--</div>
    </div>
    <div class="card red">
      <div class="label">Rent due</div>
      <div class="rent-val" id="rent-val">£1.00</div>
      <div class="sub">£0.10 per minute</div>
    </div>
    <button class="btn btn-green" onclick="requestUnlock()">Retrieve My Item</button>
  </div>

  <div class="screen" id="screen-unlock-pin">
    <div class="card red">
      <div class="label">Enter your PIN to unlock</div>
      <div class="pin-wrap">
        <input class="pin-box" id="up1" maxlength="1" type="password" inputmode="numeric" oninput="pinInput('up',1)">
        <input class="pin-box" id="up2" maxlength="1" type="password" inputmode="numeric" oninput="pinInput('up',2)">
        <input class="pin-box" id="up3" maxlength="1" type="password" inputmode="numeric" oninput="pinInput('up',3)">
        <input class="pin-box" id="up4" maxlength="1" type="password" inputmode="numeric" oninput="pinInput('up',4)">
      </div>
      <div class="pin-msg" id="up-msg"></div>
      <button class="btn btn-green" onclick="submitUnlockPin()">Confirm &amp; Unlock</button>
    </div>
  </div>

  <div class="screen" id="screen-ended">
    <div class="final-box">
      <div class="final-label">Total amount due</div>
      <div class="final-amount" id="final-amount">£0.00</div>
      <div class="final-sub" id="final-duration">--</div>
    </div>
    <button class="btn btn-blue" onclick="resetSession()">New Session</button>
  </div>

  <div class="history">
    <div class="label">Event log</div>
    <div id="log"></div>
  </div>
  <div class="rate-note">Rate: £0.10 per minute &nbsp;|&nbsp; Min £1.00</div>
</div>
<div class="conn" id="conn">Connecting...</div>

<script>
var lastStatus='';
function addLog(msg){
  var d=document.getElementById('log');
  var t=new Date().toLocaleTimeString();
  d.innerHTML='<div class="log-line"><span>'+t+'</span> — '+msg+'</div>'+d.innerHTML;
  if(d.children.length>30)d.removeChild(d.lastChild);
}
function showScreen(name){
  ['idle','set-pin','running','unlock-pin','ended'].forEach(function(s){
    document.getElementById('screen-'+s).className='screen'+(s===name?' show':'');
  });
}
function pinInput(prefix,idx){
  var box=document.getElementById(prefix+idx);
  if(box.value.length===1&&idx<4)document.getElementById(prefix+(idx+1)).focus();
  box.className=box.value?'pin-box filled':'pin-box';
}
function getPin(prefix){
  return ['1','2','3','4'].map(function(i){return document.getElementById(prefix+i).value;}).join('');
}
function clearPin(prefix){
  ['1','2','3','4'].forEach(function(i){var b=document.getElementById(prefix+i);b.value='';b.className='pin-box';});
}
function submitSetPin(){
  var pin=getPin('sp');
  if(pin.length!==4){document.getElementById('sp-msg').textContent='Enter all 4 digits';return;}
  fetch('/setpin',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'pin='+pin})
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){clearPin('sp');document.getElementById('sp-msg').textContent='';}
      else{document.getElementById('sp-msg').textContent=d.msg||'Error';}
    });
}
function requestUnlock(){fetch('/requestunlock',{method:'POST'});}
function submitUnlockPin(){
  var pin=getPin('up');
  if(pin.length!==4){document.getElementById('up-msg').textContent='Enter all 4 digits';return;}
  fetch('/verifypin',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'pin='+pin})
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){clearPin('up');document.getElementById('up-msg').textContent='';}
      else{document.getElementById('up-msg').textContent=d.msg||'Wrong PIN';clearPin('up');document.getElementById('up1').focus();}
    });
}
function resetSession(){fetch('/reset',{method:'POST'});}
function update(){
  fetch('/data')
    .then(function(r){return r.json();})
    .then(function(d){
      document.getElementById('conn').textContent='Live';
      document.getElementById('conn').className='conn ok';
      var dot=document.getElementById('dot');
      var sc=document.getElementById('status-card');
      document.getElementById('status-val').textContent=d.status;
      if(d.state==='idle'){dot.className='dot blue';sc.className='card blue';showScreen('idle');}
      else if(d.state==='set_pin'){dot.className='dot yellow';sc.className='card yellow';showScreen('set-pin');if(d.pinMsg)document.getElementById('sp-msg').textContent=d.pinMsg;}
      else if(d.state==='running'){
        dot.className='dot red';sc.className='card red';showScreen('running');
        document.getElementById('elapsed-val').textContent=d.elapsed;
        document.getElementById('rent-val').textContent='£'+d.rent.toFixed(2);
        document.getElementById('started-val').textContent='Since '+d.started;
      }
      else if(d.state==='unlock_pin'){dot.className='dot red';sc.className='card red';showScreen('unlock-pin');if(d.pinMsg)document.getElementById('up-msg').textContent=d.pinMsg;}
      else if(d.state==='ended'){
        dot.className='dot yellow';sc.className='card yellow';showScreen('ended');
        document.getElementById('final-amount').textContent='£'+d.finalRent;
        document.getElementById('final-duration').textContent='Duration: '+d.finalElapsed;
      }
      if(d.status!==lastStatus){addLog(d.status);lastStatus=d.status;}
    })
    .catch(function(){document.getElementById('conn').textContent='Reconnecting...';document.getElementById('conn').className='conn';});
}
setInterval(update,300);
update();
</script>
</body>
</html>
)HTML";

static std::string parsePostParam(const std::string& body, const std::string& key) {
    std::string search = key + "=";
    auto pos = body.find(search);
    if (pos == std::string::npos) return "";
    pos += search.size();
    auto end = body.find('&', pos);
    return body.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
}

struct PerSessionData { char body[64]; int bodyLen; };

static int callback_http(struct lws* wsi, enum lws_callback_reasons reason,
                         void* user, void* in, size_t len) {
    auto* psd = (PerSessionData*)user;
    switch (reason) {
    case LWS_CALLBACK_HTTP: {
        char* url = (char*)in;
        if (url && (strncmp(url,"/setpin",7)==0||strncmp(url,"/verifypin",10)==0||
                    strncmp(url,"/requestunlock",14)==0||strncmp(url,"/reset",6)==0)) {
            memset(psd->body,0,sizeof(psd->body));
            if(strncmp(url,"/setpin",    7)==0) psd->body[0]='S';
            if(strncmp(url,"/verifypin",10)==0) psd->body[0]='V';
            if(strncmp(url,"/requestunlock",14)==0) psd->body[0]='U';
            if(strncmp(url,"/reset",     6)==0) psd->body[0]='R';
            psd->bodyLen=0; return 0;
        }
        unsigned char headers[512]; unsigned char* p=headers; unsigned char* end=headers+sizeof(headers);
        if(url&&strncmp(url,"/data",5)==0){
            std::string json=g_locker?g_locker->getJSON()
                :"{\"state\":\"idle\",\"status\":\"offline\",\"rent\":0,\"elapsed\":\"00:00:00\",\"started\":\"--\",\"finalRent\":\"\",\"finalElapsed\":\"\",\"pinMsg\":\"\"}";
            lws_add_http_common_headers(wsi,HTTP_STATUS_OK,"application/json",(lws_filepos_t)json.size(),&p,end);
            lws_add_http_header_by_name(wsi,(unsigned char*)"Access-Control-Allow-Origin:",(unsigned char*)"*",1,&p,end);
            lws_finalize_write_http_header(wsi,headers,&p,end);
            lws_write(wsi,(unsigned char*)json.c_str(),json.size(),LWS_WRITE_HTTP_FINAL);
        } else {
            lws_add_http_common_headers(wsi,HTTP_STATUS_OK,"text/html",(lws_filepos_t)strlen(DASHBOARD_HTML),&p,end);
            lws_finalize_write_http_header(wsi,headers,&p,end);
            lws_write(wsi,(unsigned char*)DASHBOARD_HTML,strlen(DASHBOARD_HTML),LWS_WRITE_HTTP_FINAL);
        }
        return 1;
    }
    case LWS_CALLBACK_HTTP_BODY: {
        int copy=std::min((int)len,(int)(sizeof(psd->body)-psd->bodyLen-1));
        memcpy(psd->body+1+psd->bodyLen,(char*)in,copy);
        psd->bodyLen+=copy; return 0;
    }
    case LWS_CALLBACK_HTTP_BODY_COMPLETION: {
        psd->body[1+psd->bodyLen]='\0';
        std::string body(psd->body+1,psd->bodyLen);
        std::string resp="{\"ok\":true}";
        if(psd->body[0]=='S'&&g_locker){std::string pin=parsePostParam(body,"pin");if(!g_locker->setPin(pin))resp="{\"ok\":false,\"msg\":\"PIN must be 4 digits\"}";}
        else if(psd->body[0]=='V'&&g_locker){std::string pin=parsePostParam(body,"pin");if(!g_locker->verifyPin(pin))resp="{\"ok\":false,\"msg\":\"Wrong PIN — try again\"}";}
        else if(psd->body[0]=='U'&&g_locker){g_locker->requestUnlock();}
        else if(psd->body[0]=='R'&&g_locker){g_locker->resetSession();}
        unsigned char headers[256]; unsigned char* p=headers; unsigned char* end=headers+sizeof(headers);
        lws_add_http_common_headers(wsi,HTTP_STATUS_OK,"application/json",(lws_filepos_t)resp.size(),&p,end);
        lws_finalize_write_http_header(wsi,headers,&p,end);
        lws_write(wsi,(unsigned char*)resp.c_str(),resp.size(),LWS_WRITE_HTTP_FINAL);
        return 1;
    }
    default: break;
    }
    return 0;
}

static struct lws_protocols protocols[] = {
    { "http", callback_http, sizeof(PerSessionData), 0, 0, nullptr, 0 },
    { nullptr, nullptr, 0, 0, 0, nullptr, 0 }
};

int main() {
    signal(SIGINT,  onShutdown);
    signal(SIGTERM, onShutdown);

    Solenoid     solenoid;
    WeightSensor sensor;

    if (!solenoid.init()) std::cerr << "WARNING: Solenoid not initialised\n";
    if (!sensor.init())   std::cerr << "WARNING: HX711 not initialised\n";

    sensor.calibrateZero();
    g_solenoid = &solenoid;

    LockerController locker(sensor, solenoid);
    g_locker = &locker;

    struct lws_context_creation_info info{};
    info.port      = HTTP_PORT;
    info.protocols = protocols;

    struct lws_context* ctx = lws_create_context(&info);
    if (!ctx) {
        std::cerr << "Failed to start server on port " << HTTP_PORT << "\n";
        solenoid.shutdown();
        return 1;
    }

    sensor.start();

    std::cout << "=========================================\n";
    std::cogit push origin main --forceut << "Smart Locker — PIN Protected\n";
    std::cout << "Rate : £0.10 per minute\n";
    std::cout << "Min  : £" << MIN_CHARGE << "\n";
    std::cout << "Open : http://<Pi-IP>:" << HTTP_PORT << "\n";
    std::cout << "Ctrl+C to stop and unlock\n";
    std::cout << "=========================================\n";

    while (true) lws_service(ctx, 10);

    sensor.stop();
    solenoid.shutdown();
    lws_context_destroy(ctx);
    return 0;
}