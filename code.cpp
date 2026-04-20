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


// SOLENOID

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

        if (!req) {
            std::cerr << "ERROR: Cannot request solenoid GPIO line\n";
            gpiod_chip_close(chip); chip = nullptr;
            return false;
        }

        ready = true;
        // Start locked
        lock();
        std::cout << "Solenoid initialised OK — LOCKED\n";
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

    bool isLocked() { return locked; }

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


// WEIGHT SENSOR

class WeightSensor {
public:
    using Callback = std::function<void(long)>;

    WeightSensor() : running(false), rawValue(0),
                     zeroOffset(-489500),
                     chip(nullptr), req(nullptr), ready(false) {}

    bool init() {
        chip = gpiod_chip_open(CHIP);
        if (!chip) { std::cerr << "ERROR: Cannot open GPIO chip for HX711\n"; return false; }

        unsigned int dt_off  = DT_PIN;
        unsigned int sck_off = SCK_PIN;

        struct gpiod_line_settings* dt_cfg = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(dt_cfg, GPIOD_LINE_DIRECTION_INPUT);

        struct gpiod_line_settings* sck_cfg = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(sck_cfg, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(sck_cfg, GPIOD_LINE_VALUE_INACTIVE);

        struct gpiod_line_config* lc = gpiod_line_config_new();
        gpiod_line_config_add_line_settings(lc, &dt_off,  1, dt_cfg);
        gpiod_line_config_add_line_settings(lc, &sck_off, 1, sck_cfg);

        struct gpiod_request_config* rc = gpiod_request_config_new();
        gpiod_request_config_set_consumer(rc, "hx711");

        req = gpiod_chip_request_lines(chip, rc, lc);

        gpiod_line_config_free(lc);
        gpiod_line_settings_free(dt_cfg);
        gpiod_line_settings_free(sck_cfg);
        gpiod_request_config_free(rc);

        if (!req) {
            std::cerr << "ERROR: Cannot request HX711 GPIO lines\n";
            gpiod_chip_close(chip); chip = nullptr;
            return false;
        }

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
        if (count > 0) {
            zeroOffset = sum / count;
            std::cout << "Zero offset: " << zeroOffset << "\n";
        } else {
            std::cout << "Calibration failed, using default: " << zeroOffset << "\n";
        }
    }

    void setCallback(Callback cb) { onChange = cb; }
    void start() { running = true; worker = std::thread(&WeightSensor::loop, this); }
    void stop()  {
        running = false;
        if (worker.joinable()) worker.join();
        if (req)  { gpiod_line_request_release(req);  req  = nullptr; }
        if (chip) { gpiod_chip_close(chip);            chip = nullptr; }
    }

    long getRaw()  { std::lock_guard<std::mutex> l(mtx); return rawValue; }
    long getZero() { return zeroOffset; }

private:
    std::atomic<bool>          running;
    long                       rawValue;
    long                       zeroOffset;
    std::mutex                 mtx;
    std::thread                worker;
    Callback                   onChange;
    struct gpiod_chip*         chip;
    struct gpiod_line_request* req;
    bool                       ready;

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
            int bit = (gpiod_line_request_get_value(req, DT_PIN)
                       == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;
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


// LOCKER CONTROLLER
// States:
//   IDLE       — locker closed, waiting for user to press Open
//   OPEN       — locker unlocked, waiting for item to be placed
//   ITEM_IN    — item detected, waiting for user to press Close
//   RUNNING    — locker locked, timer and rent running
//   ENDED      — session ended, showing final bill

enum class State { IDLE, OPEN, ITEM_IN, RUNNING, ENDED };

class LockerController {
public:
    LockerController(WeightSensor& s, Solenoid& sol)
        : sensor(s), solenoid(sol),
          state(State::IDLE),
          rentAmount(0.0f), elapsedSecs(0),
          startTimeStr("--"), finalRent(""),
          itemDetectCount(0) {
        sensor.setCallback([this](long r){ onRaw(r); });
    }

    // Called from HTTP — button press from browser
    void pressOpen() {
        std::lock_guard<std::mutex> l(mtx);
        if (state == State::IDLE || state == State::ENDED) {
            state = State::OPEN;
            solenoid.unlock();
            finalRent = "";
            std::cout << "User pressed OPEN\n";
        }
    }

    void pressClose() {
        std::lock_guard<std::mutex> l(mtx);
        if (state == State::ITEM_IN) {
            state        = State::RUNNING;
            sessionStart = std::chrono::steady_clock::now();
            startTimeStr = currentTime();
            rentAmount   = MIN_CHARGE;
            elapsedSecs  = 0;
            solenoid.lock();
            std::cout << "User pressed CLOSE — session started: " << startTimeStr << "\n";
        }
    }

    std::string getJSON() {
        std::lock_guard<std::mutex> l(mtx);

        // Update timer and rent while running
        if (state == State::RUNNING) {
            auto now    = std::chrono::steady_clock::now();
            elapsedSecs = std::chrono::duration_cast<std::chrono::seconds>(
                              now - sessionStart).count();
            float minutes = elapsedSecs / 60.0f;
            rentAmount    = minutes * RATE_PER_MIN;
            if (rentAmount < MIN_CHARGE) rentAmount = MIN_CHARGE;
        }

        long long h = elapsedSecs / 3600;
        long long m = (elapsedSecs % 3600) / 60;
        long long s = elapsedSecs % 60;
        std::ostringstream timer;
        timer << std::setfill('0') << std::setw(2) << h << ":"
              << std::setfill('0') << std::setw(2) << m << ":"
              << std::setfill('0') << std::setw(2) << s;

        std::string stateStr;
        std::string statusMsg;
        switch (state) {
            case State::IDLE:
                stateStr  = "idle";
                statusMsg = "Locker closed";
                break;
            case State::OPEN:
                stateStr  = "open";
                statusMsg = "Locker open — place an item inside";
                break;
            case State::ITEM_IN:
                stateStr  = "item_in";
                statusMsg = "Item detected — press Close Locker";
                break;
            case State::RUNNING:
                stateStr  = "running";
                statusMsg = "Item secured — timer running";
                break;
            case State::ENDED:
                stateStr  = "ended";
                statusMsg = "Session ended";
                break;
        }

        std::ostringstream o;
        o << std::fixed << std::setprecision(2);
        o << "{\"state\":\""     << stateStr    << "\""
          << ",\"status\":\""    << statusMsg   << "\""
          << ",\"rent\":"        << ((state == State::RUNNING) ? rentAmount : 0.0f)
          << ",\"elapsed\":\""   << ((state == State::RUNNING) ? timer.str() : "00:00:00") << "\""
          << ",\"started\":\""   << ((state == State::RUNNING) ? startTimeStr : "--") << "\""
          << ",\"finalRent\":\""  << finalRent   << "\""
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
    std::string   startTimeStr;
    std::string   finalRent;
    int           itemDetectCount;
    std::chrono::steady_clock::time_point sessionStart;

    std::string currentTime() {
        auto t  = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream o;
        o << std::setfill('0')
          << std::setw(2) << tm.tm_hour << ":"
          << std::setw(2) << tm.tm_min  << ":"
          << std::setw(2) << tm.tm_sec;
        return o.str();
    }

    void onRaw(long r) {
        std::lock_guard<std::mutex> l(mtx);

        long diff = sensor.getZero() - r;
        bool hasItem = (diff > RAW_TRIGGER);

        if (state == State::OPEN) {
            // Waiting for item to be placed
            if (hasItem) {
                itemDetectCount++;
                if (itemDetectCount >= 10) {
                    state           = State::ITEM_IN;
                    itemDetectCount = 0;
                    std::cout << "Item detected — diff=" << diff << "\n";
                }
            } else {
                itemDetectCount = 0;
            }

        } else if (state == State::RUNNING) {
            // Session running — watch for item removal (shouldn't happen but handle it)
            if (!hasItem) {
                // Item removed while locked — end session
                auto now      = std::chrono::steady_clock::now();
                elapsedSecs   = std::chrono::duration_cast<std::chrono::seconds>(
                                    now - sessionStart).count();
                float minutes = elapsedSecs / 60.0f;
                rentAmount    = minutes * RATE_PER_MIN;
                if (rentAmount < MIN_CHARGE) rentAmount = MIN_CHARGE;
                std::ostringstream r;
                r << std::fixed << std::setprecision(2) << rentAmount;
                finalRent = r.str();
                state     = State::ENDED;
                solenoid.unlock();
                std::cout << "Item removed. Duration=" << elapsedSecs
                          << "s  Rent=£" << rentAmount << "\n";
            }
        }
    }
};


// GLOBALS

static LockerController* g_locker   = nullptr;
static Solenoid*         g_solenoid = nullptr;

void onShutdown(int) {
    std::cout << "\nShutting down — unlocking solenoid...\n";
    if (g_solenoid) g_solenoid->unlock();
    exit(0);
}


// HTML DASHBOARD

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
  .grid{display:grid;grid-template-columns:1fr;gap:1.4rem;width:100%;max-width:460px}
  .card{background:var(--card);border:1px solid var(--border);border-radius:16px;padding:1.8rem;position:relative;overflow:hidden;transition:all .4s}
  .card::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:var(--green);transition:background .4s}
  .card.red::before{background:var(--red)}
  .card.yellow::before{background:var(--yellow)}
  .card.blue::before{background:var(--blue)}
  .card.inactive{opacity:.3}
  .label{font-size:.72rem;letter-spacing:.25em;text-transform:uppercase;color:#445;margin-bottom:.7rem}
  .big{font-family:'Share Tech Mono',monospace;font-size:3.2rem;color:var(--green);line-height:1}
  .big.red{color:var(--red)}
  .big.money{color:var(--yellow)}
  .sub{font-size:.95rem;color:#667;margin-top:.4rem}
  .status-text{font-size:1.35rem;font-weight:500;letter-spacing:.04em}
  .dot{display:inline-block;width:11px;height:11px;border-radius:50%;background:var(--green);margin-right:.55rem;animation:pulse 2s infinite;vertical-align:middle}
  .dot.red{background:var(--red);animation:none}
  .dot.yellow{background:var(--yellow);animation:none}
  .dot.blue{background:var(--blue)}
  @keyframes pulse{0%,100%{opacity:1;transform:scale(1)}50%{opacity:.4;transform:scale(1.4)}}
  .row{display:grid;grid-template-columns:1fr 1fr;gap:1.4rem}
  .btn{width:100%;padding:1.1rem;border:none;border-radius:12px;font-family:'Rajdhani',sans-serif;font-size:1.2rem;font-weight:700;letter-spacing:.15em;text-transform:uppercase;cursor:pointer;transition:all .2s}
  .btn:active{transform:scale(.97)}
  .btn-open{background:var(--green);color:#0a0e17}
  .btn-open:hover{background:#00cc80}
  .btn-close{background:var(--red);color:#fff}
  .btn-close:hover{background:#cc3355}
  .btn:disabled{opacity:.3;cursor:not-allowed;transform:none}
  .conn{position:fixed;top:1rem;right:1.2rem;font-size:.7rem;letter-spacing:.15em;text-transform:uppercase;color:#334}
  .conn.ok{color:var(--green)}
  .final-box{background:#1a1a2e;border:2px solid var(--yellow);border-radius:16px;padding:2rem;text-align:center;display:none}
  .final-box.show{display:block}
  .final-amount{font-family:'Share Tech Mono',monospace;font-size:4.5rem;color:var(--yellow);margin:.5rem 0}
  .final-label{font-size:.8rem;letter-spacing:.2em;text-transform:uppercase;color:#667}
  .final-dur{font-size:1rem;color:#556;margin-top:.5rem}
  .history{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:1.4rem;max-height:110px;overflow-y:auto}
  .history .label{margin-bottom:.6rem}
  .log-line{font-family:'Share Tech Mono',monospace;font-size:.75rem;color:#556;margin-bottom:.3rem;border-bottom:1px solid var(--border);padding-bottom:.3rem}
  .log-line span{color:#889}
  .rate-note{text-align:center;font-size:.78rem;color:#334;letter-spacing:.1em;padding:.4rem 0}
</style>
</head>
<body>
<h1>Smart Locker</h1>
<div class="grid">

  <!-- Status -->
  <div class="card" id="status-card">
    <div class="label">Locker status</div>
    <div style="padding:.4rem 0">
      <span class="dot" id="dot"></span>
      <span class="status-text" id="status-val">Locker closed</span>
    </div>
  </div>

  <!-- Open button — shown when idle or ended -->
  <button class="btn btn-open" id="btn-open" onclick="pressOpen()">
    Open Locker
  </button>

  <!-- Close button — shown when item is detected -->
  <button class="btn btn-close" id="btn-close" onclick="pressClose()" disabled style="display:none">
    Close Locker
  </button>

  <!-- Timer + Rent — shown when running -->
  <div class="row" id="session-row" style="display:none">
    <div class="card red" id="timer-card">
      <div class="label">Duration</div>
      <div class="big red" id="elapsed-val">00:00:00</div>
      <div class="sub" id="started-val">--</div>
    </div>
    <div class="card red" id="rent-card">
      <div class="label">Rent due</div>
      <div class="big money" id="rent-val">£1.00</div>
      <div class="sub">£0.10 per min</div>
    </div>
  </div>

  <!-- Final bill -->
  <div class="final-box" id="final-box">
    <div class="final-label">Total amount due</div>
    <div class="final-amount" id="final-amount">£0.00</div>
    <div class="final-dur" id="final-duration">--</div>
  </div>

  <!-- Log -->
  <div class="history">
    <div class="label">Event log</div>
    <div id="log"></div>
  </div>

  <div class="rate-note">Rate: £0.10 per minute &nbsp;|&nbsp; Min charge: £1.00</div>
</div>
<div class="conn" id="conn">Connecting...</div>

<script>
var lastStatus = '';
var lastElapsed = '00:00:00';

function addLog(msg) {
  var d = document.getElementById('log');
  var t = new Date().toLocaleTimeString();
  d.innerHTML = '<div class="log-line"><span>' + t + '</span> — ' + msg + '</div>' + d.innerHTML;
  if (d.children.length > 30) d.removeChild(d.lastChild);
}

function pressOpen() {
  fetch('/open', { method: 'POST' });
}

function pressClose() {
  fetch('/close', { method: 'POST' });
}

function update() {
  fetch('/data')
    .then(function(r) { return r.json(); })
    .then(function(d) {
      document.getElementById('conn').textContent = 'Live';
      document.getElementById('conn').className = 'conn ok';

      var dot        = document.getElementById('dot');
      var sc         = document.getElementById('status-card');
      var btnOpen    = document.getElementById('btn-open');
      var btnClose   = document.getElementById('btn-close');
      var sessionRow = document.getElementById('session-row');
      var finalBox   = document.getElementById('final-box');

      document.getElementById('status-val').textContent = d.status;

      // Reset all
      btnOpen.style.display  = 'none';
      btnClose.style.display = 'none';
      btnClose.disabled      = true;
      sessionRow.style.display = 'none';
      finalBox.className       = 'final-box';

      if (d.state === 'idle') {
        dot.className    = 'dot';
        sc.className     = 'card';
        btnOpen.style.display = 'block';
        btnOpen.disabled = false;

      } else if (d.state === 'open') {
        dot.className    = 'dot blue';
        sc.className     = 'card blue';
        btnOpen.style.display = 'block';
        btnOpen.disabled = true;

      } else if (d.state === 'item_in') {
        dot.className         = 'dot yellow';
        sc.className          = 'card yellow';
        btnClose.style.display = 'block';
        btnClose.disabled      = false;

      } else if (d.state === 'running') {
        dot.className            = 'dot red';
        sc.className             = 'card red';
        sessionRow.style.display = 'grid';
        document.getElementById('elapsed-val').textContent = d.elapsed;
        document.getElementById('rent-val').textContent    = '£' + d.rent.toFixed(2);
        document.getElementById('started-val').textContent = 'Since ' + d.started;
        lastElapsed = d.elapsed;

      } else if (d.state === 'ended') {
        dot.className         = 'dot yellow';
        sc.className          = 'card yellow';
        finalBox.className    = 'final-box show';
        document.getElementById('final-amount').textContent  = '£' + d.finalRent;
        document.getElementById('final-duration').textContent = 'Duration: ' + lastElapsed;
        btnOpen.style.display = 'block';
        btnOpen.disabled      = false;
        btnOpen.textContent   = 'New Session';
      }

      if (d.status !== lastStatus) { addLog(d.status); lastStatus = d.status; }
    })
    .catch(function() {
      document.getElementById('conn').textContent = 'Reconnecting...';
      document.getElementById('conn').className = 'conn';
    });
}

setInterval(update, 300);
update();
</script>
</body>
</html>
)HTML";

// ===========================
// HTTP CALLBACK
// Handles: GET /        → dashboard HTML
//          GET /data    → JSON state
//          POST /open   → user pressed Open
//          POST /close  → user pressed Close
// ===========================
static int callback_http(struct lws* wsi, enum lws_callback_reasons reason,
                         void*, void* in, size_t len) {
    if (reason != LWS_CALLBACK_HTTP) return 0;

    char* url = (char*)in;

    if (url && strncmp(url, "/open", 5) == 0) {
        if (g_locker) g_locker->pressOpen();
        const char* resp = "OK";
        unsigned char headers[256];
        unsigned char* p   = headers;
        unsigned char* end = headers + sizeof(headers);
        lws_add_http_common_headers(wsi, HTTP_STATUS_OK,
            "text/plain", 2, &p, end);
        lws_finalize_write_http_header(wsi, headers, &p, end);
        lws_write(wsi, (unsigned char*)resp, 2, LWS_WRITE_HTTP_FINAL);
        return 1;
    }

    if (url && strncmp(url, "/close", 6) == 0) {
        if (g_locker) g_locker->pressClose();
        const char* resp = "OK";
        unsigned char headers[256];
        unsigned char* p   = headers;
        unsigned char* end = headers + sizeof(headers);
        lws_add_http_common_headers(wsi, HTTP_STATUS_OK,
            "text/plain", 2, &p, end);
        lws_finalize_write_http_header(wsi, headers, &p, end);
        lws_write(wsi, (unsigned char*)resp, 2, LWS_WRITE_HTTP_FINAL);
        return 1;
    }

    unsigned char headers[512];
    unsigned char* p   = headers;
    unsigned char* end = headers + sizeof(headers);

    if (url && strncmp(url, "/data", 5) == 0) {
        std::string json = g_locker
            ? g_locker->getJSON()
            : "{\"state\":\"idle\",\"status\":\"offline\",\"rent\":0,\"elapsed\":\"00:00:00\",\"started\":\"--\",\"finalRent\":\"\"}";
        if (lws_add_http_common_headers(wsi, HTTP_STATUS_OK,
                "application/json", (lws_filepos_t)json.size(), &p, end)) return 1;
        if (lws_add_http_header_by_name(wsi,
                (unsigned char*)"Access-Control-Allow-Origin:",
                (unsigned char*)"*", 1, &p, end)) return 1;
        if (lws_finalize_write_http_header(wsi, headers, &p, end)) return 1;
        lws_write(wsi, (unsigned char*)json.c_str(), json.size(), LWS_WRITE_HTTP_FINAL);
    } else {
        if (lws_add_http_common_headers(wsi, HTTP_STATUS_OK,
                "text/html", (lws_filepos_t)strlen(DASHBOARD_HTML), &p, end)) return 1;
        if (lws_finalize_write_http_header(wsi, headers, &p, end)) return 1;
        lws_write(wsi, (unsigned char*)DASHBOARD_HTML,
                  strlen(DASHBOARD_HTML), LWS_WRITE_HTTP_FINAL);
    }
    return 1;
}

static struct lws_protocols protocols[] = {
    { "http", callback_http, 0, 0, 0, nullptr, 0 },
    { nullptr, nullptr,      0, 0, 0, nullptr, 0 }
};

// ===========================
// MAIN
// ===========================
int main() {
    signal(SIGINT,  onShutdown);
    signal(SIGTERM, onShutdown);

    Solenoid     solenoid;
    WeightSensor sensor;

    if (!solenoid.init())
        std::cerr << "WARNING: Solenoid not initialised\n";
    if (!sensor.init())
        std::cerr << "WARNING: HX711 not initialised\n";

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
    std::cout << "Smart Locker\n";
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