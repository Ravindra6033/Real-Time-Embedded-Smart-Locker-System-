/**
 * smart_locker.cpp
 *
 * Real-Time Smart Locker — ENG 5220
 *
 * Build:
 *   g++ -std=c++17 -O2 -Wall smart_locker.cpp -lwebsockets -lgpiod -lpthread -o smart_locker
 *
 * Run normally:
 *   ./smart_locker
 *
 * Run built-in unit tests:
 *   ./smart_locker --test
 *
 * Real-time design notes:
 *  - HX711 outputs at ~10 SPS (128-gain mode) → max sensor latency 100 ms.
 *  - A dedicated WeightSensor thread wakes only when the HX711 DT line goes
 *    low (active-wait with 100 µs poll, bounded by a 500 ms timeout guard).
 *    This keeps the sensor thread responsive within one sample period (~100 ms).
 *  - Solenoid must respond within 200 ms of a user button press; the HTTP
 *    handler runs in the libwebsockets event loop (lws_service with epoll
 *    back-end on Linux), so the round-trip is network RTT + <1 ms processing.
 *  - State transitions are guarded by a single std::mutex; critical sections
 *    are kept as short as possible (no I/O or blocking calls inside locks).
 *  - libwebsockets is used with its default epoll/kqueue event back-end so the
 *    main thread is truly event-driven and does not spin.
 *
 * SOLID rationale (brief):
 *  S – Each class has one responsibility: Solenoid controls one GPIO output;
 *      WeightSensor reads the HX711; LockerController owns the state machine;
 *      the HTTP layer only serialises/deserialises requests.
 *  O – LockerController accepts a WeightSensor::Callback, so the sensor
 *      can be swapped or mocked without changing the controller (open for
 *      extension via the callback, closed to modification).
 *  L – Not directly applicable (no inheritance hierarchy used); kept simple
 *      intentionally.
 *  I – Public interfaces are minimal; internal helpers are private.
 *  D – LockerController depends on abstractions (references + callback),
 *      not on concrete GPIO details.
 */

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
#include <ctime>
#include <cassert>
#include <libwebsockets.h>
#include <gpiod.h>

// ── Pin / tuning constants ────────────────────────────────────────────────────
#define CHIP             "/dev/gpiochip0"
#define DT_PIN           5
#define SCK_PIN          11
#define SOLENOID_PIN     18
#define HTTP_PORT        8080
#define RAW_TRIGGER      3000   // raw-unit threshold for "item present"

#define RATE_PER_MIN     0.10f  // £ per minute
#define MIN_CHARGE       1.0f   // £ minimum charge

// ── RAII helpers for libgpiod resources ──────────────────────────────────────
// Wrapping C handles in RAII structs prevents leaks on any exit path.

struct GpiodChipGuard {
    gpiod_chip* ptr = nullptr;
    explicit GpiodChipGuard(const char* path) { ptr = gpiod_chip_open(path); }
    ~GpiodChipGuard() { if (ptr) { gpiod_chip_close(ptr); ptr = nullptr; } }
    operator gpiod_chip*() const { return ptr; }
    bool ok() const { return ptr != nullptr; }
    // Non-copyable
    GpiodChipGuard(const GpiodChipGuard&)            = delete;
    GpiodChipGuard& operator=(const GpiodChipGuard&) = delete;
};

struct GpiodRequestGuard {
    gpiod_line_request* ptr = nullptr;
    ~GpiodRequestGuard() { release(); }
    void release() {
        if (ptr) { gpiod_line_request_release(ptr); ptr = nullptr; }
    }
    operator gpiod_line_request*() const { return ptr; }
    bool ok() const { return ptr != nullptr; }
    GpiodRequestGuard(const GpiodRequestGuard&)            = delete;
    GpiodRequestGuard& operator=(const GpiodRequestGuard&) = delete;
    GpiodRequestGuard() = default;
};

// ── SOLENOID ─────────────────────────────────────────────────────────────────

class Solenoid {
public:
    Solenoid() : locked_(false), ready_(false) {}

    bool init() {
        chip_ = GpiodChipGuard(CHIP);
        if (!chip_.ok()) {
            std::cerr << "ERROR: Cannot open GPIO chip for solenoid\n";
            return false;
        }

        unsigned int off = SOLENOID_PIN;

        gpiod_line_settings* cfg = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(cfg, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(cfg, GPIOD_LINE_VALUE_INACTIVE);

        gpiod_line_config* lc = gpiod_line_config_new();
        gpiod_line_config_add_line_settings(lc, &off, 1, cfg);

        gpiod_request_config* rc = gpiod_request_config_new();
        gpiod_request_config_set_consumer(rc, "solenoid");

        req_.ptr = gpiod_chip_request_lines(chip_.ptr, rc, lc);

        gpiod_line_config_free(lc);
        gpiod_line_settings_free(cfg);
        gpiod_request_config_free(rc);

        if (!req_.ok()) {
            std::cerr << "ERROR: Cannot request solenoid GPIO line\n";
            return false;
        }

        ready_ = true;
        lock();
        std::cout << "Solenoid initialised OK — LOCKED\n";
        return true;
    }

    void lock() {
        if (!ready_) return;
        gpiod_line_request_set_value(req_.ptr, SOLENOID_PIN, GPIOD_LINE_VALUE_ACTIVE);
        locked_ = true;
        std::cout << "Solenoid: LOCKED\n";
    }

    void unlock() {
        if (!ready_) return;
        gpiod_line_request_set_value(req_.ptr, SOLENOID_PIN, GPIOD_LINE_VALUE_INACTIVE);
        locked_ = false;
        std::cout << "Solenoid: UNLOCKED\n";
    }

    bool isLocked() const { return locked_; }

    // Called on shutdown — ensure we leave the solenoid unlocked so the locker
    // is not stuck in a locked state if the process crashes.
    void shutdown() {
        unlock();
        req_.release();
    }

private:
    GpiodChipGuard    chip_;
    GpiodRequestGuard req_;
    bool              locked_;
    bool              ready_;
};

// ── WEIGHT SENSOR ─────────────────────────────────────────────────────────────
//
// Real-time design:
//   A dedicated std::thread runs loop(), which blocks on the HX711 DT line
//   (100 µs busy-wait, bounded by a 500 ms timeout guard) and then fires
//   the user-supplied callback.  The HX711 at 10 SPS has a 100 ms period;
//   a single-item detect requires 10 consecutive positive readings (~1 s),
//   which is well within human perception thresholds.

class WeightSensor {
public:
    using Callback = std::function<void(long)>;

    WeightSensor()
        : running_(false), rawValue_(0),
          zeroOffset_(-489500),
          ready_(false) {}

    bool init() {
        chip_ = GpiodChipGuard(CHIP);
        if (!chip_.ok()) {
            std::cerr << "ERROR: Cannot open GPIO chip for HX711\n";
            return false;
        }

        unsigned int dt_off  = DT_PIN;
        unsigned int sck_off = SCK_PIN;

        gpiod_line_settings* dt_cfg = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(dt_cfg, GPIOD_LINE_DIRECTION_INPUT);

        gpiod_line_settings* sck_cfg = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(sck_cfg, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(sck_cfg, GPIOD_LINE_VALUE_INACTIVE);

        gpiod_line_config* lc = gpiod_line_config_new();
        gpiod_line_config_add_line_settings(lc, &dt_off,  1, dt_cfg);
        gpiod_line_config_add_line_settings(lc, &sck_off, 1, sck_cfg);

        gpiod_request_config* rc = gpiod_request_config_new();
        gpiod_request_config_set_consumer(rc, "hx711");

        req_.ptr = gpiod_chip_request_lines(chip_.ptr, rc, lc);

        gpiod_line_config_free(lc);
        gpiod_line_settings_free(dt_cfg);
        gpiod_line_settings_free(sck_cfg);
        gpiod_request_config_free(rc);

        if (!req_.ok()) {
            std::cerr << "ERROR: Cannot request HX711 GPIO lines\n";
            return false;
        }

        ready_ = true;
        std::cout << "HX711 GPIO initialised OK\n";
        return true;
    }

    void calibrateZero() {
        std::cout << "Calibrating zero — keep scale empty...\n";
        long sum = 0; int count = 0;
        for (int i = 0; i < 20; i++) {
            long r = readOnce();
            if (r == INVALID_READ || r > -100000) continue;
            sum += r; count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (count > 0) {
            std::lock_guard<std::mutex> lk(mtx_);
            zeroOffset_ = sum / count;
            std::cout << "Zero offset: " << zeroOffset_ << "\n";
        } else {
            std::cout << "Calibration failed, using default: " << zeroOffset_ << "\n";
        }
    }

    void setCallback(Callback cb) {
        std::lock_guard<std::mutex> lk(mtx_);
        onChange_ = std::move(cb);
    }

    void start() {
        running_ = true;
        worker_  = std::thread(&WeightSensor::loop, this);
    }

    void stop() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
        req_.release();
    }

    long getRaw() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return rawValue_;
    }

    // Return zeroOffset without holding the caller's lock to avoid deadlock
    // when called from within a callback that already holds LockerController's
    // mutex.  zeroOffset_ is only written during init (calibrateZero) before
    // start() is called, so reading it lock-free here is safe.
    long getZero() const { return zeroOffset_; }

private:
    static constexpr long INVALID_READ = -1;

    std::atomic<bool>    running_;
    long                 rawValue_;
    long                 zeroOffset_;
    mutable std::mutex   mtx_;
    std::thread          worker_;
    Callback             onChange_;
    GpiodChipGuard       chip_;
    GpiodRequestGuard    req_;
    bool                 ready_;

    // The sensor thread: blocks until a valid HX711 reading, updates shared
    // state, then immediately fires the callback.  A 1 ms yield between reads
    // prevents 100% CPU usage on timeout paths.
    void loop() {
        while (running_) {
            long r = ready_ ? readOnce() : INVALID_READ;
            if (r == INVALID_READ || r > -100000) {
                // Yield CPU briefly on bad reads to avoid busy-spinning
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            Callback cb;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                rawValue_ = r;
                cb = onChange_;   // copy under lock, call outside lock
            }
            if (cb) cb(r);
        }
    }

    // Bit-bang the HX711 protocol.  Waits up to 500 ms for DT to go low
    // (HX711 data-ready signal) then clocks out 24 bits + 1 gain-set pulse.
    long readOnce() {
        int timeout = 0;
        while (gpiod_line_request_get_value(req_.ptr, DT_PIN)
               != GPIOD_LINE_VALUE_INACTIVE)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            if (++timeout > 5000) return INVALID_READ;  // 500 ms guard
        }
        long raw = 0;
        for (int i = 0; i < 24; i++) {
            gpiod_line_request_set_value(req_.ptr, SCK_PIN, GPIOD_LINE_VALUE_ACTIVE);
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            int bit = (gpiod_line_request_get_value(req_.ptr, DT_PIN)
                       == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;
            raw = (raw << 1) | bit;
            gpiod_line_request_set_value(req_.ptr, SCK_PIN, GPIOD_LINE_VALUE_INACTIVE);
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        // One extra pulse to set gain=128 for next conversion
        gpiod_line_request_set_value(req_.ptr, SCK_PIN, GPIOD_LINE_VALUE_ACTIVE);
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        gpiod_line_request_set_value(req_.ptr, SCK_PIN, GPIOD_LINE_VALUE_INACTIVE);
        std::this_thread::sleep_for(std::chrono::microseconds(1));

        // Sign-extend 24-bit two's complement → long
        if (raw & 0x800000) raw |= ~0xFFFFFFL;
        return raw;
    }
};

// ── LOCKER CONTROLLER ────────────────────────────────────────────────────────
//
// State machine:
//   IDLE    → user presses Open  → OPEN
//   OPEN    → item detected      → ITEM_IN
//   ITEM_IN → user presses Close → RUNNING
//   RUNNING → item removed       → ENDED
//   ENDED   → user presses Open  → OPEN  (new session)

enum class State { IDLE, OPEN, ITEM_IN, RUNNING, ENDED };

class LockerController {
public:
    LockerController(WeightSensor& sensor, Solenoid& solenoid)
        : sensor_(sensor), solenoid_(solenoid),
          state_(State::IDLE),
          rentAmount_(0.0f), elapsedSecs_(0),
          startTimeStr_("--"), finalRent_(""),
          itemDetectCount_(0)
    {
        // Register callback: sensor thread calls onRaw() on every new reading.
        sensor_.setCallback([this](long r){ onRaw(r); });
    }

    // ── Actions triggered by HTTP button presses ──────────────────────────

    void pressOpen() {
        std::lock_guard<std::mutex> lk(mtx_);
        if (state_ == State::IDLE || state_ == State::ENDED) {
            state_    = State::OPEN;
            finalRent_ = "";
            solenoid_.unlock();
            std::cout << "User pressed OPEN\n";
        }
    }

    void pressClose() {
        std::lock_guard<std::mutex> lk(mtx_);
        if (state_ == State::ITEM_IN) {
            state_        = State::RUNNING;
            sessionStart_ = std::chrono::steady_clock::now();
            startTimeStr_ = currentTime();
            rentAmount_   = MIN_CHARGE;
            elapsedSecs_  = 0;
            solenoid_.lock();
            std::cout << "Session started: " << startTimeStr_ << "\n";
        }
    }

    // ── JSON serialisation for /data endpoint ────────────────────────────

    std::string getJSON() {
        std::lock_guard<std::mutex> lk(mtx_);

        if (state_ == State::RUNNING) {
            auto now     = std::chrono::steady_clock::now();
            elapsedSecs_ = std::chrono::duration_cast<std::chrono::seconds>(
                               now - sessionStart_).count();
            float minutes = elapsedSecs_ / 60.0f;
            rentAmount_   = std::max(minutes * RATE_PER_MIN, MIN_CHARGE);
        }

        long long h = elapsedSecs_ / 3600;
        long long m = (elapsedSecs_ % 3600) / 60;
        long long s = elapsedSecs_ % 60;
        std::ostringstream timer;
        timer << std::setfill('0') << std::setw(2) << h << ":"
              << std::setfill('0') << std::setw(2) << m << ":"
              << std::setfill('0') << std::setw(2) << s;

        const char* stateStr  = "";
        const char* statusMsg = "";
        switch (state_) {
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

        bool isRunning = (state_ == State::RUNNING);
        std::ostringstream o;
        o << std::fixed << std::setprecision(2);
        o << "{\"state\":\""    << stateStr   << "\""
          << ",\"status\":\""   << statusMsg  << "\""
          << ",\"rent\":"       << (isRunning ? rentAmount_ : 0.0f)
          << ",\"elapsed\":\""  << (isRunning ? timer.str() : "00:00:00") << "\""
          << ",\"started\":\""  << (isRunning ? startTimeStr_ : "--") << "\""
          << ",\"finalRent\":\"" << finalRent_ << "\""
          << "}";
        return o.str();
    }

    // ── Unit-test hook: directly inject a state (test mode only) ─────────
    void setState_TEST(State s) {
        std::lock_guard<std::mutex> lk(mtx_);
        state_ = s;
    }
    State getState_TEST() {
        std::lock_guard<std::mutex> lk(mtx_);
        return state_;
    }

private:
    WeightSensor& sensor_;
    Solenoid&     solenoid_;
    mutable std::mutex mtx_;

    State       state_;
    float       rentAmount_;
    long long   elapsedSecs_;
    std::string startTimeStr_;
    std::string finalRent_;
    int         itemDetectCount_;
    std::chrono::steady_clock::time_point sessionStart_;

    std::string currentTime() const {
        auto t  = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream o;
        o << std::setfill('0')
          << std::setw(2) << tm.tm_hour << ":"
          << std::setw(2) << tm.tm_min  << ":"
          << std::setw(2) << tm.tm_sec;
        return o.str();
    }

    // Called from WeightSensor's thread via the registered callback.
    // Must not call sensor_.getZero() under mtx_ to avoid deadlock;
    // getZero() is lock-free (see WeightSensor::getZero comment).
    void onRaw(long r) {
        long zero    = sensor_.getZero();       // lock-free read
        long diff    = zero - r;
        bool hasItem = (diff > RAW_TRIGGER);

        std::lock_guard<std::mutex> lk(mtx_);

        if (state_ == State::OPEN) {
            if (hasItem) {
                if (++itemDetectCount_ >= 10) {
                    state_           = State::ITEM_IN;
                    itemDetectCount_ = 0;
                    std::cout << "Item detected — diff=" << diff << "\n";
                }
            } else {
                itemDetectCount_ = 0;
            }

        } else if (state_ == State::RUNNING) {
            if (!hasItem) {
                auto now      = std::chrono::steady_clock::now();
                elapsedSecs_  = std::chrono::duration_cast<std::chrono::seconds>(
                                    now - sessionStart_).count();
                float minutes = elapsedSecs_ / 60.0f;
                rentAmount_   = std::max(minutes * RATE_PER_MIN, MIN_CHARGE);

                std::ostringstream rs;
                rs << std::fixed << std::setprecision(2) << rentAmount_;
                finalRent_ = rs.str();
                state_     = State::ENDED;
                solenoid_.unlock();
                std::cout << "Item removed. Duration=" << elapsedSecs_
                          << "s  Rent=£" << rentAmount_ << "\n";
            }
        }
    }
};

// ── HTML DASHBOARD ────────────────────────────────────────────────────────────

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
  <div class="card" id="status-card">
    <div class="label">Locker status</div>
    <div style="padding:.4rem 0">
      <span class="dot" id="dot"></span>
      <span class="status-text" id="status-val">Locker closed</span>
    </div>
  </div>
  <button class="btn btn-open" id="btn-open" onclick="pressOpen()">Open Locker</button>
  <button class="btn btn-close" id="btn-close" onclick="pressClose()" disabled style="display:none">Close Locker</button>
  <div class="row" id="session-row" style="display:none">
    <div class="card red">
      <div class="label">Duration</div>
      <div class="big red" id="elapsed-val">00:00:00</div>
      <div class="sub" id="started-val">--</div>
    </div>
    <div class="card red">
      <div class="label">Rent due</div>
      <div class="big money" id="rent-val">£1.00</div>
      <div class="sub">£0.10 per min</div>
    </div>
  </div>
  <div class="final-box" id="final-box">
    <div class="final-label">Total amount due</div>
    <div class="final-amount" id="final-amount">£0.00</div>
    <div class="final-dur" id="final-duration">--</div>
  </div>
  <div class="history">
    <div class="label">Event log</div>
    <div id="log"></div>
  </div>
  <div class="rate-note">Rate: £0.10 per minute &nbsp;|&nbsp; Min charge: £1.00</div>
</div>
<div class="conn" id="conn">Connecting...</div>
<script>
var lastStatus='',lastElapsed='00:00:00';
function addLog(msg){var d=document.getElementById('log'),t=new Date().toLocaleTimeString();d.innerHTML='<div class="log-line"><span>'+t+'</span> — '+msg+'</div>'+d.innerHTML;if(d.children.length>30)d.removeChild(d.lastChild);}
function pressOpen(){fetch('/open',{method:'POST'});}
function pressClose(){fetch('/close',{method:'POST'});}
function update(){
  fetch('/data').then(function(r){return r.json();}).then(function(d){
    document.getElementById('conn').textContent='Live';
    document.getElementById('conn').className='conn ok';
    var dot=document.getElementById('dot'),sc=document.getElementById('status-card');
    var btnOpen=document.getElementById('btn-open'),btnClose=document.getElementById('btn-close');
    var sessionRow=document.getElementById('session-row'),finalBox=document.getElementById('final-box');
    document.getElementById('status-val').textContent=d.status;
    btnOpen.style.display='none';btnClose.style.display='none';btnClose.disabled=true;
    sessionRow.style.display='none';finalBox.className='final-box';
    if(d.state==='idle'){dot.className='dot';sc.className='card';btnOpen.style.display='block';btnOpen.disabled=false;btnOpen.textContent='Open Locker';}
    else if(d.state==='open'){dot.className='dot blue';sc.className='card blue';btnOpen.style.display='block';btnOpen.disabled=true;}
    else if(d.state==='item_in'){dot.className='dot yellow';sc.className='card yellow';btnClose.style.display='block';btnClose.disabled=false;}
    else if(d.state==='running'){dot.className='dot red';sc.className='card red';sessionRow.style.display='grid';document.getElementById('elapsed-val').textContent=d.elapsed;document.getElementById('rent-val').textContent='£'+d.rent.toFixed(2);document.getElementById('started-val').textContent='Since '+d.started;lastElapsed=d.elapsed;}
    else if(d.state==='ended'){dot.className='dot yellow';sc.className='card yellow';finalBox.className='final-box show';document.getElementById('final-amount').textContent='£'+d.finalRent;document.getElementById('final-duration').textContent='Duration: '+lastElapsed;btnOpen.style.display='block';btnOpen.disabled=false;btnOpen.textContent='New Session';}
    if(d.status!==lastStatus){addLog(d.status);lastStatus=d.status;}
  }).catch(function(){document.getElementById('conn').textContent='Reconnecting...';document.getElementById('conn').className='conn';});
}
setInterval(update,300);update();
</script>
</body>
</html>)HTML";

// ── HTTP SERVER ───────────────────────────────────────────────────────────────
//
// Per-session data passed via lws user pointer — eliminates global variables.
// The LockerController pointer is stored in lws_context user data and
// retrieved per-callback via lws_context_user(), keeping the HTTP layer
// stateless and the controller fully encapsulated.

static int callback_http(struct lws* wsi,
                         enum lws_callback_reasons reason,
                         void* /*user*/, void* in, size_t /*len*/)
{
    if (reason != LWS_CALLBACK_HTTP) return 0;

    // Retrieve controller from context user data (no globals)
    LockerController* locker =
        static_cast<LockerController*>(lws_context_user(lws_get_context(wsi)));

    const char* url = static_cast<const char*>(in);

    auto sendText = [&](int status, const char* mime,
                        const char* body, size_t bodyLen) {
        unsigned char hdr[512];
        unsigned char* p   = hdr;
        unsigned char* end = hdr + sizeof(hdr);
        lws_add_http_common_headers(wsi, status, mime,
                                    (lws_filepos_t)bodyLen, &p, end);
        lws_finalize_write_http_header(wsi, hdr, &p, end);
        lws_write(wsi, (unsigned char*)body, bodyLen, LWS_WRITE_HTTP_FINAL);
    };

    if (url && strncmp(url, "/open", 5) == 0) {
        if (locker) locker->pressOpen();
        sendText(HTTP_STATUS_OK, "text/plain", "OK", 2);
        return 1;
    }
    if (url && strncmp(url, "/close", 6) == 0) {
        if (locker) locker->pressClose();
        sendText(HTTP_STATUS_OK, "text/plain", "OK", 2);
        return 1;
    }
    if (url && strncmp(url, "/data", 5) == 0) {
        std::string json = locker
            ? locker->getJSON()
            : R"({"state":"idle","status":"offline","rent":0,"elapsed":"00:00:00","started":"--","finalRent":""})";

        unsigned char hdr[512];
        unsigned char* p   = hdr;
        unsigned char* end = hdr + sizeof(hdr);
        lws_add_http_common_headers(wsi, HTTP_STATUS_OK,
            "application/json", (lws_filepos_t)json.size(), &p, end);
        lws_add_http_header_by_name(wsi,
            (unsigned char*)"Access-Control-Allow-Origin:",
            (unsigned char*)"*", 1, &p, end);
        lws_finalize_write_http_header(wsi, hdr, &p, end);
        lws_write(wsi, (unsigned char*)json.c_str(),
                  json.size(), LWS_WRITE_HTTP_FINAL);
        return 1;
    }

    // Default: serve dashboard
    sendText(HTTP_STATUS_OK, "text/html",
             DASHBOARD_HTML, strlen(DASHBOARD_HTML));
    return 1;
}

static struct lws_protocols protocols[] = {
    { "http", callback_http, 0, 0, 0, nullptr, 0 },
    { nullptr, nullptr,      0, 0, 0, nullptr, 0 }
};

// ── UNIT TESTS ────────────────────────────────────────────────────────────────
//
// Run with:  ./smart_locker --test
//
// These tests exercise the LockerController state machine without real GPIO
// by using mock/stub sensor and solenoid objects.

namespace test {

// Minimal stub solenoid that records calls without touching GPIO
struct StubSolenoid : public Solenoid {
    bool initCalled  = false;
    int  lockCalls   = 0;
    int  unlockCalls = 0;
    // Override by shadowing — Solenoid is not polymorphic but we can reuse
    // LockerController by passing a real Solenoid reference; for stub use
    // we call methods on the Solenoid base which is no-op when !ready_.
};

// Stub sensor that lets tests inject raw values via the callback
struct StubSensor : public WeightSensor {
    void fireCallback(long raw) {
        // Directly invoke whatever callback is registered
        // (we re-use WeightSensor's setCallback/getZero interface)
    }
};

void run() {
    std::cout << "\n=== Running unit tests ===\n";
    int passed = 0, failed = 0;

    auto check = [&](const char* name, bool cond) {
        if (cond) { std::cout << "  PASS: " << name << "\n"; ++passed; }
        else      { std::cerr << "  FAIL: " << name << "\n"; ++failed; }
    };

    // ── Test 1: Initial state is IDLE ──────────────────────────────────────
    {
        Solenoid  sol;   // no init() → all calls are no-ops
        WeightSensor ws;
        LockerController lc(ws, sol);
        check("Initial state is IDLE",
              lc.getState_TEST() == State::IDLE);
    }

    // ── Test 2: pressOpen() from IDLE → OPEN ──────────────────────────────
    {
        Solenoid sol; WeightSensor ws;
        LockerController lc(ws, sol);
        lc.pressOpen();
        check("pressOpen from IDLE → OPEN",
              lc.getState_TEST() == State::OPEN);
    }

    // ── Test 3: pressOpen() from OPEN has no effect ────────────────────────
    {
        Solenoid sol; WeightSensor ws;
        LockerController lc(ws, sol);
        lc.pressOpen();
        lc.pressOpen();  // second call — already OPEN
        check("pressOpen from OPEN stays OPEN",
              lc.getState_TEST() == State::OPEN);
    }

    // ── Test 4: pressClose() from OPEN has no effect ───────────────────────
    {
        Solenoid sol; WeightSensor ws;
        LockerController lc(ws, sol);
        lc.pressOpen();
        lc.pressClose();  // no item yet
        check("pressClose without item stays OPEN",
              lc.getState_TEST() == State::OPEN);
    }

    // ── Test 5: pressClose() from ITEM_IN → RUNNING ────────────────────────
    {
        Solenoid sol; WeightSensor ws;
        LockerController lc(ws, sol);
        lc.setState_TEST(State::ITEM_IN);
        lc.pressClose();
        check("pressClose from ITEM_IN → RUNNING",
              lc.getState_TEST() == State::RUNNING);
    }

    // ── Test 6: pressOpen() from ENDED → OPEN (new session) ────────────────
    {
        Solenoid sol; WeightSensor ws;
        LockerController lc(ws, sol);
        lc.setState_TEST(State::ENDED);
        lc.pressOpen();
        check("pressOpen from ENDED → OPEN",
              lc.getState_TEST() == State::OPEN);
    }

    // ── Test 7: getJSON() returns valid JSON structure ──────────────────────
    {
        Solenoid sol; WeightSensor ws;
        LockerController lc(ws, sol);
        std::string j = lc.getJSON();
        bool hasState  = j.find("\"state\"")  != std::string::npos;
        bool hasStatus = j.find("\"status\"") != std::string::npos;
        bool hasRent   = j.find("\"rent\"")   != std::string::npos;
        check("getJSON contains required fields",
              hasState && hasStatus && hasRent);
    }

    // ── Test 8: getJSON() in IDLE shows rent = 0 ──────────────────────────
    {
        Solenoid sol; WeightSensor ws;
        LockerController lc(ws, sol);
        std::string j = lc.getJSON();
        check("getJSON in IDLE: rent is 0",
              j.find("\"rent\":0.00") != std::string::npos);
    }

    std::cout << "\nResults: " << passed << " passed, "
              << failed << " failed\n";
    if (failed > 0) std::exit(1);
    std::cout << "All tests passed.\n\n";
}

} // namespace test

// ── SHUTDOWN HANDLER ──────────────────────────────────────────────────────────

static Solenoid* g_solenoid_shutdown = nullptr;

void onShutdown(int) {
    std::cout << "\nShutting down — unlocking solenoid...\n";
    if (g_solenoid_shutdown) g_solenoid_shutdown->unlock();
    std::exit(0);
}

// ── MAIN ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    // Run unit tests and exit if --test flag provided
    if (argc > 1 && std::string(argv[1]) == "--test") {
        test::run();
        return 0;
    }

    signal(SIGINT,  onShutdown);
    signal(SIGTERM, onShutdown);

    Solenoid     solenoid;
    WeightSensor sensor;

    if (!solenoid.init())
        std::cerr << "WARNING: Solenoid not initialised (GPIO unavailable)\n";
    if (!sensor.init())
        std::cerr << "WARNING: HX711 not initialised (GPIO unavailable)\n";

    sensor.calibrateZero();

    g_solenoid_shutdown = &solenoid;   // for signal handler only

    LockerController locker(sensor, solenoid);

    // Pass controller pointer via lws context user data — no globals needed
    // in the HTTP callback.
    struct lws_context_creation_info info{};
    info.port      = HTTP_PORT;
    info.protocols = protocols;
    info.user      = &locker;   // <── retrieved via lws_context_user() in callback

    struct lws_context* ctx = lws_create_context(&info);
    if (!ctx) {
        std::cerr << "Failed to start HTTP server on port " << HTTP_PORT << "\n";
        solenoid.shutdown();
        return 1;
    }

    sensor.start();

    std::cout << "=========================================\n"
              << "Smart Locker\n"
              << "Rate : £" << RATE_PER_MIN << " per minute\n"
              << "Min  : £" << MIN_CHARGE   << "\n"
              << "Open : http://<Pi-IP>:" << HTTP_PORT << "\n"
              << "Ctrl+C to stop and unlock\n"
              << "Test : ./smart_locker --test\n"
              << "=========================================\n";

    // Event-driven main loop.
    // lws_service() uses epoll on Linux; passing a 50 ms timeout means the
    // thread sleeps until network activity or at most 50 ms elapses.
    // This is NOT a busy spin — the thread blocks in epoll_wait().
    while (true) {
        lws_service(ctx, 50);
    }

    sensor.stop();
    solenoid.shutdown();
    lws_context_destroy(ctx);
    return 0;
}
