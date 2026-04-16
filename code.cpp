#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <chrono>
#include <atomic>
#include <functional>
#include <gpiod.h>

#define CHIP "/dev/gpiochip0"
#define SOLENOID_LINE 18

// ===========================
// WEIGHT SENSOR (EVENT DRIVEN)
// ===========================
class WeightSensor {
public:
    using Callback = std::function<void(float)>;

    WeightSensor() : running(false), weight(0.0f) {}

    void setCallback(Callback cb) {
        onChange = cb;
    }

    void start() {
        running = true;
        worker = std::thread(&WeightSensor::loop, this);
    }

    void stop() {
        running = false;
        if (worker.joinable())
            worker.join();
    }

    float getWeight() {
        std::lock_guard<std::mutex> lock(mtx);
        return weight;
    }

private:
    std::atomic<bool> running;
    float weight;
    std::mutex mtx;
    std::thread worker;
    Callback onChange;

    void loop() {
        while (running) {
            float newWeight = simulate();

            {
                std::lock_guard<std::mutex> lock(mtx);
                weight = newWeight;
            }

            if (onChange)
                onChange(newWeight);

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    float simulate() {
        static float v = 0.0f;
        return v += 0.5f;
    }
};

// ===========================
// LOCKER CONTROLLER (libgpiod v2 FIXED)
// ===========================
class LockerController {
public:
    LockerController(WeightSensor& s)
        : sensor(s), status("Idle"), itemStored(false)
    {
        chip = gpiod_chip_open(CHIP);

        // FIX 1: correct type (unsigned int)
        unsigned int line_offset = SOLENOID_LINE;

        settings = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(
            settings,
            GPIOD_LINE_DIRECTION_OUTPUT
        );

        // FIX 2: correct enum type (not int)
        gpiod_line_settings_set_output_value(
            settings,
            GPIOD_LINE_VALUE_INACTIVE
        );

        line_cfg = gpiod_line_config_new();

        gpiod_line_config_add_line_settings(
            line_cfg,
            &line_offset,
            1,
            settings
        );

        req_cfg = gpiod_request_config_new();
        gpiod_request_config_set_consumer(req_cfg, "locker");

        request = gpiod_chip_request_lines(
            chip,
            req_cfg,
            line_cfg
        );

        sensor.setCallback([this](float w) {
            onWeightChange(w);
        });
    }

    ~LockerController() {
        gpiod_line_request_set_value(
            request,
            SOLENOID_LINE,
            GPIOD_LINE_VALUE_INACTIVE
        );

        gpiod_line_request_release(request);

        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
        gpiod_request_config_free(req_cfg);

        gpiod_chip_close(chip);
    }

    void unlockForDeposit() {
        std::thread([this]() {
            {
                std::lock_guard<std::mutex> lock(mtx);
                status = "Unlocked";
            }

            setLock(true);

            std::this_thread::sleep_for(std::chrono::seconds(10));

            setLock(false);

            {
                std::lock_guard<std::mutex> lock(mtx);
                status = "Locked";
            }
        }).detach();
    }

    void display() {
        std::lock_guard<std::mutex> lock(mtx);

        std::cout << "\rStatus: " << status
                  << " | Weight: " << sensor.getWeight()
                  << " g     " << std::flush;
    }

private:
    WeightSensor& sensor;
    std::string status;
    bool itemStored;

    std::mutex mtx;

    struct gpiod_chip* chip;
    struct gpiod_line_config* line_cfg;
    struct gpiod_line_settings* settings;
    struct gpiod_request_config* req_cfg;
    struct gpiod_line_request* request;

    void setLock(bool unlock) {
        gpiod_line_request_set_value(
            request,
            SOLENOID_LINE,
            unlock ? GPIOD_LINE_VALUE_ACTIVE
                   : GPIOD_LINE_VALUE_INACTIVE
        );
    }

    void onWeightChange(float weight) {
        std::lock_guard<std::mutex> lock(mtx);

        if (weight > 20.0f && !itemStored) {
            itemStored = true;
            status = "Item Stored";
        }
    }
};

// ===========================
// MAIN
// ===========================
int main() {
    WeightSensor sensor;
    LockerController locker(sensor);

    sensor.start();

    std::cout << "Smart Locker System\n";
    std::cout << "Commands: rent / exit\n";

    std::string input;

    while (true) {
        locker.display();
        std::getline(std::cin, input);

        if (input == "rent") {
            locker.unlockForDeposit();
        }
        else if (input == "exit") {
            break;
        }
    }

    sensor.stop();
    return 0;
}