#ifndef _ACTION_EXECUTOR_H_
#define _ACTION_EXECUTOR_H_

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <driver/gpio.h>
#include <nvs_flash.h>

#include "action_list.h"
#include "config.h"

// Callback to set a servo angle: void(int servo_index, int angle_0_180)
using ServoSetFunc = void (*)(int, int);

struct ActionRequest {
    ServoAction action;  // Copy of action data (safe for queue)
    uint16_t speed_ms;   // Override default_ms, 0 = use default
    uint8_t cycles;      // Override default_cycles, 0 = use default
};

class ActionExecutor {
public:
    ActionExecutor() {}

    void Start(ServoSetFunc set_servo) {
        set_servo_ = set_servo;
        queue_ = xQueueCreate(8, sizeof(ActionRequest));
        stop_requested_ = false;
        xTaskCreate(TaskEntry, "action_exec", 3072, this, 2, nullptr);
    }

    bool Run(const char* name, uint16_t speed_ms, uint8_t cycles) {
        const ServoAction* action = FindAction(name);
        if (!action) return false;
        ActionRequest req;
        memcpy(&req.action, action, sizeof(ServoAction));
        req.speed_ms = speed_ms;
        req.cycles = cycles;
        xQueueSend(queue_, &req, 0);
        return true;
    }

    void Stop() {
        stop_requested_ = true;
        // Drain queue
        ActionRequest dummy;
        while (xQueueReceive(queue_, &dummy, 0) == pdTRUE) {}
    }

    bool IsRunning() const { return running_; }

    void SetBothServos(int a, int b) {
        if (set_servo_) { set_servo_(0, a); set_servo_(1, b); }
    }

    // Emotion-driven: play a random small ear movement
    void PlayRandomEmote() {
        // Pick from small/fast actions: both_tiny_fast, both_shake, left_tap, right_tap, etc.
        static const char* emotes[] = {
            "both_tiny_fast", "both_shake", "both_nod_fast",
            "left_tap", "left_nod_fast", "left_shake",
            "right_tap", "right_nod_fast", "right_shake",
            "dance_fast"
        };
        int idx = rand() % (sizeof(emotes) / sizeof(emotes[0]));
        Run(emotes[idx], 0, 0);  // Use default speed/cycles
    }

    const ServoAction* FindAction(const char* name) {
        // 1) Search predefined actions
        for (int i = 0; i < kActionCount; i++) {
            if (strcmp(kActionList[i].name, name) == 0) return &kActionList[i];
        }
        // 2) Search NVS custom actions
        return FindCustomAction(name);
    }

    const ServoAction* FindCustomAction(const char* name) {
        nvs_handle_t h;
        if (nvs_open("actions", NVS_READONLY, &h) != ESP_OK) return nullptr;
        size_t len = 0;
        if (nvs_get_str(h, "list", nullptr, &len) != ESP_OK || len == 0) {
            nvs_close(h); return nullptr;
        }
        char* json = (char*)malloc(len);
        if (!json || nvs_get_str(h, "list", json, &len) != ESP_OK) {
            free(json); nvs_close(h); return nullptr;
        }
        nvs_close(h);

        // Parse JSON array: [{"name":"xxx","servo":0,...},...]
        const char* p = json;
        const ServoAction* found = nullptr;
        while (*p) {
            if (*p == '{') {
                const char* end = strchr(p, '}');
                if (!end) break;
                std::string entry(p, end - p + 1);

                // Check name match
                char search[128];
                snprintf(search, sizeof(search), "\"name\":\"%s\"", name);
                if (strstr(entry.c_str(), search)) {
                    // Parse fields
                    static ServoAction dyn_action;
                    dyn_action.name = name;  // pointer to caller's string, OK for queue lifetime
                    dyn_action.servo = JsonGetInt(entry.c_str(), "servo");
                    dyn_action.start_angle = JsonGetInt(entry.c_str(), "start");
                    dyn_action.end_angle = JsonGetInt(entry.c_str(), "end");
                    dyn_action.default_ms = JsonGetInt(entry.c_str(), "ms");
                    dyn_action.default_cycles = JsonGetInt(entry.c_str(), "cycles");
                    dyn_action.flags = JsonGetInt(entry.c_str(), "flags");
                    found = &dyn_action;
                    break;
                }
                p = end + 1;
            } else {
                p++;
            }
        }
        free(json);
        return found;
    }

    static int JsonGetInt(const char* json, const char* key) {
        char search[64];
        snprintf(search, sizeof(search), "\"%s\":", key);
        const char* p = strstr(json, search);
        if (!p) return 0;
        p += strlen(search);
        while (*p == ' ' || *p == '\t') p++;
        return atoi(p);
    }

private:
    ServoSetFunc set_servo_ = nullptr;
    QueueHandle_t queue_ = nullptr;
    bool running_ = false;
    bool stop_requested_ = false;

    static void TaskEntry(void* arg) {
        static_cast<ActionExecutor*>(arg)->TaskLoop();
    }

    void TaskLoop() {
        ActionRequest req;
        TickType_t idle_start = 0;
        while (true) {
            // Wait for next action, with timeout to detect idle
            if (xQueueReceive(queue_, &req, pdMS_TO_TICKS(1000)) == pdTRUE) {
                stop_requested_ = false;
                running_ = true;
                ExecuteAction(req);
                running_ = false;
                idle_start = xTaskGetTickCount();
            } else if (!running_ && (xTaskGetTickCount() - idle_start) > pdMS_TO_TICKS(2000)) {
                // Idle for 2s after last action → reset ears to 0°
                if (set_servo_) {
                    set_servo_(0, 0);
                    set_servo_(1, 0);
                }
            }
        }
    }

    void ExecuteAction(const ActionRequest& req) {
        gpio_set_level(SERVO_POWER_GPIO, 1);  // Ensure servo power ON
        const ServoAction& a = req.action;
        uint16_t half_ms = req.speed_ms > 0 ? req.speed_ms : a.default_ms;
        uint8_t cycles = req.cycles > 0 ? req.cycles : a.default_cycles;
        bool back_forth = (a.flags & ACTION_FLAG_BACK_FORTH) != 0;

        const int step_ms = 20;
        int start = a.start_angle;
        int end = a.end_angle;
        int range = end - start;

        for (int c = 0; c < cycles && !stop_requested_; c++) {
            if (range == 0) {
                SetBoth(a.servo, start);
                for (int t = 0; t < half_ms && !stop_requested_; t += step_ms)
                    vTaskDelay(pdMS_TO_TICKS(step_ms));
                break;
            }

            int steps = half_ms / step_ms;
            if (steps < 2) steps = 2;
            for (int s = 0; s <= steps && !stop_requested_; s++) {
                int angle = start + (range * s / steps);
                if (a.servo == SERVO_A) set_servo_(0, angle);
                else if (a.servo == SERVO_B) set_servo_(1, angle);
                else if (a.flags & ACTION_FLAG_ALT) { set_servo_(0, angle); set_servo_(1, 180 - angle); }
                else { set_servo_(0, angle); set_servo_(1, angle); }
                vTaskDelay(pdMS_TO_TICKS(step_ms));
            }

            if (!back_forth) {
                for (int t = 0; t < half_ms && !stop_requested_; t += step_ms)
                    vTaskDelay(pdMS_TO_TICKS(step_ms));
                break;
            }

            for (int s = 0; s <= steps && !stop_requested_; s++) {
                int angle = end - (range * s / steps);
                if (a.servo == SERVO_A) set_servo_(0, angle);
                else if (a.servo == SERVO_B) set_servo_(1, angle);
                else if (a.flags & ACTION_FLAG_ALT) { set_servo_(0, angle); set_servo_(1, 180 - angle); }
                else { set_servo_(0, angle); set_servo_(1, angle); }
                vTaskDelay(pdMS_TO_TICKS(step_ms));
            }
        }
    }

    void SetBoth(uint8_t servo, int angle) {
        if (servo == SERVO_A || servo == SERVO_BOTH) set_servo_(0, angle);
        if (servo == SERVO_B || servo == SERVO_BOTH) set_servo_(1, angle);
    }
};

#endif  // _ACTION_EXECUTOR_H_
