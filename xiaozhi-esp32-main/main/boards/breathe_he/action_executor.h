#ifndef _PANDA_ACTION_EXECUTOR_H_
#define _PANDA_ACTION_EXECUTOR_H_

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "action_list.h"

using ServoSetFunc = void (*)(int, int);

struct ActionRequest {
    const ServoAction* action;
    uint16_t speed;   // ms/step, 0 = STEP_MS (100ms)
    uint8_t  cycles;  // 0 = once
    bool     loop;    // true = loop until stopped
};

class ActionExecutor {
public:
    ActionExecutor(uint8_t group_mask, const char* tag)
        : group_mask_(group_mask), tag_(tag) {
        servo_offset_ = 0;
        servo_count_ = 0;
        // Iterate over 3 servo bits (breathe_he has 3 servos)
        for (int i = 0; i < 3; i++) {
            if (group_mask_ & (1 << i)) {
                if (servo_count_ == 0) servo_offset_ = i;
                servo_count_++;
            }
        }
    }

    void Start(ServoSetFunc set_servo) {
        set_servo_ = set_servo;
        queue_ = xQueueCreate(8, sizeof(ActionRequest));
        stop_requested_ = false;
        char name[16];
        snprintf(name, sizeof(name), "act_%s", tag_);
        xTaskCreate(TaskEntry, name, 3072, this, 5, &task_handle_);
    }

    bool Run(const char* name, uint16_t speed = 0, uint8_t cycles = 0, bool loop = false) {
        const ServoAction* action = FindAction(name);
        if (!action) {
            ESP_LOGW("ActionExec", "%s: action '%s' NOT FOUND (kActionCount=%d)", tag_, name, kActionCount);
            return false;
        }
        return Enqueue(action, speed, cycles, loop);
    }

    bool RunById(uint16_t id, uint16_t speed = 0, uint8_t cycles = 0, bool loop = false) {
        const ServoAction* action = FindActionById(id);
        if (!action) {
            ESP_LOGW("ActionExec", "%s: action id=%d NOT FOUND", tag_, id);
            return false;
        }
        return Enqueue(action, speed, cycles, loop);
    }

    void Stop() {
        stop_requested_ = true;
        ActionRequest dummy;
        while (xQueueReceive(queue_, &dummy, 0) == pdTRUE) {}
    }

    // Clear queued actions without interrupting the currently running one
    void ClearPending() {
        ActionRequest dummy;
        while (xQueueReceive(queue_, &dummy, 0) == pdTRUE) {}
    }

    bool IsRunning() const { return running_; }
    bool IsBusy() const { return running_ || uxQueueMessagesWaiting(queue_) > 0; }

    const ServoAction* FindAction(const char* name) {
        for (int i = 0; i < kActionCount; i++)
            if (strcmp(kActionList[i].name, name) == 0) return &kActionList[i];
        return nullptr;
    }

    const ServoAction* FindActionById(uint16_t id) {
        for (int i = 0; i < kActionCount; i++)
            if (kActionList[i].id == id) return &kActionList[i];
        return nullptr;
    }

    void PlayRandomEmote() {
        static const char* head_list[] = {
            "head_left", "head_right", "head_nod", "head_shake",
            "head_tilt_l", "head_tilt_r", "head_scan", "head_curious", "head_calm",
        };
        static const char* left_hand_list[] = {
            "lh_crawl", "lh_breathe", "lh_lift", "lh_wave",
        };
        static const char* right_hand_list[] = {
            "rh_crawl", "rh_breathe", "rh_lift", "rh_wave",
        };
        static const char* hands_list[] = {
            "hands_breathe", "hands_breatheslow", "hands_crawlboth",
            "hands_crawlalt", "hands_rest", "hands_sway",
        };
        const char** list = nullptr;
        int cnt = 0;
        if (group_mask_ == GROUP_HEAD) {
            list = head_list;
            cnt = sizeof(head_list) / sizeof(head_list[0]);
        } else if (group_mask_ == GROUP_LEFT_HAND) {
            list = left_hand_list;
            cnt = sizeof(left_hand_list) / sizeof(left_hand_list[0]);
        } else if (group_mask_ == GROUP_RIGHT_HAND) {
            list = right_hand_list;
            cnt = sizeof(right_hand_list) / sizeof(right_hand_list[0]);
        } else if (group_mask_ == GROUP_HANDS) {
            list = hands_list;
            cnt = sizeof(hands_list) / sizeof(hands_list[0]);
        }
        if (list) Run(list[rand() % cnt], 0, 0);
    }

private:
    ServoSetFunc set_servo_ = nullptr;
    QueueHandle_t queue_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    bool running_ = false;
    bool stop_requested_ = false;
    uint8_t group_mask_;
    uint8_t servo_offset_;
    uint8_t servo_count_;
    const char* tag_;

    bool Enqueue(const ServoAction* action, uint16_t speed, uint8_t cycles, bool loop) {
        ActionRequest req;
        req.action = action;
        req.speed = speed;
        req.cycles = cycles;
        req.loop = loop;
        if (xQueueSend(queue_, &req, 0) != pdTRUE) {
            ESP_LOGW("ActionExec", "%s: queue full, dropped '%s'", tag_, action->name);
            return false;
        }
        ESP_LOGI("ActionExec", "%s: queued '%s' speed=%d cyc=%d", tag_, action->name, speed, cycles);
        return true;
    }

    static void TaskEntry(void* arg) {
        static_cast<ActionExecutor*>(arg)->TaskLoop();
    }

    void TaskLoop() {
        ActionRequest req;
        while (true) {
            if (xQueueReceive(queue_, &req, portMAX_DELAY) == pdTRUE) {
                stop_requested_ = false;
                running_ = true;
                ExecuteAction(req);
                running_ = false;
            }
        }
    }

    void ExecuteAction(const ActionRequest& req) {
        const ServoAction& action = *req.action;
        uint8_t cycles = req.cycles > 0 ? req.cycles : 1;
        bool loop = req.loop;
        uint16_t step_ms = req.speed > 0 ? req.speed : STEP_MS;
        // aps = angles per step: GROUP_ALL uses all 3 servos, otherwise uses subset
        uint8_t aps = (group_mask_ == GROUP_ALL) ? 3 : servo_count_;
        ESP_LOGI("ActionExec", "%s: exec '%s' steps=%d aps=%d", tag_, action.name, action.series[0].total_steps, aps);

        uint32_t n = 0;
        while ((loop || n < cycles) && !stop_requested_) {
            for (int s = 0; s < action.series_count && !stop_requested_; s++) {
                const ServoActionSeries& se = action.series[s];
                if (PlayData(se.data, se.total_steps, aps, step_ms)) break;
            }
            n++;
        }
    }

    bool PlayData(const int16_t* data, uint8_t total_steps,
                  uint8_t aps, uint16_t step_ms) {
        for (int s = 0; s < total_steps; s++) {
            if (stop_requested_) return true;
            const int16_t* step_data = data + s * aps;
            for (int a = 0; a < aps; a++) {
                int servo_idx = servo_offset_ + a;
                if (group_mask_ & (1 << servo_idx))
                    if (set_servo_) set_servo_(servo_idx, step_data[a]);
            }
            TickType_t start = xTaskGetTickCount();
            while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(step_ms)) {
                if (stop_requested_) return true;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        return false;
    }
};

#endif  // _PANDA_ACTION_EXECUTOR_H_
