#ifndef _CAT_ACTION_EXECUTOR_H_
#define _CAT_ACTION_EXECUTOR_H_

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
        for (int i = 0; i < 5; i++) {
            if (group_mask_ & (1 << i)) {
                if (servo_count_ == 0) servo_offset_ = i;
                servo_count_++;
            }
        }
    }

    void Start(ServoSetFunc set_servo) {
        set_servo_ = set_servo;
        queue_ = xQueueCreate(4, sizeof(ActionRequest));
        stop_requested_ = false;
        char name[16];
        snprintf(name, sizeof(name), "act_%s", tag_);
        xTaskCreate(TaskEntry, name, 3072, this, 5, &task_handle_);
    }

    bool Run(const char* name, uint16_t speed = 0, uint8_t cycles = 0, bool loop = false) {
        const ServoAction* action = FindAction(name);
        if (!action) return false;
        return Enqueue(action, speed, cycles, loop);
    }

    bool RunById(uint16_t id, uint16_t speed = 0, uint8_t cycles = 0, bool loop = false) {
        const ServoAction* action = FindActionById(id);
        if (!action) return false;
        return Enqueue(action, speed, cycles, loop);
    }

    void Stop() {
        stop_requested_ = true;
        ActionRequest dummy;
        while (xQueueReceive(queue_, &dummy, 0) == pdTRUE) {}
    }

    bool IsRunning() const { return running_; }

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
        static const char* neck_list[] = {"neck_nod","neck_shake","neck_wave","neck_left","neck_right"};
        static const char* tail_list[] = {"tail_wag","tail_wag_fast","tail_tremble","tail_up","tail_curl"};
        static const char* head_list[] = {"head_nod","head_shake","head_tilt","head_left","head_right","head_loop"};
        const char** list = nullptr;
        int cnt = 0;
        if (group_mask_ == GROUP_NECK) { list = neck_list; cnt = 5; }
        else if (group_mask_ == GROUP_TAIL) { list = tail_list; cnt = 5; }
        else if (group_mask_ == GROUP_HEAD) { list = head_list; cnt = 6; }
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
        xQueueSend(queue_, &req, 0);
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
        uint8_t aps = (group_mask_ == GROUP_ALL) ? 5 : servo_count_;

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

#endif  // _CAT_ACTION_EXECUTOR_H_
