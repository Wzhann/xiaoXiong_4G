#ifndef _CAT_ACTION_EXECUTOR_H_
#define _CAT_ACTION_EXECUTOR_H_

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
    uint16_t speed;     // ms/step, 0 = STEP_MS (100ms)
    uint8_t  cycles;    // 0 = once
    bool     loop;      // true = loop until stopped
    float    amplitude; // 1.0=normal, >1.0=wider, <1.0=subtler (scales deviation from 90°)
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
        queue_ = xQueueCreate(8, sizeof(ActionRequest));
        stop_requested_ = false;
        char name[16];
        snprintf(name, sizeof(name), "act_%s", tag_);
        xTaskCreate(TaskEntry, name, 3072, this, 5, &task_handle_);
    }

    bool Run(const char* name, uint16_t speed = 0, uint8_t cycles = 0, bool loop = false, float amplitude = -1.0f) {
        if (amplitude < 0.0f) amplitude = default_amplitude_;
        const ServoAction* action = FindAction(name);
        if (!action) {
            ESP_LOGW("ActionExec", "%s: action '%s' NOT FOUND (kActionCount=%d)", tag_, name, kActionCount);
            return false;
        }
        return Enqueue(action, speed, cycles, loop, amplitude);
    }

    bool RunById(uint16_t id, uint16_t speed = 0, uint8_t cycles = 0, bool loop = false, float amplitude = -1.0f) {
        if (amplitude < 0.0f) amplitude = default_amplitude_;
        const ServoAction* action = FindActionById(id);
        if (!action) {
            ESP_LOGW("ActionExec", "%s: action id=%d NOT FOUND", tag_, id);
            return false;
        }
        return Enqueue(action, speed, cycles, loop, amplitude);
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

    // Set default amplitude for subsequent Run() calls (1.0 = normal, 1.5 = 50% larger)
    void SetAmplitude(float amp) { default_amplitude_ = amp; }
    float GetAmplitude() const { return default_amplitude_; }

    void PlayRandomEmote() {
        static const char* neck_list[] = {"neck_nod","neck_wave","neck_left","neck_right",
                                          "neck_circle","neck_curious","neck_sway",
                                          "neck_tilt_left","neck_tilt_right","neck_nod_slow",
                                          "neck_glance_lu","neck_figure8","neck_stretch","neck_relax"};
        static const char* tail_list[] = {"tail_wag","tail_up","tail_curl","tail_loop","tail_sway",
                                          "tail_bounce","tail_scurve","tail_perk","tail_wag_slow",
                                          "tail_question","tail_spiral","tail_rest","tail_quiver"};
        static const char* head_list[] = {"head_nod","head_tilt","head_left","head_right",
                                          "head_loop","head_sway","head_curious",
                                          "head_glance_l","head_glance_r","head_dbl_take",
                                          "head_bob","head_scan_l","head_scan_r","head_tilt_l","head_tilt_r",
                                          "head_calm","head_relax","head_micro_l","head_micro_r"};
        const char** list = nullptr;
        int cnt = 0;
        if (group_mask_ == GROUP_NECK) { list = neck_list; cnt = sizeof(neck_list)/sizeof(neck_list[0]); }
        else if (group_mask_ == GROUP_TAIL) { list = tail_list; cnt = sizeof(tail_list)/sizeof(tail_list[0]); }
        else if (group_mask_ == GROUP_HEAD) { list = head_list; cnt = sizeof(head_list)/sizeof(head_list[0]); }
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
    float default_amplitude_ = 1.0f;

    bool Enqueue(const ServoAction* action, uint16_t speed, uint8_t cycles, bool loop, float amplitude = 1.0f) {
        ActionRequest req;
        req.action = action;
        req.speed = speed;
        req.cycles = cycles;
        req.loop = loop;
        req.amplitude = amplitude;
        if (xQueueSend(queue_, &req, 0) != pdTRUE) {
            ESP_LOGW("ActionExec", "%s: queue full, dropped '%s'", tag_, action->name);
            return false;
        }
        ESP_LOGI("ActionExec", "%s: queued '%s' speed=%d cyc=%d amp=%.2f", tag_, action->name, speed, cycles, amplitude);
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
        // For ALL actions on per-group executors: stride=5, count=servo_count_, skip to offset
        uint8_t stride, count, data_off;
        if (action.group_mask == GROUP_ALL && group_mask_ != GROUP_ALL) {
            stride = 5;             // full 5-servo step
            count = servo_count_;   // only our servos
            data_off = servo_offset_; // skip to neck/tail/head section
        } else {
            stride = servo_count_;
            count = servo_count_;
            data_off = 0;
        }
        ESP_LOGI("ActionExec", "%s: exec '%s' steps=%d stride=%d count=%d off=%d",
                 tag_, action.name, action.series[0].total_steps, stride, count, data_off);

        uint32_t n = 0;
        while ((loop || n < cycles) && !stop_requested_) {
            for (int s = 0; s < action.series_count && !stop_requested_; s++) {
                const ServoActionSeries& se = action.series[s];
                if (PlayData(se.data, se.total_steps, stride, count, data_off, step_ms, req.amplitude)) break;
            }
            n++;
        }
    }

    bool PlayData(const int16_t* data, uint8_t total_steps,
                  uint8_t stride, uint8_t count, uint8_t data_off, uint16_t step_ms, float amplitude = 1.0f) {
        // Clamp amplitude to safe range
        float amp = amplitude < 0.2f ? 0.2f : (amplitude > 2.5f ? 2.5f : amplitude);
        // Micro-step interpolation: subdivide each step into ~20ms slices for buttery-smooth motion
        constexpr int kMicroStepMs = 20;
        int micro_steps = (step_ms + kMicroStepMs - 1) / kMicroStepMs;
        if (micro_steps < 1) micro_steps = 1;
        int16_t prev_angles[5] = {90, 90, 90, 90, 90};  // start from neutral
        bool first_step = true;

        for (int s = 0; s < total_steps; s++) {
            if (stop_requested_) return true;
            const int16_t* step_data = data + s * stride + data_off;

            // Compute target angles for this step (with amplitude scaling)
            int16_t target[5];
            for (int a = 0; a < count; a++) {
                int raw = step_data[a];
                int scaled = 90 + (int)((float)(raw - 90) * amp);
                if (scaled < 0) scaled = 0;
                if (scaled > 180) scaled = 180;
                target[a] = (int16_t)scaled;
            }

            // On first step, jump to first position to avoid drifting from unknown start
            if (first_step) {
                for (int a = 0; a < count; a++) {
                    int servo_idx = servo_offset_ + a;
                    if (group_mask_ & (1 << servo_idx) && set_servo_)
                        set_servo_(servo_idx, target[a]);
                    prev_angles[a] = target[a];
                }
                first_step = false;
                vTaskDelay(pdMS_TO_TICKS(kMicroStepMs));
                continue;
            }

            // Interpolate from prev_angles → target over micro_steps
            for (int m = 1; m <= micro_steps; m++) {
                if (stop_requested_) return true;
                float t = (float)m / (float)micro_steps;  // 0→1
                for (int a = 0; a < count; a++) {
                    int servo_idx = servo_offset_ + a;
                    if (group_mask_ & (1 << servo_idx) && set_servo_) {
                        int angle = prev_angles[a] + (int)((float)(target[a] - prev_angles[a]) * t);
                        if (angle < 0) angle = 0;
                        if (angle > 180) angle = 180;
                        set_servo_(servo_idx, angle);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(kMicroStepMs));
            }
            // Remember target for next step's interpolation
            for (int a = 0; a < count; a++) prev_angles[a] = target[a];
        }
        return false;
    }
};

#endif  // _CAT_ACTION_EXECUTOR_H_
