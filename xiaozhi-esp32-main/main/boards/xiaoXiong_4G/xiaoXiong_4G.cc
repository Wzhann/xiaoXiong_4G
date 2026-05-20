#include "application.h"
#include "assets/lang_config.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "display/dual_eye_render.h"
#include "display/lcd_display.h"
#include "dual_network_board.h"
#include "esp32_camera.h"
#include "esp_lcd_gc9d01n.h"
#include "eye_display.h"
#include "i2c_device.h"
#include "mcp_server.h"

#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <esp_lcd_gc9a01.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <cJSON.h>
#include <freertos/queue.h>
#include <cstring>
#include <string>

#define TAG "xiaoXiong_4G"

// The old bitmap eye animation is intentionally disabled.
// We now drive both panels with procedural eye rendering from `dual_eye_render.cc`.

static uint16_t s_black_line[DISPLAY_WIDTH] = {0};
static const int RX_BUF_SIZE = 1024;

namespace {

constexpr uint8_t kHostHead0 = '?';
constexpr uint8_t kHostHead1 = '!';
constexpr uint8_t kHostTail0 = 'A';
constexpr uint8_t kHostTail1 = 'A';

constexpr uint8_t kSlaveHead0 = '!';
constexpr uint8_t kSlaveHead1 = '?';
constexpr uint8_t kSlaveTail0 = 'B';
constexpr uint8_t kSlaveTail1 = 'B';

constexpr uint8_t kCmdServoControl = 0x01;
constexpr uint8_t kCmdServoAck = 0x00;
constexpr uint8_t kCmdSensorState = 0x01;
constexpr uint8_t kCmdBodyDetect = 0x03;

enum ToyServoId : uint8_t {
    kServoLeftLeg = 1,
    kServoRightLeg = 2,
    kServoTail = 3,
    kServoLeftArm = 4,
    kServoRightArm = 5,
    kServoHead = 6,
    kServoCount = 6,
};

enum SensorBit : uint8_t {
    kSensorLeftLeg = 0,
    kSensorRightLeg = 1,
    kSensorBelly = 2,
    kSensorLeftHand = 3,
    kSensorRightHand = 4,
    kSensorLeftCheek = 5,
    kSensorRightCheek = 6,
    kSensorForehead = 7,

};

enum MotionEasing : uint8_t {
    kMotionEasingLinear = 0,
    kMotionEasingEaseIn = 1,
    kMotionEasingEaseOut = 2,
    kMotionEasingEaseInOut = 3,
};

struct ServoAction {
    uint8_t servo_id;
    uint8_t center_angle;
    uint8_t swing_delta;
    uint16_t step_delay_ms;
    uint8_t repeats;
};

struct MotionSequenceStep {
    uint8_t servo_id;
    uint8_t center_angle;
    uint8_t swing_delta;
    uint8_t path_count;
    uint8_t path_angles[8];
    uint8_t target_angle;
    uint16_t step_delay_ms;
    uint16_t duration_ms;
    uint16_t frame_ms;
    uint8_t repeats;
    uint16_t hold_ms;
    uint8_t easing;
    bool smooth;
};

struct MotionSequence {
    uint8_t step_count;
    MotionSequenceStep steps[12];
};

static bool g_sensor_left_hand = false;
static bool g_sensor_right_hand = false;
static bool g_sensor_left_leg = false;
static bool g_sensor_right_leg = false;
static bool g_sensor_belly = false;
static bool g_sensor_forehead = false;
static bool g_sensor_left_cheek = false;
static bool g_sensor_right_cheek = false;
static uint8_t g_sensor_raw_bits = 0;
static bool g_body_detected = false;
static bool g_prev_body_detected = false;
// static QueueHandle_t g_servo_action_queue = nullptr;
static QueueHandle_t g_servo_action_queues[kServoCount] = {nullptr};
static TaskHandle_t g_servo_task_handles[kServoCount] = {nullptr};
static bool g_servo_busy[kServoCount] = {false};
static uint8_t g_servo_current_angle[kServoCount] = {90, 90, 90, 90, 90, 90};
static QueueHandle_t g_motion_sequence_queue = nullptr;
static TaskHandle_t g_motion_sequence_task_handle = nullptr;
static bool g_motion_sequence_busy = false;

static TaskHandle_t g_toy_sensor_task_handle = nullptr;
static uint8_t g_prev_sensor_bits = 0;
static int64_t g_sensor_last_trigger_us[8] = {0};
static int64_t g_body_detect_last_trigger_us = 0;
static int64_t g_last_sensor_log_time_us = 0;
static bool g_toy_uart_initialized = false;
static constexpr size_t kToyProtocolFrameSize = 8;
static constexpr int64_t kSensorLogIntervalUs = 5 * 1000 * 1000;
static constexpr int64_t kSensorDebounceUs = 500 * 1000;
static constexpr int64_t kToySensorStartupDelayUs = 5 * 1000 * 1000;
static constexpr int64_t kTouchPromptCooldownUs = 2 * 1000 * 1000;
static int64_t g_touch_prompt_last_trigger_us = 0;

size_t ServoIndex(uint8_t servo_id) { return static_cast<size_t>(servo_id - 1); }

bool IsValidServoId(uint8_t servo_id) {
    return servo_id >= kServoLeftLeg && servo_id <= kServoCount;
}

bool IsToySensorStartupDelayElapsed() { return esp_timer_get_time() >= kToySensorStartupDelayUs; }

void AppendTouchedPart(std::string& parts, const char* part) {
    if (!parts.empty()) {
        parts += "和";
    }
    parts += part;
}

std::string BuildTouchedPartsText(uint8_t sensor_bits) {
    std::string parts;
    if (sensor_bits & (1 << kSensorLeftLeg)) {
        AppendTouchedPart(parts, "左脚");
    }
    if (sensor_bits & (1 << kSensorRightLeg)) {
        AppendTouchedPart(parts, "右脚");
    }
    if (sensor_bits & (1 << kSensorLeftHand)) {
        AppendTouchedPart(parts, "左手");
    }
    if (sensor_bits & (1 << kSensorRightHand)) {
        AppendTouchedPart(parts, "右手");
    }
    if (sensor_bits & (1 << kSensorBelly)) {
        AppendTouchedPart(parts, "肚子");
    }
    if (sensor_bits & (1 << kSensorLeftCheek)) {
        AppendTouchedPart(parts, "左脸");
    }
    if (sensor_bits & (1 << kSensorRightCheek)) {
        AppendTouchedPart(parts, "右脸");
    }
    if (sensor_bits & (1 << kSensorForehead)) {
        AppendTouchedPart(parts, "额头");
    }
    return parts;
}

void SendTouchPromptToAssistant(const std::string& touched_parts) {
    if (touched_parts.empty()) {
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    if ((now_us - g_touch_prompt_last_trigger_us) < kTouchPromptCooldownUs) {
        ESP_LOGI(TAG, "touch prompt cooldown, ignore: %s", touched_parts.c_str());
        return;
    }
    g_touch_prompt_last_trigger_us = now_us;

    std::string prompt = "摸了你的";
    prompt += touched_parts;
    Application::GetInstance().SendTextToAssistant(prompt);
}

void SendBodyDetectPromptToAssistant() {
    Application::GetInstance().SendTextToAssistant("我靠近了你");
}

uint8_t ClampAngle(int angle) {
    if (angle < 0) {
        return 0;
    }
    if (angle > 180) {
        return 180;
    }
    return static_cast<uint8_t>(angle);
}

int ClampInt(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

int GetJsonInt(const cJSON* object, const char* primary_name, const char* fallback_name,
               int default_value, int min_value, int max_value) {
    const cJSON* item = cJSON_GetObjectItem(object, primary_name);
    if ((item == nullptr || !cJSON_IsNumber(item)) && fallback_name != nullptr) {
        item = cJSON_GetObjectItem(object, fallback_name);
    }
    if (!cJSON_IsNumber(item)) {
        return default_value;
    }
    return ClampInt(item->valueint, min_value, max_value);
}

bool ParseServoIdFromString(const char* servo_name, uint8_t* servo_id) {
    if (servo_name == nullptr || servo_id == nullptr) {
        return false;
    }

    if (strcmp(servo_name, "left_leg") == 0 || strcmp(servo_name, "左脚") == 0) {
        *servo_id = kServoLeftLeg;
    } else if (strcmp(servo_name, "right_leg") == 0 || strcmp(servo_name, "右脚") == 0) {
        *servo_id = kServoRightLeg;
    } else if (strcmp(servo_name, "tail") == 0 || strcmp(servo_name, "尾巴") == 0) {
        *servo_id = kServoTail;
    } else if (strcmp(servo_name, "head") == 0 || strcmp(servo_name, "头") == 0 ||
               strcmp(servo_name, "头部") == 0) {
        *servo_id = kServoHead;
    } else if (strcmp(servo_name, "left_arm") == 0 || strcmp(servo_name, "left_hand") == 0 ||
               strcmp(servo_name, "左手") == 0) {
        *servo_id = kServoLeftArm;
    } else if (strcmp(servo_name, "right_arm") == 0 || strcmp(servo_name, "right_hand") == 0 ||
               strcmp(servo_name, "右手") == 0) {
        *servo_id = kServoRightArm;
    } else {
        return false;
    }

    return true;
}

uint8_t ParseMotionEasing(const cJSON* item) {
    const cJSON* easing = cJSON_GetObjectItem(item, "easing");
    if (!cJSON_IsString(easing)) {
        easing = cJSON_GetObjectItem(item, "mode");
    }
    if (!cJSON_IsString(easing)) {
        return kMotionEasingLinear;
    }

    const char* value = easing->valuestring;
    if (strcmp(value, "ease_in") == 0 || strcmp(value, "accelerate") == 0 ||
        strcmp(value, "匀加速") == 0) {
        return kMotionEasingEaseIn;
    }
    if (strcmp(value, "ease_out") == 0 || strcmp(value, "decelerate") == 0 ||
        strcmp(value, "减速") == 0) {
        return kMotionEasingEaseOut;
    }
    if (strcmp(value, "ease_in_out") == 0 || strcmp(value, "smooth") == 0 ||
        strcmp(value, "缓动") == 0) {
        return kMotionEasingEaseInOut;
    }
    return kMotionEasingLinear;
}

bool IsSmoothMotionMode(const cJSON* item) {
    const cJSON* mode = cJSON_GetObjectItem(item, "mode");
    if (!cJSON_IsString(mode)) {
        return false;
    }

    const char* value = mode->valuestring;
    return strcmp(value, "move") == 0 || strcmp(value, "linear") == 0 ||
           strcmp(value, "ease") == 0 || strcmp(value, "ease_in") == 0 ||
           strcmp(value, "ease_out") == 0 || strcmp(value, "ease_in_out") == 0 ||
           strcmp(value, "smooth") == 0 || strcmp(value, "slow") == 0 ||
           strcmp(value, "匀速") == 0 || strcmp(value, "匀加速") == 0 ||
           strcmp(value, "慢慢动") == 0;
}

bool ParseMotionSequenceStep(const cJSON* item, MotionSequenceStep* step) {
    if (!cJSON_IsObject(item) || step == nullptr) {
        return false;
    }

    uint8_t servo_id = 0;
    const cJSON* servo_id_item = cJSON_GetObjectItem(item, "servo_id");
    if (cJSON_IsNumber(servo_id_item)) {
        servo_id = static_cast<uint8_t>(servo_id_item->valueint);
    } else {
        const cJSON* servo_item = cJSON_GetObjectItem(item, "servo");
        if (!cJSON_IsString(servo_item)) {
            servo_item = cJSON_GetObjectItem(item, "part");
        }
        if (!cJSON_IsString(servo_item) ||
            !ParseServoIdFromString(servo_item->valuestring, &servo_id)) {
            return false;
        }
    }

    if (!IsValidServoId(servo_id)) {
        return false;
    }

    step->servo_id = servo_id;
    step->center_angle = ClampAngle(GetJsonInt(item, "center_angle", "center", 90, 0, 180));
    step->swing_delta = static_cast<uint8_t>(GetJsonInt(item, "swing_delta", "swing", 20, 0, 60));
    step->target_angle =
        ClampAngle(GetJsonInt(item, "target_angle", "target", step->center_angle, 0, 180));
    const cJSON* target_item = cJSON_GetObjectItem(item, "target_angle");
    if (!cJSON_IsNumber(target_item)) {
        target_item = cJSON_GetObjectItem(item, "target");
    }
    if (cJSON_IsNumber(target_item)) {
        step->smooth = true;
    }
    const cJSON* angle_item = cJSON_GetObjectItem(item, "angle");
    if (cJSON_IsNumber(angle_item)) {
        step->target_angle = ClampAngle(angle_item->valueint);
        step->smooth = true;
    }

    const cJSON* path = cJSON_GetObjectItem(item, "angles");
    if (!cJSON_IsArray(path)) {
        path = cJSON_GetObjectItem(item, "path");
    }
    if (cJSON_IsArray(path)) {
        cJSON* angle = nullptr;
        cJSON_ArrayForEach (angle, path) {
            if (step->path_count >= sizeof(step->path_angles) / sizeof(step->path_angles[0])) {
                ESP_LOGW(TAG, "motion step path too long");
                return false;
            }
            if (!cJSON_IsNumber(angle)) {
                return false;
            }
            step->path_angles[step->path_count++] = ClampAngle(angle->valueint);
        }
    }
    step->smooth = step->smooth || IsSmoothMotionMode(item);
    const cJSON* smooth = cJSON_GetObjectItem(item, "smooth");
    if (cJSON_IsBool(smooth)) {
        step->smooth = cJSON_IsTrue(smooth);
    }
    step->step_delay_ms =
        static_cast<uint16_t>(GetJsonInt(item, "step_delay_ms", "delay_ms", 260, 120, 900));
    step->duration_ms =
        static_cast<uint16_t>(GetJsonInt(item, "duration_ms", "duration", 700, 100, 5000));
    step->frame_ms =
        static_cast<uint16_t>(GetJsonInt(item, "frame_ms", "interval_ms", 40, 20, 120));
    step->repeats = static_cast<uint8_t>(GetJsonInt(item, "repeats", "repeat", 1, 1, 6));
    step->hold_ms = static_cast<uint16_t>(GetJsonInt(item, "hold_ms", "pause_ms", 80, 0, 1500));
    step->easing = ParseMotionEasing(item);
    return true;
}

void SendServoCommand(uint8_t servo_id, uint8_t angle) {
    const uint8_t frame[8] = {
        kHostHead0, kHostHead1, kCmdServoControl, servo_id, angle, 0x00, kHostTail0, kHostTail1,
    };
    uart_write_bytes(TOY_SERVO_UART_PORT_NUM, frame, sizeof(frame));
    if (IsValidServoId(servo_id)) {
        g_servo_current_angle[ServoIndex(servo_id)] = angle;
    }
    ESP_LOGI(TAG, "servo id=%u angle=%u", servo_id, angle);
}

void EnqueueServoAction(uint8_t servo_id, uint8_t center_angle, uint8_t swing_delta,
                        uint16_t step_delay_ms, uint8_t repeats = 1) {
    if (!IsValidServoId(servo_id)) {
        return;
    }
    const size_t index = ServoIndex(servo_id);
    QueueHandle_t queue = g_servo_action_queues[index];
    if (queue == nullptr) {
        return;
    }
    if (g_motion_sequence_busy) {
        ESP_LOGI(TAG, "motion sequence busy, ignore servo id=%u action", servo_id);
        return;
    }
    if (g_servo_busy[index] || uxQueueMessagesWaiting(queue) > 0) {
        ESP_LOGI(TAG, "servo id=%u busy, ignore action", servo_id);
        return;
    }
    ServoAction action = {
        .servo_id = servo_id,
        .center_angle = center_angle,
        .swing_delta = swing_delta,
        .step_delay_ms = step_delay_ms,
        .repeats = repeats,
    };
    if (g_servo_busy[index] || uxQueueMessagesWaiting(queue) > 0) {
        xQueueOverwrite(queue, &action);
        ESP_LOGI(TAG, "servo id=%u busy, keep latest action", servo_id);
        return;
    }

    xQueueSend(queue, &action, 0);
}

bool RunPandaMotionAction(const std::string& action) {
    if (action == "greeting" || action == "hello" || action == "wave") {
        EnqueueServoAction(kServoHead, 90, 18, 260, 1);
        EnqueueServoAction(kServoRightArm, 90, 32, 240, 3);
        EnqueueServoAction(kServoTail, 90, 24, 220, 2);
        return true;
    }

    if (action == "move" || action == "wiggle") {
        EnqueueServoAction(kServoLeftLeg, 90, 20, 240, 2);
        EnqueueServoAction(kServoRightLeg, 90, 20, 240, 2);
        EnqueueServoAction(kServoLeftArm, 90, 24, 240, 2);
        EnqueueServoAction(kServoRightArm, 90, 24, 240, 2);
        EnqueueServoAction(kServoTail, 90, 28, 220, 3);
        return true;
    }

    if (action == "comfort" || action == "hug" || action == "soothe") {
        EnqueueServoAction(kServoHead, 90, 14, 380, 2);
        EnqueueServoAction(kServoLeftArm, 90, 22, 360, 1);
        EnqueueServoAction(kServoRightArm, 90, 22, 360, 1);
        return true;
    }

    if (action == "happy" || action == "mood_happy") {
        EnqueueServoAction(kServoHead, 90, 22, 220, 2);
        EnqueueServoAction(kServoLeftArm, 90, 30, 220, 2);
        EnqueueServoAction(kServoRightArm, 90, 30, 220, 2);
        EnqueueServoAction(kServoTail, 90, 30, 180, 4);
        return true;
    }

    if (action == "sad" || action == "mood_low" || action == "calm") {
        EnqueueServoAction(kServoHead, 82, 12, 520, 1);
        EnqueueServoAction(kServoTail, 84, 12, 520, 1);
        return true;
    }

    if (action == "curious" || action == "mood") {
        EnqueueServoAction(kServoHead, 90, 26, 300, 2);
        EnqueueServoAction(kServoTail, 90, 20, 300, 2);
        return true;
    }

    ESP_LOGW(TAG, "unknown panda motion action: %s", action.c_str());
    return false;
}

bool QueuePandaMotionSequence(const std::string& sequence_json) {
    cJSON* root = cJSON_Parse(sequence_json.c_str());
    if (!cJSON_IsArray(root)) {
        ESP_LOGW(TAG, "motion sequence must be a JSON array: %s", sequence_json.c_str());
        cJSON_Delete(root);
        return false;
    }

    MotionSequence sequence = {};
    cJSON* item = nullptr;
    cJSON_ArrayForEach (item, root) {
        if (sequence.step_count >= sizeof(sequence.steps) / sizeof(sequence.steps[0])) {
            ESP_LOGW(TAG, "motion sequence too long, max=%u",
                     static_cast<unsigned>(sizeof(sequence.steps) / sizeof(sequence.steps[0])));
            cJSON_Delete(root);
            return false;
        }

        MotionSequenceStep step = {};
        if (!ParseMotionSequenceStep(item, &step)) {
            ESP_LOGW(TAG, "invalid motion sequence step");
            cJSON_Delete(root);
            return false;
        }
        sequence.steps[sequence.step_count++] = step;
    }
    cJSON_Delete(root);

    if (sequence.step_count == 0 || g_motion_sequence_queue == nullptr) {
        return false;
    }
    if (g_motion_sequence_busy || uxQueueMessagesWaiting(g_motion_sequence_queue) > 0) {
        ESP_LOGI(TAG, "motion sequence busy, ignore new sequence");
        return false;
    }

    g_motion_sequence_busy = true;
    if (xQueueSend(g_motion_sequence_queue, &sequence, 0) != pdTRUE) {
        g_motion_sequence_busy = false;
        return false;
    }
    return true;
}

bool QueuePandaDanceMotionSequence(int intensity, int speed, int repeats) {
    intensity = ClampInt(intensity, 1, 5);
    speed = ClampInt(speed, 1, 5);
    repeats = ClampInt(repeats, 1, 3);

    const int arm_swing = 18 + intensity * 6;
    const int leg_swing = 10 + intensity * 3;
    const int head_swing = 10 + intensity * 3;
    const int tail_swing = 14 + intensity * 4;
    const int delay_ms = 420 - speed * 50;

    char sequence[768];
    snprintf(sequence, sizeof(sequence),
             "["
             "{\"servo\":\"left_arm\",\"angles\":[90,%d,%d,90],\"delay_ms\":%d,\"repeat\":%d},"
             "{\"servo\":\"right_arm\",\"angles\":[90,%d,%d,90],\"delay_ms\":%d,\"repeat\":%d},"
             "{\"servo\":\"left_leg\",\"swing\":%d,\"delay_ms\":%d,\"repeat\":%d},"
             "{\"servo\":\"right_leg\",\"swing\":%d,\"delay_ms\":%d,\"repeat\":%d},"
             "{\"servo\":\"head\",\"swing\":%d,\"delay_ms\":%d,\"repeat\":%d},"
             "{\"servo\":\"tail\",\"swing\":%d,\"delay_ms\":%d,\"repeat\":%d}"
             "]",
             ClampAngle(90 + arm_swing), ClampAngle(90 - arm_swing), delay_ms, repeats,
             ClampAngle(90 - arm_swing), ClampAngle(90 + arm_swing), delay_ms, repeats, leg_swing,
             delay_ms, repeats, leg_swing, delay_ms, repeats, head_swing, delay_ms, repeats,
             tail_swing, delay_ms - 40, repeats + 1);

    return QueuePandaMotionSequence(sequence);
}

bool QueuePandaPresetMotion(const std::string& action, int intensity, int speed, int repeats) {
    intensity = ClampInt(intensity, 1, 5);
    speed = ClampInt(speed, 1, 5);
    repeats = ClampInt(repeats, 1, 4);

    const int delay_ms = 420 - speed * 50;
    const int arm_swing = 18 + intensity * 6;
    const int head_swing = 10 + intensity * 4;
    const int tail_swing = 14 + intensity * 4;
    char sequence[768];

    if (action == "shake_hand" || action == "handshake" || action == "握手") {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo\":\"right_arm\",\"angles\":[90,%d,%d,%d,90],\"delay_ms\":%d,"
                 "\"repeat\":%d}]",
                 ClampAngle(90 + arm_swing), ClampAngle(90 - arm_swing / 2),
                 ClampAngle(90 + arm_swing - 6), delay_ms, repeats);
    } else if (action == "wave" || action == "挥手" || action == "move_right_arm" ||
               action == "move_right_hand" || action == "动右手") {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo\":\"right_arm\",\"angles\":[90,%d,%d,%d,90],\"delay_ms\":%d,"
                 "\"repeat\":%d}]",
                 ClampAngle(90 + arm_swing), ClampAngle(90 - arm_swing), ClampAngle(90 + arm_swing),
                 delay_ms, repeats);
    } else if (action == "move_left_arm" || action == "move_left_hand" || action == "动左手") {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo\":\"left_arm\",\"angles\":[90,%d,%d,%d,90],\"delay_ms\":%d,"
                 "\"repeat\":%d}]",
                 ClampAngle(90 + arm_swing), ClampAngle(90 - arm_swing), ClampAngle(90 + arm_swing),
                 delay_ms, repeats);
    } else if (action == "nod" || action == "动动头" || action == "点头") {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo\":\"head\",\"angles\":[90,%d,%d,90],\"delay_ms\":%d,\"repeat\":%d}]",
                 ClampAngle(90 - head_swing), ClampAngle(90 + head_swing), delay_ms, repeats);
    } else if (action == "shake_head" || action == "摇头") {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo\":\"head\",\"angles\":[90,%d,%d,%d,90],\"delay_ms\":%d,"
                 "\"repeat\":%d}]",
                 ClampAngle(90 + head_swing), ClampAngle(90 - head_swing),
                 ClampAngle(90 + head_swing / 2), delay_ms, repeats);
    } else if (action == "wag_tail" || action == "摇尾巴") {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo\":\"tail\",\"swing\":%d,\"delay_ms\":%d,\"repeat\":%d}]", tail_swing,
                 delay_ms, repeats);
    } else if (action == "raise_left_leg" || action == "抬左脚") {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo\":\"left_leg\",\"mode\":\"ease_in_out\",\"target\":%d,"
                 "\"duration_ms\":700},{\"servo\":\"left_leg\",\"mode\":\"ease_in_out\","
                 "\"target\":90,\"duration_ms\":700}]",
                 ClampAngle(90 + 10 + intensity * 5));
    } else if (action == "raise_right_leg" || action == "抬右脚") {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo\":\"right_leg\",\"mode\":\"ease_in_out\",\"target\":%d,"
                 "\"duration_ms\":700},{\"servo\":\"right_leg\",\"mode\":\"ease_in_out\","
                 "\"target\":90,\"duration_ms\":700}]",
                 ClampAngle(90 + 10 + intensity * 5));
    } else if (action == "move_left_leg" || action == "动左脚") {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo\":\"left_leg\",\"swing\":%d,\"delay_ms\":%d,\"repeat\":%d}]",
                 10 + intensity * 3, delay_ms, repeats);
    } else if (action == "move_right_leg" || action == "动右脚") {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo\":\"right_leg\",\"swing\":%d,\"delay_ms\":%d,\"repeat\":%d}]",
                 10 + intensity * 3, delay_ms, repeats);
    } else if (action == "dance" || action == "跳舞") {
        return QueuePandaDanceMotionSequence(intensity, speed, ClampInt(repeats, 1, 3));
    } else {
        ESP_LOGW(TAG, "unknown preset motion action: %s", action.c_str());
        return false;
    }

    return QueuePandaMotionSequence(sequence);
}

bool QueuePandaBodyMotion(const std::string& part, const std::string& motion, int intensity,
                          int speed, int repeats, int target_angle, int duration_ms) {
    uint8_t servo_id = 0;
    if (!ParseServoIdFromString(part.c_str(), &servo_id)) {
        ESP_LOGW(TAG, "unknown body motion part: %s", part.c_str());
        return false;
    }

    intensity = ClampInt(intensity, 1, 5);
    speed = ClampInt(speed, 1, 5);
    repeats = ClampInt(repeats, 1, 4);
    duration_ms = ClampInt(duration_ms, 100, 5000);

    const int swing = 8 + intensity * 6;
    const int delay_ms = 440 - speed * 55;
    const int frame_ms = speed >= 5 ? 25 : (speed >= 4 ? 35 : (speed >= 3 ? 45 : 65));
    const bool has_target = target_angle >= 0;
    const int default_target =
        (motion.find("down") != std::string::npos || motion.find("lower") != std::string::npos ||
         motion.find("低") != std::string::npos || motion.find("放下") != std::string::npos)
            ? 90 - swing
            : 90 + swing;
    const int target = target_angle < 0 ? default_target : target_angle;

    char sequence[768];
    if (has_target && motion.find("wave") == std::string::npos &&
        motion.find("shake") == std::string::npos && motion.find("握手") == std::string::npos &&
        motion.find("挥") == std::string::npos && motion.find("摆") == std::string::npos &&
        motion.find("点头") == std::string::npos && motion.find("nod") == std::string::npos) {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo_id\":%u,\"mode\":\"ease_in_out\",\"target\":%d,\"duration_ms\":%d,"
                 "\"frame_ms\":%d}]",
                 servo_id, ClampAngle(target), duration_ms, frame_ms);
    } else if (motion.find("wave") != std::string::npos ||
               motion.find("shake") != std::string::npos ||
               motion.find("握手") != std::string::npos || motion.find("挥") != std::string::npos ||
               motion.find("摆") != std::string::npos) {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo_id\":%u,\"angles\":[90,%d,%d,%d,90],\"delay_ms\":%d,"
                 "\"repeat\":%d}]",
                 servo_id, ClampAngle(90 + swing), ClampAngle(90 - swing),
                 ClampAngle(90 + swing / 2), delay_ms, repeats);
    } else if (motion.find("nod") != std::string::npos ||
               motion.find("点头") != std::string::npos) {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo_id\":%u,\"angles\":[90,%d,%d,90],\"delay_ms\":%d,"
                 "\"repeat\":%d}]",
                 servo_id, ClampAngle(90 - swing), ClampAngle(90 + swing), delay_ms, repeats);
    } else if (motion.find("smooth") != std::string::npos ||
               motion.find("slow") != std::string::npos ||
               motion.find("linear") != std::string::npos ||
               motion.find("ease") != std::string::npos || motion.find("慢") != std::string::npos ||
               motion.find("匀速") != std::string::npos ||
               motion.find("匀加速") != std::string::npos ||
               motion.find("缓") != std::string::npos || motion.find("抬") != std::string::npos ||
               motion.find("raise") != std::string::npos ||
               motion.find("lift") != std::string::npos) {
        const char* mode = "ease_in_out";
        if (motion.find("linear") != std::string::npos ||
            motion.find("匀速") != std::string::npos) {
            mode = "linear";
        } else if (motion.find("accelerate") != std::string::npos ||
                   motion.find("匀加速") != std::string::npos) {
            mode = "ease_in";
        } else if (motion.find("decelerate") != std::string::npos ||
                   motion.find("减速") != std::string::npos) {
            mode = "ease_out";
        }
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo_id\":%u,\"mode\":\"%s\",\"target\":%d,\"duration_ms\":%d,"
                 "\"frame_ms\":%d},{\"servo_id\":%u,\"mode\":\"%s\",\"target\":90,"
                 "\"duration_ms\":%d,\"frame_ms\":%d}]",
                 servo_id, mode, ClampAngle(target), duration_ms, frame_ms, servo_id, mode,
                 duration_ms, frame_ms);
    } else {
        snprintf(sequence, sizeof(sequence),
                 "[{\"servo_id\":%u,\"swing\":%d,\"delay_ms\":%d,\"repeat\":%d}]", servo_id, swing,
                 delay_ms, repeats);
    }

    return QueuePandaMotionSequence(sequence);
}

void UpdateToySensorVariables(uint8_t sensor_bits) {
    g_sensor_raw_bits = sensor_bits;
    g_sensor_left_leg = (sensor_bits >> kSensorLeftLeg) & 0x01;
    g_sensor_right_leg = (sensor_bits >> kSensorRightLeg) & 0x01;
    g_sensor_left_hand = (sensor_bits >> kSensorLeftHand) & 0x01;
    g_sensor_right_hand = (sensor_bits >> kSensorRightHand) & 0x01;
    g_sensor_belly = (sensor_bits >> kSensorBelly) & 0x01;
    g_sensor_left_cheek = (sensor_bits >> kSensorLeftCheek) & 0x01;
    g_sensor_right_cheek = (sensor_bits >> kSensorRightCheek) & 0x01;
    g_sensor_forehead = (sensor_bits >> kSensorForehead) & 0x01;
}

void HandleSensorRisingEdges(uint8_t rising_bits) {
    if (rising_bits & (1 << kSensorLeftLeg)) {
        EnqueueServoAction(kServoLeftLeg, 90, 22, 300, 2);
    }
    if (rising_bits & (1 << kSensorRightLeg)) {
        EnqueueServoAction(kServoRightLeg, 90, 22, 300, 2);
    }
    if (rising_bits & (1 << kSensorLeftHand)) {
        EnqueueServoAction(kServoLeftArm, 90, 28, 300, 2);
    }
    if (rising_bits & (1 << kSensorRightHand)) {
        EnqueueServoAction(kServoRightArm, 90, 28, 300, 2);
    }
    if (rising_bits & (1 << kSensorBelly)) {
        EnqueueServoAction(kServoLeftLeg, 90, 22, 300, 2);
        EnqueueServoAction(kServoRightLeg, 90, 22, 300, 2);
        EnqueueServoAction(kServoLeftArm, 90, 28, 300, 2);
        EnqueueServoAction(kServoRightArm, 90, 28, 300, 2);
    }
    if (rising_bits & (1 << kSensorLeftCheek)) {
        EnqueueServoAction(kServoHead, 90, 30, 300, 2);
        EnqueueServoAction(kServoTail, 90, 25, 300, 2);
    }
    if (rising_bits & (1 << kSensorRightCheek)) {
        EnqueueServoAction(kServoHead, 90, 30, 300, 2);
        EnqueueServoAction(kServoTail, 90, 25, 300, 2);
    }
    if (rising_bits & (1 << kSensorForehead)) {
        EnqueueServoAction(kServoHead, 90, 20, 300, 2);
        EnqueueServoAction(kServoTail, 90, 25, 300, 2);
    }

    SendTouchPromptToAssistant(BuildTouchedPartsText(rising_bits));
}

void HandleBodyDetectRisingEdge() {
    EnqueueServoAction(kServoHead, 90, 20, 300, 2);
    EnqueueServoAction(kServoTail, 90, 25, 300, 2);
    SendBodyDetectPromptToAssistant();
}

uint8_t FilterDebouncedRisingBits(uint8_t rising_bits) {
    if (rising_bits == 0) {
        return 0;
    }

    const int64_t now_us = esp_timer_get_time();
    uint8_t filtered_bits = 0;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        if ((rising_bits & (1 << bit)) == 0) {
            continue;
        }
        if ((now_us - g_sensor_last_trigger_us[bit]) < kSensorDebounceUs) {
            continue;
        }
        g_sensor_last_trigger_us[bit] = now_us;
        filtered_bits |= (1 << bit);
    }
    return filtered_bits;
}

void ProcessSlaveFrame(const uint8_t* frame) {
    if (frame[2] == kCmdSensorState) {
        const uint8_t sensor_bits = frame[3];
        const uint8_t raw_rising_bits = static_cast<uint8_t>((~g_prev_sensor_bits) & sensor_bits);
        const uint8_t rising_bits = FilterDebouncedRisingBits(raw_rising_bits);

        UpdateToySensorVariables(sensor_bits);
        if (rising_bits != 0) {
            ESP_LOGI(TAG, "sensor bits=0x%02X rising=0x%02X", sensor_bits, rising_bits);
            if (IsToySensorStartupDelayElapsed()) {
                HandleSensorRisingEdges(rising_bits);
            } else {
                ESP_LOGI(TAG, "ignore sensor action during startup delay");
            }
        }

        g_prev_sensor_bits = sensor_bits;
        return;
    }

    if (frame[2] == kCmdBodyDetect) {
        const int64_t now_us = esp_timer_get_time();
        const bool body_detected = (frame[3] & 0x01) != 0;
        const bool rising = body_detected && !g_prev_body_detected;
        if (body_detected != g_body_detected) {
            ESP_LOGI(TAG, "body detect changed: %u -> %u", g_body_detected ? 1 : 0,
                     body_detected ? 1 : 0);
        }
        g_body_detected = body_detected;
        g_prev_body_detected = body_detected;
        if (rising && (now_us - g_body_detect_last_trigger_us) >= kSensorDebounceUs) {
            g_body_detect_last_trigger_us = now_us;
            if (IsToySensorStartupDelayElapsed()) {
                ESP_LOGI(TAG, "body detect rising, trigger servo action");
                HandleBodyDetectRisingEdge();
            } else {
                ESP_LOGI(TAG, "ignore body detect action during startup delay");
            }
        }
        return;
    }

    if (frame[2] == kCmdServoAck) {
        ESP_LOGI(TAG,
                 "servo ack: fn=%02X id=%u angle=%u frame=%02X %02X %02X %02X %02X %02X %02X %02X",
                 frame[3], frame[4], frame[5], frame[0], frame[1], frame[2], frame[3], frame[4],
                 frame[5], frame[6], frame[7]);
        return;
    }

    ESP_LOGW(TAG, "unknown slave frame: %02X %02X %02X %02X %02X %02X %02X %02X", frame[0],
             frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7]);
}

void ToyServoMotionTask(void* arg) {
    const uint8_t servo_id = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(arg));
    if (!IsValidServoId(servo_id)) {
        vTaskDelete(nullptr);
        return;
    }

    const size_t index = ServoIndex(servo_id);
    QueueHandle_t queue = g_servo_action_queues[index];
    if (queue == nullptr) {
        vTaskDelete(nullptr);
        return;
    }

    ServoAction action = {};
    while (true) {
        if (xQueueReceive(queue, &action, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        g_servo_busy[index] = true;

        const int left_angle = action.center_angle - action.swing_delta;
        const int right_angle = action.center_angle + action.swing_delta;

        SendServoCommand(action.servo_id, ClampAngle(action.center_angle));
        vTaskDelay(pdMS_TO_TICKS(40));
        for (uint8_t i = 0; i < action.repeats; ++i) {
            SendServoCommand(action.servo_id, ClampAngle(right_angle));
            vTaskDelay(pdMS_TO_TICKS(action.step_delay_ms));
            SendServoCommand(action.servo_id, ClampAngle(left_angle));
            vTaskDelay(pdMS_TO_TICKS(action.step_delay_ms));
        }
        SendServoCommand(action.servo_id, ClampAngle(action.center_angle));
        g_servo_busy[index] = false;

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

bool WaitForServoIdle(uint8_t servo_id, uint32_t timeout_ms) {
    if (!IsValidServoId(servo_id)) {
        return false;
    }

    const int64_t deadline_us = esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000;
    const size_t index = ServoIndex(servo_id);
    while (g_servo_busy[index]) {
        if (esp_timer_get_time() >= deadline_us) {
            ESP_LOGW(TAG, "timeout waiting servo id=%u idle", servo_id);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return true;
}

int ApplyMotionEasing(uint8_t easing, int progress) {
    progress = ClampInt(progress, 0, 1000);
    switch (easing) {
        case kMotionEasingEaseIn:
            return (progress * progress) / 1000;
        case kMotionEasingEaseOut: {
            const int inverse = 1000 - progress;
            return 1000 - (inverse * inverse) / 1000;
        }
        case kMotionEasingEaseInOut:
            if (progress < 500) {
                return (2 * progress * progress) / 1000;
            } else {
                const int inverse = 1000 - progress;
                return 1000 - (2 * inverse * inverse) / 1000;
            }
        default:
            return progress;
    }
}

void SmoothMoveServo(uint8_t servo_id, uint8_t target_angle, uint16_t duration_ms,
                     uint16_t frame_ms, uint8_t easing) {
    const int start_angle = g_servo_current_angle[ServoIndex(servo_id)];
    const int target = ClampAngle(target_angle);
    const int frame_count = ClampInt(duration_ms / frame_ms, 1, 100);

    for (int frame = 1; frame <= frame_count; ++frame) {
        const int progress = (frame * 1000) / frame_count;
        const int eased = ApplyMotionEasing(easing, progress);
        const int angle = start_angle + ((target - start_angle) * eased) / 1000;
        SendServoCommand(servo_id, ClampAngle(angle));
        vTaskDelay(pdMS_TO_TICKS(frame_ms));
    }
}

void ExecuteMotionSequenceStep(const MotionSequenceStep& step) {
    if (!WaitForServoIdle(step.servo_id, 3000)) {
        return;
    }

    const size_t index = ServoIndex(step.servo_id);
    g_servo_busy[index] = true;

    if (step.path_count > 0 && step.smooth) {
        for (uint8_t repeat = 0; repeat < step.repeats; ++repeat) {
            for (uint8_t i = 0; i < step.path_count; ++i) {
                SmoothMoveServo(step.servo_id, step.path_angles[i], step.duration_ms, step.frame_ms,
                                step.easing);
            }
        }
    } else if (step.path_count > 0) {
        for (uint8_t repeat = 0; repeat < step.repeats; ++repeat) {
            for (uint8_t i = 0; i < step.path_count; ++i) {
                SendServoCommand(step.servo_id, step.path_angles[i]);
                vTaskDelay(pdMS_TO_TICKS(step.step_delay_ms));
            }
        }
    } else if (step.smooth) {
        for (uint8_t i = 0; i < step.repeats; ++i) {
            SmoothMoveServo(step.servo_id, step.target_angle, step.duration_ms, step.frame_ms,
                            step.easing);
        }
    } else {
        const int left_angle = step.center_angle - step.swing_delta;
        const int right_angle = step.center_angle + step.swing_delta;

        SendServoCommand(step.servo_id, ClampAngle(step.center_angle));
        vTaskDelay(pdMS_TO_TICKS(40));
        for (uint8_t i = 0; i < step.repeats; ++i) {
            SendServoCommand(step.servo_id, ClampAngle(right_angle));
            vTaskDelay(pdMS_TO_TICKS(step.step_delay_ms));
            SendServoCommand(step.servo_id, ClampAngle(left_angle));
            vTaskDelay(pdMS_TO_TICKS(step.step_delay_ms));
        }
        SendServoCommand(step.servo_id, ClampAngle(step.center_angle));
    }

    g_servo_busy[index] = false;

    if (step.hold_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(step.hold_ms));
    }
}

void ToyMotionSequenceTask(void* arg) {
    MotionSequence sequence = {};
    while (true) {
        if (xQueueReceive(g_motion_sequence_queue, &sequence, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        g_motion_sequence_busy = true;
        ESP_LOGI(TAG, "run motion sequence, steps=%u", sequence.step_count);
        for (uint8_t i = 0; i < sequence.step_count; ++i) {
            ExecuteMotionSequenceStep(sequence.steps[i]);
        }
        g_motion_sequence_busy = false;
    }
}

void ToySensorUartTask(void* arg) {
    uint8_t frame[kToyProtocolFrameSize] = {0};
    size_t index = 0;

    while (true) {
        uint8_t byte = 0;
        int len = uart_read_bytes(TOY_SERVO_UART_PORT_NUM, &byte, 1,
                                  pdMS_TO_TICKS(TOY_SERVO_RX_TIMEOUT_MS));
        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (index == 0) {
            if (byte == kSlaveHead0) {
                frame[index++] = byte;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (index == 1) {
            if (byte == kSlaveHead1) {
                frame[index++] = byte;
            } else {
                index = (byte == kSlaveHead0) ? 1 : 0;
                if (index == 1) {
                    frame[0] = byte;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        frame[index++] = byte;
        if (index >= kToyProtocolFrameSize) {
            if (frame[6] == kSlaveTail0 && frame[7] == kSlaveTail1) {
                const int64_t now_us = esp_timer_get_time();
                if (now_us - g_last_sensor_log_time_us >= kSensorLogIntervalUs) {
                    ESP_LOGI(TAG, "rx frame: %02X %02X %02X %02X %02X %02X %02X %02X", frame[0],
                             frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7]);
                    if (frame[2] == kCmdSensorState) {
                        const uint8_t sensor_bits = frame[3];
                        ESP_LOGI(TAG,
                                 "sensors: left_leg=%u right_leg=%u left_hand=%u right_hand=%u "
                                 "belly=%u left_cheek=%u right_cheek=%u forehead=%u",
                                 (sensor_bits >> kSensorLeftLeg) & 0x01,
                                 (sensor_bits >> kSensorRightLeg) & 0x01,
                                 (sensor_bits >> kSensorLeftHand) & 0x01,
                                 (sensor_bits >> kSensorRightHand) & 0x01,
                                 (sensor_bits >> kSensorBelly) & 0x01,
                                 (sensor_bits >> kSensorLeftCheek) & 0x01,
                                 (sensor_bits >> kSensorRightCheek) & 0x01,
                                 (sensor_bits >> kSensorForehead) & 0x01);
                    } else if (frame[2] == kCmdBodyDetect) {
                        ESP_LOGI(TAG, "body_detect=%u", frame[3] & 0x01);
                    }
                    g_last_sensor_log_time_us = now_us;
                }
                ProcessSlaveFrame(frame);
            }
            index = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

}  // namespace

static void ClearPanelToBlack(esp_lcd_panel_handle_t panel) {
    if (panel == nullptr) {
        return;
    }
    for (int row = 0; row < DISPLAY_HEIGHT; ++row) {
        esp_lcd_panel_draw_bitmap(panel, 0, row, DISPLAY_WIDTH, row + 1, s_black_line);
    }
}

class XiaoXiong4GBoard : public DualNetworkBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;

    class DirectImageDisplay : public Display {
    public:
        explicit DirectImageDisplay(esp_lcd_panel_handle_t* panels) : panels_(panels) {
            width_ = DISPLAY_WIDTH;
            height_ = DISPLAY_HEIGHT;
        }

        void SetupUI() override {
#if XIAOXIONG_ENABLE_EYE_RENDER
            if (task_update_eye_handler == nullptr) {
                xTaskCreatePinnedToCore(task_dual_eye_update, "task_eye_update", 1024 * 8, panels_,
                                        4, &task_update_eye_handler, 0);
            }
#else
            ClearPanelToBlack(panels_[0]);
            ClearPanelToBlack(panels_[1]);
            ESP_LOGW(TAG, "Eye render disabled for wake-word diagnostics");
#endif
        }

        void SetPowerSaveMode(bool on) override {}
        void SetStatus(const char* status) override {}
        void ShowNotification(const char* notification, int duration_ms = 3000) override {}
        void SetEmotion(const char* emotion) override {}
        void SetChatMessage(const char* role, const char* content) override {}
        void ClearChatMessages() override {}
        void UpdateStatusBar(bool update_all = false) override {}

    private:
        esp_lcd_panel_handle_t* panels_;
        bool Lock(int timeout_ms = 0) override { return true; }
        void Unlock() override {}
    };

    Display* display_;
    Button boot_button_;
    bool is_echo_base_connected_ = false;
    Esp32Camera* camera_ = nullptr;

    esp_lcd_panel_io_handle_t panel_io_1_ = nullptr;
    esp_lcd_panel_handle_t panel_1_ = nullptr;
    esp_lcd_panel_io_handle_t panel_io_2_ = nullptr;
    esp_lcd_panel_handle_t panel_2_ = nullptr;
    esp_lcd_panel_handle_t eye_panels_[2] = {nullptr, nullptr};

    void InitializeToyServoUart() {
        if (g_toy_uart_initialized) {
            return;
        }

        uart_config_t uart_config = {
            .baud_rate = TOY_SERVO_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_ERROR_CHECK(
            uart_driver_install(TOY_SERVO_UART_PORT_NUM, RX_BUF_SIZE * 2, 0, 0, nullptr, 0));
        ESP_ERROR_CHECK(uart_param_config(TOY_SERVO_UART_PORT_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(TOY_SERVO_UART_PORT_NUM, TOY_SERVO_UART_TXD,
                                     TOY_SERVO_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        g_toy_uart_initialized = true;
        ESP_LOGI(TAG, "Toy UART ready: port=%d baud=%d tx=%d rx=%d", TOY_SERVO_UART_PORT_NUM,
                 TOY_SERVO_UART_BAUD_RATE, TOY_SERVO_UART_TXD, TOY_SERVO_UART_RXD);
    }

    void InitializeToySensorServo() {
        InitializeToyServoUart();

        for (uint8_t servo_id = kServoLeftLeg; servo_id <= kServoCount; ++servo_id) {
            const size_t index = ServoIndex(servo_id);
            if (g_servo_action_queues[index] == nullptr) {
                g_servo_action_queues[index] = xQueueCreate(1, sizeof(ServoAction));
            }

            if (g_servo_action_queues[index] != nullptr && g_servo_task_handles[index] == nullptr) {
                xTaskCreatePinnedToCore(ToyServoMotionTask, "toy_servo_motion", 4096,
                                        reinterpret_cast<void*>(static_cast<uintptr_t>(servo_id)),
                                        2, &g_servo_task_handles[index], 1);
            }
        }

        if (g_motion_sequence_queue == nullptr) {
            g_motion_sequence_queue = xQueueCreate(1, sizeof(MotionSequence));
        }
        if (g_motion_sequence_queue != nullptr && g_motion_sequence_task_handle == nullptr) {
            xTaskCreatePinnedToCore(ToyMotionSequenceTask, "toy_motion_sequence", 4096, nullptr, 2,
                                    &g_motion_sequence_task_handle, 1);
        }

        if (g_toy_sensor_task_handle == nullptr) {
            xTaskCreatePinnedToCore(ToySensorUartTask, "toy_sensor_uart", 4096, nullptr, 2,
                                    &g_toy_sensor_task_handle, 1);
        }
    }

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // i2c_master_bus_config_t i2c_bus_cfg_ = {
        //     .i2c_port = I2C_NUM_0,
        //     .sda_io_num = CAMERA_PIN_SIOD,
        //     .scl_io_num = CAMERA_PIN_SIOC,
        //     .clk_source = I2C_CLK_SRC_DEFAULT,
        //     .glitch_ignore_cnt = 7,
        //     .intr_priority = 0,
        //     .trans_queue_depth = 0,
        //     .flags =
        //         {
        //             .enable_internal_pullup = 1,
        //         },
        // };
        // ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg_, &i2c_bus__));
    }

    void I2cDetect() {
        is_echo_base_connected_ = false;
        uint8_t echo_base_connected_flag = 0x00;
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                    if (address == 0x18) {
                        echo_base_connected_flag |= 0xF0;
                    } else if (address == 0x43) {
                        echo_base_connected_flag |= 0x0F;
                    }
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
        is_echo_base_connected_ = (echo_base_connected_flag == 0xFF);
    }

    void InitializeEyeSpi() {
        ESP_LOGI(TAG, "Initialize shared eye SPI bus");
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GC9A01_SPI1_LCD_GPIO_MOSI;
        buscfg.miso_io_num = GC9A01_SPI1_LCD_GPIO_MISO;
        buscfg.sclk_io_num = GC9A01_SPI1_LCD_GPIO_SCLK;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(GC9A01_LCD_SPI1_NUM, &buscfg, SPI_DMA_CH_AUTO));
    }

    void CreateGc9d01nDisplay1() {
        ESP_LOGI(TAG, "Init eye display 1");

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GC9A01_SPI1_LCD_GPIO_CS;
        io_config.dc_gpio_num = GC9A01_SPI1_LCD_GPIO_DC;
        io_config.spi_mode = 0;
        io_config.pclk_hz = GC9A01_LCD_PIXEL_CLK_HZ;
        io_config.trans_queue_depth = 1;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(GC9A01_LCD_SPI1_NUM, &io_config, &panel_io_1_));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GC9A01_SPI1_LCD_GPIO_RST;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9d01n(panel_io_1_, &panel_config, &panel_1_));

        lcd_io_eye = panel_io_1_;
        lcd_panel_eye = panel_1_;
    }

    void CreateGc9d01nDisplay2() {
        ESP_LOGI(TAG, "Init eye display 2");

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GC9A01_SPI2_LCD_GPIO_CS;
        io_config.dc_gpio_num = GC9A01_SPI2_LCD_GPIO_DC;
        io_config.spi_mode = 0;
        io_config.pclk_hz = GC9A01_LCD_PIXEL_CLK_HZ;
        io_config.trans_queue_depth = 1;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(GC9A01_LCD_SPI2_NUM, &io_config, &panel_io_2_));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GC9A01_SPI2_LCD_GPIO_RST;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9d01n(panel_io_2_, &panel_config, &panel_2_));

        lcd_io_eye2 = panel_io_2_;
        lcd_panel_eye2 = panel_2_;
    }

    void InitializeGc9d01nDisplays() {
        CreateGc9d01nDisplay1();
        CreateGc9d01nDisplay2();

        // Both eye panels share the same reset line. Reset once before either panel is initialized;
        // resetting after init would also reset the other CS device and stop its animation.
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_1_));

        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_1_));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_2_));

        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_1_, false));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_2_, false));
        // Panel 1 is mounted 90 degrees clockwise.
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_1_, true));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_1_, true, false));
        // Panel 2 is mounted 90 degrees counterclockwise.
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_2_, true));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_2_, false, true));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_1_, true));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_2_, true));
        ClearPanelToBlack(panel_1_);
        ClearPanelToBlack(panel_2_);
    }

    void InitializeDualDisplay() {
        InitializeEyeSpi();
        InitializeGc9d01nDisplays();

        eye_panels_[0] = panel_1_;
        eye_panels_[1] = panel_2_;
        display_ = new DirectImageDisplay(eye_panels_);
    }

    void InitializeCamera() {
#if XIAOXIONG_ENABLE_CAMERA
        camera_config_t camera_config = {
            .pin_pwdn = CAMERA_PIN_PWDN,
            .pin_reset = CAMERA_PIN_RESET,
            .pin_xclk = CAMERA_PIN_XCLK,
            .pin_sccb_sda = CAMERA_PIN_SIOD,
            .pin_sccb_scl = CAMERA_PIN_SIOC,
            .pin_d7 = CAMERA_PIN_D7,
            .pin_d6 = CAMERA_PIN_D6,
            .pin_d5 = CAMERA_PIN_D5,
            .pin_d4 = CAMERA_PIN_D4,
            .pin_d3 = CAMERA_PIN_D3,
            .pin_d2 = CAMERA_PIN_D2,
            .pin_d1 = CAMERA_PIN_D1,
            .pin_d0 = CAMERA_PIN_D0,
            .pin_vsync = CAMERA_PIN_VSYNC,
            .pin_href = CAMERA_PIN_HREF,
            .pin_pclk = CAMERA_PIN_PCLK,
            .xclk_freq_hz = XCLK_FREQ_HZ,
            .ledc_timer = LEDC_TIMER_0,
            .ledc_channel = LEDC_CHANNEL_0,

            .pixel_format = PIXFORMAT_RGB565,
            .frame_size = FRAMESIZE_QVGA,
            .jpeg_quality = 12,
            .fb_count = 1,
            .fb_location = CAMERA_FB_IN_PSRAM,
            .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
            // .sccb_i2c_port = I2C_NUM_0,
        };

        camera_ = new Esp32Camera(camera_config);
        if (camera_ != nullptr && camera_->IsAvailable()) {
            camera_->SetVFlip(true);
        } else {
            delete camera_;
            camera_ = nullptr;
            ESP_LOGW(TAG, "Camera initialization failed; camera tool disabled");
        }
#else
        ESP_LOGI(TAG, "Camera disabled");
#endif
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (GetNetworkType() == NetworkType::WIFI &&
                app.GetDeviceState() == kDeviceStateStarting) {
                auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
                wifi_board.EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting ||
                app.GetDeviceState() == kDeviceStateWifiConfiguring) {
                SwitchNetworkType();
            }
        });
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool(
            "self.system.reconfigure_wifi",
            "End this conversation and enter Wi-Fi configuration mode. Use this when the user "
            "asks to reconfigure Wi-Fi, reconnect Wi-Fi, change Wi-Fi, enter network setup, or "
            "says Chinese phrases like 重新配网、重新配置网络、进入配网模式、换个WiFi. "
            "Only call this when the device is already in Wi-Fi network mode. "
            "**CAUTION** You must ask the user to confirm this action before calling it.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                if (GetNetworkType() != NetworkType::WIFI) {
                    ESP_LOGW(TAG, "reconfigure wifi requested while not in WiFi mode");
                    return false;
                }

                auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
                wifi_board.EnterWifiConfigMode();
                return true;
            });

        mcp_server.AddTool(
            "self.panda.motion",
            "Control the panda companion toy's simple emotional servos. The model decides "
            "whether movement is needed from the user's intent. Call this tool when the user "
            "asks the panda to greet, move, comfort them, respond to mood/emotion words, or when "
            "Chinese phrases like 打招呼、你好、动一下、安慰我、抱抱、开心、难过、心情不好 appear. "
            "For body-specific actions, choose self.panda.body_motion or "
            "self.panda.motion_sequence instead of only speaking. "
            "Supported action values: greeting, move, comfort, happy, sad, curious.",
            PropertyList({Property("action", kPropertyTypeString)}),
            [](const PropertyList& properties) -> ReturnValue {
                const std::string& action = properties["action"].value<std::string>();
                return RunPandaMotionAction(action);
            });

        mcp_server.AddTool(
            "self.panda.body_motion",
            "Move one panda body part. The model should decide whether to call this from natural "
            "language, which part to move, how strongly, how fast, and whether to use a smooth "
            "motion. Use for requests like 动左手、动右手、握手、挥手、动动头、点头、摇头、"
            "摇尾巴、抬左脚、慢慢动头、匀速抬右手. "
            "part values: head, tail, left_arm, right_arm, left_leg, right_leg; Chinese also OK: "
            "头、尾巴、左手、右手、左脚、右脚. "
            "motion can be free text such as move, wave, shake, nod, raise, smooth, slow, linear, "
            "ease_in_out, 握手、挥手、摆动、点头、摇、抬、慢慢动、匀速、匀加速. "
            "Use target_angle -1 to let firmware choose a gentle angle; otherwise 0-180. "
            "For complex full-body or non-repeating dances, use self.panda.motion_sequence.",
            PropertyList({Property("part", kPropertyTypeString),
                          Property("motion", kPropertyTypeString),
                          Property("intensity", kPropertyTypeInteger, 3, 1, 5),
                          Property("speed", kPropertyTypeInteger, 3, 1, 5),
                          Property("repeats", kPropertyTypeInteger, 1, 1, 4),
                          Property("target_angle", kPropertyTypeInteger, -1, -1, 180),
                          Property("duration_ms", kPropertyTypeInteger, 700, 100, 5000)}),
            [](const PropertyList& properties) -> ReturnValue {
                return QueuePandaBodyMotion(
                    properties["part"].value<std::string>(),
                    properties["motion"].value<std::string>(), properties["intensity"].value<int>(),
                    properties["speed"].value<int>(), properties["repeats"].value<int>(),
                    properties["target_angle"].value<int>(),
                    properties["duration_ms"].value<int>());
            });

        mcp_server.AddTool(
            "self.panda.motion_sequence",
            "Run a custom panda servo motion sequence. Use this when the user asks for specific "
            "body actions "
            "such as 动动头、握手、摇尾巴、抬左脚、跳个舞, or asks how to move. The model may "
            "decide the "
            "servos, absolute angle paths, swing amount, speed, and repeats, but must stay gentle "
            "and safe. "
            "Argument `sequence` must be a compact JSON array string with 1 to 12 steps. "
            "Each step object fields: servo or part (head, tail, left_arm, right_arm, left_leg, "
            "right_leg; "
            "also accepts Chinese names 头、尾巴、左手、右手、左脚、右脚), center_angle or center "
            "(0-180, default 90), "
            "swing_delta or swing (0-60, default 20), step_delay_ms or delay_ms (120-900, default "
            "260), "
            "repeats or repeat (1-6, default 1), hold_ms or pause_ms (0-1500, default 80). "
            "For smooth controllable speed, use mode/easing (linear, ease_in, ease_out, "
            "ease_in_out), target_angle or target, duration_ms (100-5000), and frame_ms (20-120). "
            "For non-repetitive motions, use angles/path as an array of up to 8 absolute angles; "
            "add smooth:true to interpolate between points. "
            "Examples: slow nod "
            "[{\"servo\":\"head\",\"mode\":\"ease_in_out\",\"angles\":[90,65,105,90],"
            "\"duration_ms\":500,\"frame_ms\":40}], "
            "slow hand raise [{\"servo\":\"right_arm\",\"mode\":\"linear\",\"target\":130,"
            "\"duration_ms\":1200}], "
            "shake hand "
            "[{\"servo\":\"right_arm\",\"angles\":[90,125,80,118,90],\"delay_ms\":160}], "
            "dance: "
            "[{\"servo\":\"left_arm\",\"mode\":\"ease_in_out\",\"angles\":[90,125,70,110,90]},"
            "{\"servo\":\"right_arm\",\"angles\":[90,60,120,75,90]},"
            "{\"servo\":\"tail\",\"swing\":30,\"repeat\":3}].",
            PropertyList({Property("sequence", kPropertyTypeString)}),
            [](const PropertyList& properties) -> ReturnValue {
                const std::string& sequence = properties["sequence"].value<std::string>();
                return QueuePandaMotionSequence(sequence);
            });

        mcp_server.AddTool(
            "self.panda.preset_motion",
            "Run a reliable preset body action. Use this tool whenever the user asks for "
            "握手、挥手、"
            "动左手、动右手、动动头、点头、摇头、摇尾巴、动左脚、动右脚、抬左脚、抬右脚、跳舞, "
            "instead of only speaking. Prefer self.panda.body_motion when the user gives a custom "
            "part, speed, angle, or movement style. If the user says 动左手, call "
            "action=move_left_arm. "
            "If the user says 动右手, call action=move_right_arm. "
            "Supported action values: shake_hand, wave, nod, shake_head, wag_tail, raise_left_leg, "
            "raise_right_leg, move_left_arm, move_right_arm, move_left_leg, move_right_leg, dance. "
            "Arguments intensity 1-5, speed 1-5, repeats 1-4.",
            PropertyList({Property("action", kPropertyTypeString),
                          Property("intensity", kPropertyTypeInteger, 3, 1, 5),
                          Property("speed", kPropertyTypeInteger, 3, 1, 5),
                          Property("repeats", kPropertyTypeInteger, 2, 1, 4)}),
            [](const PropertyList& properties) -> ReturnValue {
                const std::string& action = properties["action"].value<std::string>();
                return QueuePandaPresetMotion(action, properties["intensity"].value<int>(),
                                              properties["speed"].value<int>(),
                                              properties["repeats"].value<int>());
            });

        mcp_server.AddTool(
            "self.panda.dance",
            "Make the panda dance by moving arms, legs, head, and tail together. Use this tool "
            "whenever the user asks 跳舞、跳个舞、舞蹈、dance. Prefer this over only speaking. "
            "Arguments: intensity 1-5 controls movement amplitude; speed 1-5 controls speed; "
            "repeats 1-3 controls dance length.",
            PropertyList({Property("intensity", kPropertyTypeInteger, 3, 1, 5),
                          Property("speed", kPropertyTypeInteger, 3, 1, 5),
                          Property("repeats", kPropertyTypeInteger, 2, 1, 3)}),
            [](const PropertyList& properties) -> ReturnValue {
                return QueuePandaDanceMotionSequence(properties["intensity"].value<int>(),
                                                     properties["speed"].value<int>(),
                                                     properties["repeats"].value<int>());
            });
    }

public:
    XiaoXiong4GBoard()
        : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, ML307_DTR_PIN, 0),
          boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        I2cDetect();
        InitializeCamera();
        InitializeDualDisplay();
        InitializeToySensorServo();
        InitializeTools();
        InitializeButtons();
    }

    // virtual AudioCodec* GetAudioCodec() override {
    //     static Es8311AudioCodec audio_codec(
    //         i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
    //         AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
    //         AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_GPIO_PA, AUDIO_CODEC_ES8311_ADDR, false);
    //     return &audio_codec;
    // }
    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_GPIO_PA, AUDIO_CODEC_ES8311_ADDR, false);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }

    virtual Camera* GetCamera() override { return camera_; }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual std::string GetDeviceStatusJson() override {
        std::string json = DualNetworkBoard::GetDeviceStatusJson();
        cJSON* root = cJSON_Parse(json.c_str());
        if (root == nullptr) {
            return json;
        }

        cJSON* toy_sensors = cJSON_CreateObject();
        cJSON_AddNumberToObject(toy_sensors, "raw_bits", g_sensor_raw_bits);
        cJSON_AddBoolToObject(toy_sensors, "left_leg", g_sensor_left_leg);
        cJSON_AddBoolToObject(toy_sensors, "right_leg", g_sensor_right_leg);
        cJSON_AddBoolToObject(toy_sensors, "left_hand", g_sensor_left_hand);
        cJSON_AddBoolToObject(toy_sensors, "right_hand", g_sensor_right_hand);
        cJSON_AddBoolToObject(toy_sensors, "belly", g_sensor_belly);
        cJSON_AddBoolToObject(toy_sensors, "left_cheek", g_sensor_left_cheek);
        cJSON_AddBoolToObject(toy_sensors, "right_cheek", g_sensor_right_cheek);
        cJSON_AddBoolToObject(toy_sensors, "forehead", g_sensor_forehead);
        cJSON_AddBoolToObject(toy_sensors, "body_detected", g_body_detected);
        cJSON_AddItemToObject(root, "toy_sensors", toy_sensors);

        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str != nullptr) {
            json.assign(json_str);
            cJSON_free(json_str);
        }
        cJSON_Delete(root);
        return json;
    }
};

DECLARE_BOARD(XiaoXiong4GBoard);
