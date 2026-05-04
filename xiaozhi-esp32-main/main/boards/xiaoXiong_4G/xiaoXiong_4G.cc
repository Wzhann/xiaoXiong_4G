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
    kServoHead = 4,
    kServoLeftArm = 5,
    kServoRightArm = 6,
    kServoCount = 6,
};

enum SensorBit : uint8_t {
    kSensorLeftLeg = 0,
    kSensorRightLeg = 1,
    kSensorLeftHand = 2,
    kSensorRightHand = 3,
    kSensorBelly = 4,
    kSensorLeftCheek = 5,
    kSensorRightCheek = 6,
    kSensorForehead = 7,
};

struct ServoAction {
    uint8_t servo_id;
    uint8_t center_angle;
    uint8_t swing_delta;
    uint16_t step_delay_ms;
    uint8_t repeats;
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
    return servo_id >= kServoLeftLeg && servo_id <= kServoRightArm;
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

void SendServoCommand(uint8_t servo_id, uint8_t angle) {
    const uint8_t frame[8] = {
        kHostHead0, kHostHead1, kCmdServoControl, servo_id, angle, 0x00, kHostTail0, kHostTail1,
    };
    uart_write_bytes(TOY_SERVO_UART_PORT_NUM, frame, sizeof(frame));
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

        for (uint8_t servo_id = kServoLeftLeg; servo_id <= kServoRightArm; ++servo_id) {
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
            "self.panda.motion",
            "Control the panda companion toy's servos. Call this tool when the user asks the "
            "panda to move, greet, wave, comfort them, respond to mood/emotion words, or when "
            "Chinese phrases like 打招呼、你好、动一下、安慰我、抱抱、开心、难过、心情不好 appear. "
            "Supported action values: greeting, move, comfort, happy, sad, curious.",
            PropertyList({Property("action", kPropertyTypeString)}),
            [](const PropertyList& properties) -> ReturnValue {
                const std::string& action = properties["action"].value<std::string>();
                return RunPandaMotionAction(action);
            });
    }

public:
    XiaoXiong4GBoard()
        : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, ML307_DTR_PIN),
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
