#include "action_executor.h"
#include "action_list.h"
#include "application.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "mpu6050.h"
#include "power_save_timer.h"
#include "wifi_board.h"
#include "wifi_configuration_ap.h"
#include "wifi_manager.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <cstdlib>

#define TAG "cat_tailAndNeck"

// Servo PWM config: 50Hz, 1MHz timer resolution, 13-bit duty
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_FREQ_HZ 50
#define SERVO_DUTY_RES LEDC_TIMER_13_BIT
#define SERVO_MAX_DUTY ((1 << 13) - 1)  // 8191
#define SERVO_PERIOD_US 20000           // 20ms

static constexpr int kServoCount = 5;
static constexpr ledc_channel_t kServoChannels[kServoCount] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};
static constexpr gpio_num_t kServoPins[kServoCount] = {SERVO_0_GPIO, SERVO_1_GPIO, SERVO_2_GPIO,
                                                       SERVO_3_GPIO, SERVO_4_GPIO};
static constexpr const char* kServoNames[kServoCount] = {"Neck0_IO15", "Neck1_IO16", "Tail0_IO17",
                                                         "Tail1_IO18", "Head_IO8"};

// Convert angle (0-180) to LEDC duty for 500-2500us pulse
static uint32_t AngleToDuty(int angle) {
    angle = std::clamp(angle, 0, 180);
    // pulse_us = 500 + angle * 2000 / 180
    uint32_t pulse_us = 500 + angle * 2000 / 180;
    return pulse_us * (SERVO_MAX_DUTY + 1) / SERVO_PERIOD_US;
}

class CatTailAndNeckBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    PowerSaveTimer* power_save_timer_ = nullptr;
    TaskHandle_t power_monitor_task_ = nullptr;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_handle_ = nullptr;
    int battery_level_ = 0;
    int battery_vbat_filtered_mv_ = 0;
    int servo_angle_[kServoCount] = {90, 90, 90, 90, 90};
    bool emote_enabled_ = true;
    // 3 independent executors — each group runs its own actions in parallel
    ActionExecutor neck_exec_{GROUP_NECK, "neck"};
    ActionExecutor tail_exec_{GROUP_TAIL, "tail"};
    ActionExecutor head_exec_{GROUP_HEAD, "head"};

    // ===== Touch button + MPU-6050 gyro =====
    TaskHandle_t touch_task_ = nullptr;
    bool mpu_ok_ = false;
    bool gyro_actions_enabled_ = true;  // MCP toggle for motion→action
    static constexpr int TOUCH_OVERSAMPLE = 256;
    static constexpr uint32_t TOUCH_DEBOUNCE_MS = 100;
    static constexpr uint32_t MOTION_COOLDOWN_MS = 1500;

    // Touch voltage windows (4-bit DAC: 16 levels, 206mV step)
    struct TouchLut {
        uint16_t lo, hi;
        uint8_t code;
    };
    static constexpr TouchLut kTouchLut[] = {
        {0, 130, 0x00},     {130, 290, 0x01},   {290, 480, 0x02},   {480, 670, 0x03},
        {670, 870, 0x04},   {870, 1070, 0x05},  {1070, 1280, 0x06}, {1280, 1490, 0x07},
        {1490, 1690, 0x08}, {1690, 1890, 0x09}, {1890, 2100, 0x0A}, {2100, 2310, 0x0B},
        {2310, 2510, 0x0C}, {2510, 2710, 0x0D}, {2710, 2910, 0x0E}, {2910, 3150, 0x0F},
    };
    static constexpr int kTouchLutN = sizeof(kTouchLut) / sizeof(kTouchLut[0]);

    int TouchLookup(int mv) {
        for (int i = 0; i < kTouchLutN; i++)
            if (mv >= kTouchLut[i].lo && mv <= kTouchLut[i].hi)
                return kTouchLut[i].code;
        return -1;
    }

    int TouchRead() {
        int64_t sum = 0;
        for (int i = 0; i < TOUCH_OVERSAMPLE; i++) {
            int raw;
            adc_oneshot_read(adc_handle_, ADC_CHANNEL_4, &raw);
            sum += raw;
        }
        int avg = sum / TOUCH_OVERSAMPLE;
        int mv;
        adc_cali_raw_to_voltage(adc_cali_handle_, avg, &mv);
        return TouchLookup(mv);
    }

    // Touch → action mapping (per group, triggered by touch)
    void OnTouch(int code) {
        if (code <= 0) return;
        if (code & 0x01) neck_exec_.Run("neck_nod", 0, 1);         // SW0 → neck
        if (code & 0x02) tail_exec_.Run("tail_wag", 0, 1);         // SW1 → tail
        if (code & 0x04) head_exec_.Run("head_nod", 0, 1);         // SW2 → head
        if (code & 0x08) head_exec_.Run("head_shake", 0, 1);       // SW3 → head
        if (code == 0x03) { StopAllActions(); RunAction("all_happy", 0, 1); }   // 0+1
        if (code == 0x0C) { StopAllActions(); RunAction("all_dance", 0, 1); }   // 2+3
    }

    // Motion detection task (touch + MPU-6050)
    void TouchMotionTask() {
        vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for system init
        ESP_LOGI(TAG, "Touch & motion task started (mpu=%s)", mpu_ok_ ? "OK" : "NONE");

        // Calibrate gyro bias now that system is stable
        if (mpu_ok_) {
            mpu6050_calibrate();
            ESP_LOGI(TAG, "Gyro calibrated, motion detection active");
        }

        uint32_t last_motion_ms = 0;
        int last_touch = -1, stable_cnt = 0, active_touch = -1;
        TickType_t last_wake = xTaskGetTickCount();

        while (true) {
            // 1. Read touch buttons (every 20ms)
            int touch = TouchRead();
            if (touch == last_touch) {
                stable_cnt++;
                if (active_touch < 0 && touch > 0 && stable_cnt >= 5) {
                    // New touch detected
                    active_touch = touch;
                    ESP_LOGI(TAG, "👆 Touch: code=0x%02X", touch);
                    OnTouch(touch);
                } else if (active_touch >= 0 && touch == 0 && stable_cnt >= 8) {
                    // Released
                    active_touch = -1;
                }
            } else {
                stable_cnt = 1;
                last_touch = touch;
            }

            // 2. MPU-6050: motion detect every 3rd cycle, raw log every 500ms
            static int tick = 0;
            tick++;
            if (mpu_ok_ && tick % 25 == 0) {
                mpu6050_print_raw();  // every ~500ms
            }
            if (mpu_ok_ && tick % 3 == 0) {
                mpu6050_motion_t motion = mpu6050_detect_motion();
                if (motion != MOTION_NONE) {
                    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (now - last_motion_ms > MOTION_COOLDOWN_MS) {
                        last_motion_ms = now;
                        ESP_LOGI(TAG, "🌀 Motion: %s | Orient: %s", mpu6050_motion_name(motion),
                                 mpu6050_orient_name(mpu6050_get_orientation()));
                        // Motion → action (when enabled)
                        if (gyro_actions_enabled_) {
                            switch (motion) {
                                case MOTION_PICKED_UP: RunAction("all_greeting", 0, 1); break;
                                case MOTION_SHAKEN:    RunAction("all_dance", 0, 1); break;
                                case MOTION_FLIPPED:   RunAction("all_curious", 0, 1); break;
                                case MOTION_PETTING:   neck_exec_.Run("neck_wave", 0, 2);
                                                       tail_exec_.Run("tail_wag", 0, 2); break;
                                case MOTION_DROPPED:   RunAction("all_sleep", 0, 1); break;
                                default: break;
                            }
                        }
                    }
                }
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    void InitializePowerManagement() {
        // Configure POWER_CTRL as output, set HIGH to latch power on
        gpio_config_t pwr_ctrl_cfg = {
            .pin_bit_mask = 1ULL << POWER_CTRL_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&pwr_ctrl_cfg);
        gpio_set_level(POWER_CTRL_GPIO, 1);  // Latch power on immediately
        ESP_LOGI(TAG, "POWER_CTRL (IO%d) set HIGH, power latched", POWER_CTRL_GPIO);

        // POWER_OUT (IO6) — GPIO input with pull-up, then hold to prevent MSPI override
        gpio_config_t pwr_out_cfg = {
            .pin_bit_mask = 1ULL << POWER_OUT_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&pwr_out_cfg);
        gpio_hold_en(POWER_OUT_GPIO);

        xTaskCreate(
            [](void* arg) {
                auto* board = static_cast<CatTailAndNeckBoard*>(arg);
                uint32_t press_count = 0;
                uint32_t release_count = 0;
                const uint32_t poll_interval_ms = 10;
                const uint32_t long_press_ticks = POWER_LONG_PRESS_MS / poll_interval_ms;
                const uint32_t debounce_ticks = 50 / poll_interval_ms;  // 50ms debounce

                // Wait 5s for IO6 to stabilize after power-up
                vTaskDelay(pdMS_TO_TICKS(5000));

                uint32_t tick = 0;
                while (true) {
                    // ADC read IO6, 1V threshold
                    int adc_raw = 0;
                    adc_oneshot_read(board->adc_handle_, ADC_CHANNEL_5, &adc_raw);
                    int adc_mv = board->AdcToMv(adc_raw);
                    bool pressed = (adc_mv < 1000);  // active-low: pressed when voltage < 1V
                    tick++;

                    // Every 10s: read battery ADC + log IO6
                    if (tick % 1000 == 0) {
                        ESP_LOGI(TAG, "POWER_OUT (IO6) adc=%dmV pressed=%d", adc_mv, pressed);

                        int bat_raw = 0;
                        adc_oneshot_read(board->adc_handle_, BATTERY_ADC_CHANNEL, &bat_raw);
                        int vpin_mv = board->AdcToMv(bat_raw);
                        int vbat_mv = static_cast<int>(vpin_mv * BATTERY_DIVIDER_RATIO);
                        if (board->battery_vbat_filtered_mv_ == 0) {
                            board->battery_vbat_filtered_mv_ = vbat_mv;
                        } else {
                            board->battery_vbat_filtered_mv_ +=
                                (vbat_mv - board->battery_vbat_filtered_mv_) / 5;
                        }
                        int vbat_f = board->battery_vbat_filtered_mv_;
                        int bat_level = (vbat_f - BATTERY_EMPTY_VOLTAGE_MV) * 100 /
                                        (BATTERY_FULL_VOLTAGE_MV - BATTERY_EMPTY_VOLTAGE_MV);
                        board->battery_level_ = std::clamp(bat_level, 0, 100);
                        ESP_LOGI(TAG, "Battery (IO3) raw=%dmV filtered=%dmV level=%d%%", vbat_mv,
                                 vbat_f, board->battery_level_);
                    }

                    if (pressed) {
                        press_count++;
                        release_count = 0;
                        if (press_count >= long_press_ticks) {
                            ESP_LOGW(TAG, "Power button long press detected (%dms), shutting down",
                                     POWER_LONG_PRESS_MS);
                            gpio_set_level(POWER_CTRL_GPIO, 0);
                        }
                    } else {
                        release_count++;
                        if (release_count >= debounce_ticks) {
                            press_count = 0;
                            release_count = debounce_ticks;
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
                }
            },
            "power_monitor", 2048, this, 1, &power_monitor_task_);
    }

    void InitializeCodecI2c() {
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
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));

        if (i2c_master_probe(codec_i2c_bus_, AUDIO_CODEC_ES8311_ADDR, pdMS_TO_TICKS(1000)) !=
            ESP_OK) {
            ESP_LOGE(TAG, "ES8311 not found on I2C bus — audio will not work");
        }
        I2cDetect();
    }

    void I2cDetect() {
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                uint8_t address = i + j;
                esp_err_t ret = i2c_master_probe(codec_i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(160, 300);
        power_save_timer_->SetEnabled(true);
    }

    void InitializeBatteryAdc() {
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id = ADC_UNIT_1,
            .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        esp_err_t ret = adc_oneshot_new_unit(&init_cfg, &adc_handle_);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ADC init failed, battery/power monitor disabled");
            return;
        }

        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };

        // Battery voltage channel (IO3 = ADC1_CH2)
        adc_oneshot_config_channel(adc_handle_, BATTERY_ADC_CHANNEL, &chan_cfg);

        // IO6 power button ADC channel (IO6 → ADC1_CH5)
        adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_5, &chan_cfg);

        // IO5 touch button ADC channel (IO5 → ADC1_CH4, 4-bit R-2R DAC)
        adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_4, &chan_cfg);

        // ADC calibration via curve fitting (eFuse VREF)
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_1,
            .chan = ADC_CHANNEL_5,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_handle_);
        if (cali_ret == ESP_OK) {
            ESP_LOGI(TAG, "ADC calibration enabled (curve fitting)");
        } else {
            ESP_LOGW(TAG, "ADC calibration skipped (ret=0x%x), using linear", cali_ret);
        }
    }

    int AdcToMv(int raw) const {
        if (adc_cali_handle_) {
            int mv = 0;
            if (adc_cali_raw_to_voltage(adc_cali_handle_, raw, &mv) == ESP_OK)
                return mv;
        }
        return raw * 3300 / 4096;  // fallback: 3.3V, 12-bit
    }

    static void SetServoAngle(int servo_index, int angle) {
        angle = std::clamp(angle, 0, 180);
        uint32_t duty = AngleToDuty(angle);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, kServoChannels[servo_index], duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, kServoChannels[servo_index]);
    }

    void InitializeServo() {
        // Power on servos via IO4
        gpio_config_t pwr_cfg = {
            .pin_bit_mask = 1ULL << SERVO_POWER_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&pwr_cfg);
        gpio_set_level(SERVO_POWER_GPIO, 1);
        ESP_LOGI(TAG, "Servo power ON (IO%d HIGH)", SERVO_POWER_GPIO);

        ledc_timer_config_t timer_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = SERVO_DUTY_RES,
            .timer_num = SERVO_TIMER,
            .freq_hz = SERVO_FREQ_HZ,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        esp_err_t ret = ledc_timer_config(&timer_cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "LEDC timer init failed, servo disabled");
            return;
        }

        for (int i = 0; i < kServoCount; i++) {
            ledc_channel_config_t ch_cfg = {
                .gpio_num = static_cast<int>(kServoPins[i]),
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = kServoChannels[i],
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = SERVO_TIMER,
                .duty = AngleToDuty(servo_angle_[i]),  // Start at 90°
                .hpoint = 0,
                .flags = {.output_invert = 0},
            };
            ret = ledc_channel_config(&ch_cfg);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Servo IO%d init failed, skipped", kServoPins[i]);
                continue;
            }
            ESP_LOGI(TAG, "Servo %d (%s) initialized at 90°", i, kServoNames[i]);
        }

        // Start all 3 action executors (each drives only its own group)
        neck_exec_.Start(SetServoAngle);
        tail_exec_.Start(SetServoAngle);
        head_exec_.Start(SetServoAngle);
    }

    // Route action name to the correct executor(s) based on prefix
    ActionExecutor* GetExecutorFor(const char* name) {
        const ServoAction* action = neck_exec_.FindAction(name);
        if (!action)
            return nullptr;
        if (action->group_mask == GROUP_NECK)
            return &neck_exec_;
        if (action->group_mask == GROUP_TAIL)
            return &tail_exec_;
        if (action->group_mask == GROUP_HEAD)
            return &head_exec_;
        return nullptr;  // GROUP_ALL handled separately (broadcast)
    }

    void RunOnAllGroups(const char* name, uint16_t speed, uint8_t cycles, bool loop) {
        // For combined (all_*) actions: submit to all 3 executors simultaneously
        neck_exec_.Run(name, speed, cycles, loop);
        tail_exec_.Run(name, speed, cycles, loop);
        head_exec_.Run(name, speed, cycles, loop);
    }

    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        // === Action tools ===

        mcp.AddTool("self.action.run",
                    "Run action by name or id. Each group independent — can overlap! "
                    "Neck(1001-6): left right nod shake wave loop. "
                    "Tail(2001-7): wag wag_fast up down tremble curl loop. "
                    "Head(3001-6): left right nod shake tilt loop. "
                    "All(4001-7): reset happy greeting sleep wake dance curious. "
                    "speed: ms/step (0=100). cycles: 次. loop: true=循环.",
                    PropertyList({Property("name", kPropertyTypeString, ""),
                                  Property("id", kPropertyTypeInteger, 0),
                                  Property("speed", kPropertyTypeInteger, 0),
                                  Property("cycles", kPropertyTypeInteger, 0),
                                  Property("loop", kPropertyTypeBoolean, false)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        const auto& name = props["name"].value<std::string>();
                        int id = props["id"].value<int>();
                        uint16_t speed = props["speed"].value<int>();
                        uint8_t cycles = props["cycles"].value<int>();
                        bool loop = props["loop"].value<bool>();

                        bool ok = (id > 0) ? RunActionById(id, speed, cycles, loop)
                                           : RunAction(name.c_str(), speed, cycles, loop);

                        char buf[64];
                        snprintf(buf, sizeof(buf), "%s: %s", ok ? "OK" : "FAIL",
                                 id > 0 ? std::to_string(id).c_str() : name.c_str());
                        return std::string(buf);
                    });

        mcp.AddTool("self.action.list", "List all action names.", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        std::string names;
                        for (int i = 0; i < kActionCount; i++) {
                            if (i > 0)
                                names += ", ";
                            names += kActionList[i].name;
                        }
                        return names;
                    });

        mcp.AddTool("self.action.stop", "Stop all running actions across all 3 groups.",
                    PropertyList(), [this](const PropertyList&) -> ReturnValue {
                        StopAllActions();
                        return std::string("OK: all stopped");
                    });

        mcp.AddTool("self.action.emote",
                    "Toggle emotion-driven spontaneous movement during chat. on=true/false.",
                    PropertyList({Property("on", kPropertyTypeBoolean, true)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        emote_enabled_ = props["on"].value<bool>();
                        ESP_LOGI(TAG, "Emote mode %s", emote_enabled_ ? "ON" : "OFF");
                        return true;
                    });

        // Battery status
        mcp.AddTool("self.battery.status", "Get battery voltage(mV) and level(%).", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        char buf[64];
                        snprintf(buf, sizeof(buf), "%dmV %d%%", battery_vbat_filtered_mv_,
                                 battery_level_);
                        ESP_LOGI(TAG, "Battery status: %s", buf);
                        return std::string(buf);
                    });

        // Emotion-driven spontaneous movement — randomly triggers across all 3 groups
        xTaskCreate(
            [](void* arg) {
                auto* board = static_cast<CatTailAndNeckBoard*>(arg);
                vTaskDelay(pdMS_TO_TICKS(10000));  // Wait for system init
                while (true) {
                    if (board->emote_enabled_) {
                        auto state = Application::GetInstance().GetDeviceState();
                        if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
                            if ((rand() % 10) == 0) {  // 10% chance every ~4s
                                // Randomly pick a group and trigger its emote
                                int g = rand() % 3;
                                if (g == 0)
                                    board->neck_exec_.PlayRandomEmote();
                                else if (g == 1)
                                    board->tail_exec_.PlayRandomEmote();
                                else
                                    board->head_exec_.PlayRandomEmote();
                            }
                        }
                        vTaskDelay(pdMS_TO_TICKS(4000 + (rand() % 4000)));
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
            },
            "emote_task", 2048, this, 1, nullptr);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

public:
    CatTailAndNeckBoard() : WifiBoard(), boot_button_(BOOT_BUTTON_GPIO) {
        esp_log_level_set("Display", ESP_LOG_NONE);
        InitializePowerManagement();  // Must be first: latch power, ADC init
        InitializeCodecI2c();
        InitializeButtons();

        // MPU-6050 (I2C_NUM_1: IO43=SDA, IO44=SCL)
        esp_log_level_set("mpu6050", ESP_LOG_INFO);  // Ensure gyro logs are visible
        mpu_ok_ = mpu6050_init();
        if (mpu_ok_) {
            ESP_LOGI(TAG, "MPU-6050 OK (IO43=SDA, IO44=SCL)");
        } else {
            ESP_LOGW(TAG, "MPU-6050 not found, motion disabled");
        }

        InitializeBatteryAdc();  // Also configures IO5 touch CH4
        InitializeServo();
        InitializePowerSaveTimer();
        InitializeTools();

        // Start touch + motion detection task
        xTaskCreate([](void* arg) { static_cast<CatTailAndNeckBoard*>(arg)->TouchMotionTask(); },
                    "touch_motion", 4096, this, 2, &touch_task_);
    }

    // ===== Action API (call from anywhere) =====
    bool RunActionById(uint16_t id, uint16_t speed = 0, uint8_t cycles = 1, bool loop = false) {
        const ServoAction* action = neck_exec_.FindActionById(id);
        if (!action) {
            ESP_LOGW(TAG, "Action id=%d not found", id);
            return false;
        }
        if (action->group_mask == GROUP_ALL)
            RunOnAllGroups(action->name, speed, cycles, loop);
        else {
            ActionExecutor* exec = GetExecutorFor(action->name);
            if (!exec)
                return false;
            exec->Run(action->name, speed, cycles, loop);
        }
        return true;
    }

    bool RunAction(const char* name, uint16_t speed = 0, uint8_t cycles = 1, bool loop = false) {
        const ServoAction* action = neck_exec_.FindAction(name);
        if (!action) {
            ESP_LOGW(TAG, "Action '%s' not found", name);
            return false;
        }
        if (action->group_mask == GROUP_ALL)
            RunOnAllGroups(name, speed, cycles, loop);
        else {
            ActionExecutor* exec = GetExecutorFor(name);
            if (!exec)
                return false;
            exec->Run(name, speed, cycles, loop);
        }
        return true;
    }

    void StopAllActions() {
        neck_exec_.Stop();
        tail_exec_.Stop();
        head_exec_.Stop();
    }

    // ===== Virtual overrides =====

    virtual void StartNetwork() override { WifiBoard::StartNetwork(); }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_GPIO_PA, AUDIO_CODEC_ES8311_ADDR, false);
        return &audio_codec;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        level = battery_level_;
        charging = false;
        discharging = true;
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(CatTailAndNeckBoard);
