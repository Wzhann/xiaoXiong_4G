#include "action_executor.h"
#include "action_list.h"
// #include "action_web.h"  // Web page disabled, use MCP for actions
#include "application.h"
#include "wifi_configuration_ap.h"
#include "wifi_manager.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "power_save_timer.h"
#include "wifi_board.h"

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

#define TAG "tuzi_SD"

// Servo PWM config: 50Hz, 1MHz timer resolution, 13-bit duty
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_FREQ_HZ 50
#define SERVO_DUTY_RES LEDC_TIMER_13_BIT
#define SERVO_MAX_DUTY ((1 << 13) - 1)  // 8191
#define SERVO_PERIOD_US 20000           // 20ms

static constexpr ledc_channel_t kServoChannels[2] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};
static constexpr gpio_num_t kServoPins[2] = {SERVO_A_GPIO, SERVO_B_GPIO};

// Convert angle (0-180) to LEDC duty for 500-2500us pulse
static uint32_t AngleToDuty(int angle) {
    angle = std::clamp(angle, 0, 180);
    // pulse_us = 500 + angle * 2000 / 180
    uint32_t pulse_us = 500 + angle * 2000 / 180;
    return pulse_us * (SERVO_MAX_DUTY + 1) / SERVO_PERIOD_US;
}

class TuziSDBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    PowerSaveTimer* power_save_timer_ = nullptr;
    TaskHandle_t power_monitor_task_ = nullptr;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_handle_ = nullptr;
    int battery_level_ = 0;
    int battery_vbat_filtered_mv_ = 0;
    int servo_angle_[2] = {90, 90};  // Current angles for servo A/B

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
                auto* board = static_cast<TuziSDBoard*>(arg);
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

        // IO6 power button ADC channel (IO6 → ADC1_CH5, per board mapping IO_x → CH_(x-1))
        adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_5, &chan_cfg);

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

        for (int i = 0; i < 2; i++) {
            ledc_channel_config_t ch_cfg = {
                .gpio_num = static_cast<int>(kServoPins[i]),
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = kServoChannels[i],
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = SERVO_TIMER,
                .duty = SERVO_SWEEP_ENABLED ? AngleToDuty(servo_angle_[i]) : 0,
                .hpoint = 0,
                .flags = {.output_invert = 0},
            };
            ret = ledc_channel_config(&ch_cfg);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Servo IO%d init failed, skipped", kServoPins[i]);
                continue;
            }
        }
        ESP_LOGI(TAG, "Servos initialized: IO%d, IO%d", kServoPins[0], kServoPins[1]);

        // Start 1Hz sweep task
        xTaskCreate([](void* arg) { static_cast<TuziSDBoard*>(arg)->ServoSweepTask(); },
                    "servo_sweep", 2048, this, 1, nullptr);

        // Start action executor (uses SetServoAngle as callback)
        action_executor_.Start(SetServoAngle);
    }

    bool servo_sweep_enabled_ = SERVO_SWEEP_ENABLED;
    int servo_sweep_cycles_ = 0;  // >0 = auto-stop after N cycles
    bool emote_enabled_ = true;
    ActionExecutor action_executor_;

    static void SetServoAngle(int servo_index, int angle) {
        angle = std::clamp(angle, 0, 180);
        uint32_t duty = AngleToDuty(angle);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, kServoChannels[servo_index], duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, kServoChannels[servo_index]);
    }

    void ServoSweepTask() {
        // 0.5Hz sweep: 0→180 (1000ms forward), 180→0 (1000ms backward)
        const int step_ms = 20;
        const int steps_per_half = 1000 / step_ms;  // 50 steps
        const float angle_per_step = 180.0f / steps_per_half;

        float angle = 90.0f;
        bool forward = true;
        int cycle_count = 0;
        ESP_LOGI(TAG, "Servo sweep started (0.5Hz)");

        while (true) {
            if (servo_sweep_enabled_) {
                int iangle = static_cast<int>(angle);
                SetServoAngle(0, iangle);
                SetServoAngle(1, iangle);
                vTaskDelay(pdMS_TO_TICKS(step_ms));

                if (forward) {
                    angle += angle_per_step;
                    if (angle >= 180.0f) { angle = 180.0f; forward = false; cycle_count++; }
                } else {
                    angle -= angle_per_step;
                    if (angle <= 0.0f) { angle = 0.0f; forward = true; cycle_count++; }
                }
                // Auto-stop after N cycles
                if (servo_sweep_cycles_ > 0 && cycle_count / 2 >= servo_sweep_cycles_) {
                    servo_sweep_enabled_ = false;
                    servo_sweep_cycles_ = 0;
                    cycle_count = 0;
                    ESP_LOGI(TAG, "Servo sweep stopped (cycles done)");
                }
            } else {
                cycle_count = 0;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }

    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self.servo.set_angle",
                    "Set servo angle. servo: \"A\" for IO15 or \"B\" for IO16. angle: 0-180.",
                    PropertyList({Property("servo", kPropertyTypeString),
                                  Property("angle", kPropertyTypeInteger, 90, 0, 180)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        const auto& servo = props["servo"].value<std::string>();
                        int angle = props["angle"].value<int>();
                        int index = -1;
                        if (servo == "A" || servo == "a")
                            index = 0;
                        if (servo == "B" || servo == "b")
                            index = 1;
                        if (index < 0) {
                            ESP_LOGW(TAG, "Unknown servo: %s (use A or B)", servo.c_str());
                            return false;
                        }
                        servo_angle_[index] = angle;
                        SetServoAngle(index, angle);
                        ESP_LOGI(TAG, "Servo %s → %d°", servo.c_str(), angle);
                        return true;
                    });

        mcp.AddTool("self.servo.reset", "Reset both servos to center position (90°).",
                    PropertyList(), [this](const PropertyList&) -> ReturnValue {
                        for (int i = 0; i < 2; i++) {
                            servo_angle_[i] = 90;
                            SetServoAngle(i, 90);
                        }
                        ESP_LOGI(TAG, "Both servos reset to 90°");
                        return true;
                    });

        mcp.AddTool("self.servo.sweep",
                    "Sweep ears 0.5Hz. on=true/false, cycles=3(default).",
                    PropertyList({Property("on", kPropertyTypeBoolean, true),
                                  Property("cycles", kPropertyTypeInteger, 3)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        servo_sweep_enabled_ = props["on"].value<bool>();
                        servo_sweep_cycles_ = props["cycles"].value<int>();
                        ESP_LOGI(TAG, "Servo sweep %s cycles=%d",
                                 servo_sweep_enabled_ ? "ON" : "OFF", servo_sweep_cycles_);
                        return true;
                    });

        mcp.AddTool("self.servo.power", "Set servo power. on=true (IO4 HIGH) / false (IO4 LOW).",
                    PropertyList({Property("on", kPropertyTypeBoolean, true)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        bool on = props["on"].value<bool>();
                        gpio_set_level(SERVO_POWER_GPIO, on ? 1 : 0);
                        ESP_LOGI(TAG, "Servo power %s (IO%d %s)", on ? "ON" : "OFF",
                                 SERVO_POWER_GPIO, on ? "HIGH" : "LOW");
                        return true;
                    });

        mcp.AddTool("self.battery.status", "Get battery voltage(mV) and level(%).", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        char buf[64];
                        snprintf(buf, sizeof(buf), "%dmV %d%%", battery_vbat_filtered_mv_,
                                 battery_level_);
                        ESP_LOGI(TAG, "Battery status: %s", buf);
                        return std::string(buf);
                    });

        mcp.AddTool("self.action.run",
                    "Play ear animation. MUST call when user wants ears to move! "
                    "For ANY request, call 2-3 DIFFERENT actions for variety, e.g. twist then swing then bounce. "
                    "左:left_up left_down left_wave left_nod left_tap left_shake left_flick left_mid; "
                    "右:right_up right_down right_wave right_nod right_tap right_shake right_flick right_mid; "
                    "双:heart ears_up ears_down both_sweep both_nod both_shake both_tiny both_tremble both_mid both_wide; "
                    "扭:swing swing_fast twist twist_fast; "
                    "跳:dance bounce cheer playful; "
                    "眨:wink_left wink_right wink_both; "
                    "情:think happy excited proud shy curious greeting; "
                    "态:sleep wake reset. speed=ms cycles=次.",
                    PropertyList({Property("name", kPropertyTypeString),
                                  Property("speed", kPropertyTypeInteger, 0),
                                  Property("cycles", kPropertyTypeInteger, 0)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        const auto& name = props["name"].value<std::string>();
                        int speed = props["speed"].value<int>();
                        int cycles = props["cycles"].value<int>();
                        bool ok = action_executor_.Run(name.c_str(), (uint16_t)speed, (uint8_t)cycles);
                        char buf[64];
                        snprintf(buf, sizeof(buf), "%s: %s", ok ? "OK" : "FAIL", name.c_str());
                        return std::string(buf);
                    });

        mcp.AddTool("self.action.list", "List all action names.", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        std::string names;
                        for (int i = 0; i < kActionCount; i++) {
                            if (i > 0) names += ", ";
                            names += kActionList[i].name;
                        }
                        return names;
                    });

        mcp.AddTool("self.action.stop", "Stop current action.", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        action_executor_.Stop();
                        return std::string("OK: stopped");
                    });

        mcp.AddTool("self.action.emote",
                    "Toggle emotion-driven ear movement during chat. on=true/false.",
                    PropertyList({Property("on", kPropertyTypeBoolean, true)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        emote_enabled_ = props["on"].value<bool>();
                        ESP_LOGI(TAG, "Emote mode %s", emote_enabled_ ? "ON" : "OFF");
                        return true;
                    });

        // Emotion-driven ear movement task
        xTaskCreate([](void* arg) {
            auto* board = static_cast<TuziSDBoard*>(arg);
            vTaskDelay(pdMS_TO_TICKS(10000));  // Wait for system init
            while (true) {
                if (board->emote_enabled_) {
                    auto state = Application::GetInstance().GetDeviceState();
                    // Move ears during listening/speaking (chat active)
                    if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
                        if ((rand() % 10) == 0) {  // 10% chance every ~4s
                            board->action_executor_.PlayRandomEmote();
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(4000 + (rand() % 4000)));  // Random 4-8s interval
                } else {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        }, "emote_task", 2048, this, 1, nullptr);

    }

    void InitializeButtons() {
#if XIAOZHI_CHAT_ENABLED
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
#endif
    }

public:
    TuziSDBoard() : WifiBoard(), boot_button_(BOOT_BUTTON_GPIO) {
        esp_log_level_set("Display", ESP_LOG_NONE);  // No physical display
        InitializePowerManagement();  // Must be first: latch power on
        InitializeCodecI2c();
        InitializeButtons();
        InitializeBatteryAdc();
        InitializeServo();
        InitializePowerSaveTimer();
        InitializeTools();
    }

    virtual void StartNetwork() override {
#if XIAOZHI_CHAT_ENABLED
        // Normal mode: auto-connect to saved WiFi, fallback to AP if none
        WifiBoard::StartNetwork();
#else
        // Debug mode: force AP hotspot, no WiFi connection
        auto& wifi_manager = WifiManager::GetInstance();
        WifiManagerConfig config;
        config.ssid_prefix = "Xiaozhi";
        wifi_manager.Initialize(config);
        wifi_manager.StartConfigAp();
        in_config_mode_ = true;
        Application::GetInstance().SetDeviceState(kDeviceStateWifiConfiguring);
        // Register action routes after HTTP server is ready
        xTaskCreate([](void* arg) {
            auto* board = static_cast<TuziSDBoard*>(arg);
            ESP_LOGI(TAG, "Action route task started, waiting for httpd...");
            for (int retry = 0; retry < 30; retry++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                auto* ap = WifiManager::GetInstance().GetConfigAp();
                auto* server = ap ? ap->GetHttpdHandle() : nullptr;
                ESP_LOGI(TAG, "ActionRoutes retry=%d ap=%p server=%p", retry, (void*)ap, (void*)server);
                if (server) {
                    RegisterActionRoutes(server, &board->action_executor_);
                    ESP_LOGI(TAG, "Action web routes registered on port 80");
                    break;
                }
            }
            ESP_LOGI(TAG, "Action route task exiting");
            vTaskDelete(nullptr);
        }, "action_routes", 3072, this, 1, nullptr);
        Application::GetInstance().Schedule([&wifi_manager]() {
            std::string hint = "手机连接热点 ";
            hint += wifi_manager.GetApSsid();
            hint += "，浏览器访问 ";
            hint += wifi_manager.GetApWebUrl();
            hint += "/action";
            Application::GetInstance().Alert("动作测试模式", hint.c_str(), "gear");
        });
#endif
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_GPIO_PA, AUDIO_CODEC_ES8311_ADDR, false);
        return &audio_codec;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        level = battery_level_;
        // No charging detection pin, assume discharging
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

DECLARE_BOARD(TuziSDBoard);
