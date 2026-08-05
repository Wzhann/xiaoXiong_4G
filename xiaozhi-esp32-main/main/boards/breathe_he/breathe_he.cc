#include "action_executor.h"
#include "action_list.h"
#include "application.h"
#include "assets/lang_config.h"
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

#define TAG "breathe_he"

// ===== Servo PWM config =====
// 270° servos: 500-2500μs pulse width
// User angle range: -135 (servo 0°) ~ 0 (servo 135°) ~ +135 (servo 270°)
// pulse_us = 1500 + user_angle * 1000 / 135
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_FREQ_HZ 50
#define SERVO_DUTY_RES LEDC_TIMER_13_BIT
#define SERVO_MAX_DUTY ((1 << 13) - 1)  // 8191
#define SERVO_PERIOD_US 20000           // 20ms
#define SERVO_MAX_ANGLE 135             // ±135° user range for 270° servos

static constexpr int kServoCount = 3;
static constexpr ledc_channel_t kServoChannels[kServoCount] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};
static constexpr gpio_num_t kServoPins[kServoCount] = {
    SERVO_0_GPIO, SERVO_1_GPIO, SERVO_2_GPIO};
static constexpr const char* kServoNames[kServoCount] = {
    "Head_IO15", "LeftHand_IO16", "RightHand_IO17"};

// Convert user angle (-135..+135) to LEDC duty for 500-2500μs pulse
static uint32_t AngleToDuty(int angle) {
    angle = std::clamp(angle, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
    // pulse_us = 1500 + angle * 1000 / 135  (center at 1500μs)
    int32_t pulse_us = 1500 + angle * 1000 / SERVO_MAX_ANGLE;
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    return pulse_us * (SERVO_MAX_DUTY + 1) / SERVO_PERIOD_US;
}

class BreatheHeBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    PowerSaveTimer* power_save_timer_ = nullptr;
    TaskHandle_t power_monitor_task_ = nullptr;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_handle_ = nullptr;
    int battery_level_ = 0;
    int battery_vbat_filtered_mv_ = 0;
    static int breathing_amplitude_pct_;  // 0-100, scales hand servo throw
    int servo_angle_[kServoCount] = {0, 0, 0};  // Neutral = 0° for 270° servos
    bool emote_enabled_ = true;
    TickType_t last_sound_tick_ = 0;
    static constexpr uint32_t SOUND_COOLDOWN_MS = 2500;

    // 4 independent executors — each group runs its own actions in parallel
    ActionExecutor head_exec_{GROUP_HEAD, "head"};
    ActionExecutor left_hand_exec_{GROUP_LEFT_HAND, "lh"};
    ActionExecutor right_hand_exec_{GROUP_RIGHT_HAND, "rh"};
    ActionExecutor hands_exec_{GROUP_HANDS, "hands"};

    // ===== Touch button + MPU-6050 gyro =====
    TaskHandle_t touch_task_ = nullptr;
    SemaphoreHandle_t adc_mutex_ = nullptr;
    bool mpu_ok_ = false;
    bool gyro_actions_enabled_ = true;
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
        bool got_lock = (xSemaphoreTake(adc_mutex_, pdMS_TO_TICKS(50)) == pdTRUE);
        if (!got_lock) {
            static int lock_fail_cnt = 0;
            if (++lock_fail_cnt % 100 == 1)
                ESP_LOGW(TAG, "TouchRead: ADC mutex timeout (#%d)", lock_fail_cnt);
            return -1;
        }
        for (int i = 0; i < TOUCH_OVERSAMPLE; i++) {
            int raw;
            adc_oneshot_read(adc_handle_, ADC_CHANNEL_4, &raw);
            sum += raw;
        }
        xSemaphoreGive(adc_mutex_);
        int avg = sum / TOUCH_OVERSAMPLE;
        int mv = 0;
        esp_err_t r = adc_cali_raw_to_voltage(adc_cali_handle_, avg, &mv);
        if (r != ESP_OK) {
            mv = avg * 3300 / 4096;
        }
        static int raw_log_tick = 0;
        if (++raw_log_tick % 50 == 0) {
            ESP_LOGI(TAG, "ADC raw avg=%d mv=%d code=%d", avg, mv, TouchLookup(mv));
        }
        return TouchLookup(mv);
    }

    // Touch → emotion → audio + action for panda
    //   SW0 (0x01) = 摸头     → 快乐 (happy)
    //   SW1 (0x02) = 摸背     → 舒服 (comfy)
    //   SW2 (0x04) = 摸左爪   → 好奇 (curious)
    //   SW3 (0x08) = 摸右爪   → 惊讶 (surprised)
    //   SW0+SW1    = 摸头+背  → 超级开心
    //   SW2+SW3    = 摸双爪   → 紧张 (nervous)
    //   SW0+SW2    = 摸头+左爪 → 撒娇 (affectionate)
    //   SW0+SW3    = 摸头+右爪 → 兴奋 (excited)
    //   SW1+SW2    = 摸背+左爪 → 困倦 (sleepy)
    //   SW1+SW3    = 摸背+右爪 → 平静 (calm)
    void OnTouch(int code) {
        if (code <= 0) return;

        // Combo lock: clear all queued actions, wait for running to finish
        head_exec_.ClearPending();
        left_hand_exec_.ClearPending();
        right_hand_exec_.ClearPending();
        hands_exec_.ClearPending();
        uint16_t waited = 0;
        while ((head_exec_.IsBusy() || left_hand_exec_.IsBusy() ||
                right_hand_exec_.IsBusy() || hands_exec_.IsBusy()) &&
               waited < 5000) {
            vTaskDelay(pdMS_TO_TICKS(20));
            waited += 20;
        }

        auto& app = Application::GetInstance();
        auto r = rand;
        auto pick = [&r](const char* const* list, int n) -> const char* {
            return list[r() % n];
        };
        TickType_t now = xTaskGetTickCount();
        bool can_play_sound = ((now - last_sound_tick_) * portTICK_PERIOD_MS) >= SOUND_COOLDOWN_MS;
        auto pickSound = [&](const std::string_view* list, int n) {
            if (can_play_sound) {
                auto& picked = list[r() % n];
                ESP_LOGI(TAG, "🔊 Touch sound: %.*s", (int)picked.size(), picked.data());
                app.PlaySound(picked);
                last_sound_tick_ = now;
            }
        };

        // Emotion audio pools
        static const std::string_view kHappySounds[] = {
            Lang::Sounds::OGG_EMOTION_HAPPY_1, Lang::Sounds::OGG_EMOTION_HAPPY_2,
            Lang::Sounds::OGG_EMOTION_HAPPY_3, Lang::Sounds::OGG_EMOTION_HAPPY_4,
            Lang::Sounds::OGG_EMOTION_HAPPY_5,
        };
        static const std::string_view kSadSounds[] = {
            Lang::Sounds::OGG_EMOTION_SAD_1, Lang::Sounds::OGG_EMOTION_SAD_2,
            Lang::Sounds::OGG_EMOTION_SAD_3, Lang::Sounds::OGG_EMOTION_SAD_4,
            Lang::Sounds::OGG_EMOTION_SAD_5,
        };
        static const std::string_view kAngrySounds[] = {
            Lang::Sounds::OGG_EMOTION_ANGRY_1, Lang::Sounds::OGG_EMOTION_ANGRY_2,
            Lang::Sounds::OGG_EMOTION_ANGRY_3, Lang::Sounds::OGG_EMOTION_ANGRY_4,
            Lang::Sounds::OGG_EMOTION_ANGRY_5,
        };
        static const std::string_view kFearSounds[] = {
            Lang::Sounds::OGG_EMOTION_FEAR_1, Lang::Sounds::OGG_EMOTION_FEAR_2,
            Lang::Sounds::OGG_EMOTION_FEAR_3, Lang::Sounds::OGG_EMOTION_FEAR_4,
            Lang::Sounds::OGG_EMOTION_FEAR_5,
        };
        static const std::string_view kSurpriseSounds[] = {
            Lang::Sounds::OGG_EMOTION_SURPRISE_1, Lang::Sounds::OGG_EMOTION_SURPRISE_2,
            Lang::Sounds::OGG_EMOTION_SURPRISE_3, Lang::Sounds::OGG_EMOTION_SURPRISE_4,
            Lang::Sounds::OGG_EMOTION_SURPRISE_5,
        };
        static const std::string_view kDisgustSounds[] = {
            Lang::Sounds::OGG_EMOTION_DISGUST_1, Lang::Sounds::OGG_EMOTION_ANGRY_2,
        };
        static const std::string_view kNeutralSounds[] = {
            Lang::Sounds::OGG_EMOTION_NEUTRAL_1, Lang::Sounds::OGG_EMOTION_NEUTRAL_2,
            Lang::Sounds::OGG_EMOTION_NEUTRAL_3,
        };
        // Purr/comfy → happy/neutral sounds
        static const std::string_view kComfySounds[] = {
            Lang::Sounds::OGG_EMOTION_HAPPY_3, Lang::Sounds::OGG_EMOTION_HAPPY_4,
            Lang::Sounds::OGG_EMOTION_HAPPY_5, Lang::Sounds::OGG_EMOTION_NEUTRAL_1,
        };

        constexpr int kHappyN = sizeof(kHappySounds)/sizeof(kHappySounds[0]);
        constexpr int kSadN = sizeof(kSadSounds)/sizeof(kSadSounds[0]);
        constexpr int kAngryN = sizeof(kAngrySounds)/sizeof(kAngrySounds[0]);
        constexpr int kFearN = sizeof(kFearSounds)/sizeof(kFearSounds[0]);
        constexpr int kSurpriseN = sizeof(kSurpriseSounds)/sizeof(kSurpriseSounds[0]);
        constexpr int kDisgustN = sizeof(kDisgustSounds)/sizeof(kDisgustSounds[0]);
        constexpr int kNeutralN = sizeof(kNeutralSounds)/sizeof(kNeutralSounds[0]);
        constexpr int kComfyN = sizeof(kComfySounds)/sizeof(kComfySounds[0]);

        switch (code) {
            case 0x01: { // 摸头 → 快乐: head nod/wave + breathe + happy sound
                static const char* heads[] = {
                    "head_nod", "head_left", "head_right",
                    "head_tilt_l", "head_tilt_r", "head_curious",
                };
                static const char* hands[] = {
                    "hands_breathe", "hands_sway", "hands_rest",
                };
                head_exec_.Run(pick(heads, 6), 0, 1);
                hands_exec_.Run(pick(hands, 3), 0, 1);
                pickSound(kHappySounds, kHappyN);
                break;
            }
            case 0x02: { // 摸背 → 舒服: calm head + slow breathe
                static const char* heads[] = {
                    "head_calm", "head_nod", "head_tilt_l", "head_tilt_r",
                };
                static const char* hands[] = {
                    "hands_breatheslow", "hands_sway", "hands_rest",
                };
                head_exec_.Run(pick(heads, 4), 0, 2);
                hands_exec_.Run(pick(hands, 3), 0, 2);
                pickSound(kComfySounds, kComfyN);
                break;
            }
            case 0x04: { // 摸左爪 → 好奇: curious head + left hand explore
                static const char* heads[] = {
                    "head_curious", "head_scan", "head_tilt_l", "head_left",
                };
                static const char* lh[] = {
                    "lh_lift", "lh_wave", "lh_crawl",
                };
                head_exec_.Run(pick(heads, 4), 0, 1);
                left_hand_exec_.Run(pick(lh, 3), 0, 1);
                pickSound(kSurpriseSounds, kSurpriseN);
                break;
            }
            case 0x08: { // 摸右爪 → 惊讶: quick look + right hand wave
                static const char* heads[] = {
                    "head_shake", "head_scan", "head_right", "head_curious",
                };
                static const char* rh[] = {
                    "rh_lift", "rh_wave", "rh_crawl",
                };
                head_exec_.Run(pick(heads, 4), 0, 1);
                right_hand_exec_.Run(pick(rh, 3), 0, 1);
                pickSound(kSurpriseSounds, kSurpriseN);
                break;
            }
            case 0x03: { // 摸头+背 → 超级开心: all happy dance
                StopAllActions();
                static const char* acts[] = {
                    "all_happy", "all_greeting", "all_curious",
                };
                RunAllAction(pick(acts, 3), 0, 1);
                pickSound(kHappySounds, kHappyN);
                break;
            }
            case 0x0C: { // 摸双爪 → 紧张: quick breath + head shake
                static const char* heads[] = {
                    "head_shake", "head_scan",
                };
                static const char* hands[] = {
                    "hands_breathe", "hands_crawlboth",
                };
                head_exec_.Run(pick(heads, 2), 0, 1);
                hands_exec_.Run(pick(hands, 2), 0, 1);
                pickSound(kFearSounds, kFearN);
                break;
            }
            case 0x05: { // 摸头+左爪 → 撒娇: tilt + gentle hand wave
                static const char* heads[] = {
                    "head_tilt_l", "head_nod", "head_curious",
                };
                static const char* lh[] = {
                    "lh_wave", "lh_lift",
                };
                head_exec_.Run(pick(heads, 3), 0, 2);
                left_hand_exec_.Run(pick(lh, 2), 0, 2);
                pickSound(kSadSounds, kSadN);
                break;
            }
            case 0x09: { // 摸头+右爪 → 兴奋: fast head shake + breathe fast
                static const char* heads[] = {
                    "head_shake", "head_right", "head_scan",
                };
                static const char* hands[] = {
                    "hands_crawlboth", "hands_breathe",
                };
                head_exec_.Run(pick(heads, 3), 0, 2);
                hands_exec_.Run(pick(hands, 2), 0, 2);
                pickSound(kAngrySounds, kAngryN);
                break;
            }
            case 0x06: { // 摸背+左爪 → 困倦: slow calm head + rest
                static const char* heads[] = {
                    "head_calm", "head_nod",
                };
                static const char* hands[] = {
                    "hands_rest", "hands_sway", "hands_breatheslow",
                };
                head_exec_.Run(pick(heads, 2), 0, 2);
                hands_exec_.Run(pick(hands, 3), 0, 2);
                pickSound(kDisgustSounds, kDisgustN);
                break;
            }
            case 0x0A: { // 摸背+右爪 → 平静: calm everything
                static const char* heads[] = {
                    "head_calm", "head_tilt_l", "head_tilt_r",
                };
                static const char* hands[] = {
                    "hands_rest", "hands_sway", "hands_breatheslow",
                };
                head_exec_.Run(pick(heads, 3), 0, 2);
                hands_exec_.Run(pick(hands, 3), 0, 2);
                pickSound(kNeutralSounds, kNeutralN);
                break;
            }
            default: { // 多点/其他 → 平静呼吸
                static const char* heads[] = {"head_calm", "head_nod"};
                static const char* hands[] = {"hands_breatheslow", "hands_rest"};
                head_exec_.Run(pick(heads, 2), 0, 1);
                hands_exec_.Run(pick(hands, 2), 0, 1);
                pickSound(kNeutralSounds, kNeutralN);
                break;
            }
        }
    }

    // Motion detection task (touch + MPU-6050)
    void TouchMotionTask() {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Touch & motion task started (mpu=%s)", mpu_ok_ ? "OK" : "NONE");

        if (mpu_ok_) {
            mpu6050_calibrate();
            ESP_LOGI(TAG, "Gyro calibrated, motion detection active");
        }

        uint32_t last_motion_ms = 0;
        int last_touch = -1, stable_cnt = 0, active_touch = -1;
        while (true) {
            // 1. Read touch buttons (every 20ms)
            int touch = TouchRead();

            static int dbg_tick = 0;
            dbg_tick++;
            if (dbg_tick % 50 == 0) {
                ESP_LOGI(TAG, "Touch dbg: read=%d last=%d stable=%d active=%d",
                         touch, last_touch, stable_cnt, active_touch);
            }

            if (touch < 0) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            if (touch == last_touch) {
                stable_cnt++;
                if (active_touch < 0 && touch > 0 && stable_cnt >= 5) {
                    active_touch = touch;
                    ESP_LOGI(TAG, "👆 Touch: code=0x%02X", touch);
                    OnTouch(touch);
                } else if (active_touch >= 0 && touch == 0 && stable_cnt >= 8) {
                    active_touch = -1;
                    ESP_LOGI(TAG, "Touch released");
                }
            } else {
                stable_cnt = 1;
                last_touch = touch;
            }

            // 2. MPU-6050: motion detect
            static int tick = 0;
            tick++;
            if (mpu_ok_ && tick % 150 == 0) {
                mpu6050_print_raw();
            }
            if (mpu_ok_ && tick % 3 == 0) {
                mpu6050_motion_t motion = mpu6050_detect_motion();
                if (motion != MOTION_NONE) {
                    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (now - last_motion_ms > MOTION_COOLDOWN_MS) {
                        last_motion_ms = now;
                        ESP_LOGI(TAG, "🌀 Motion: %s | Orient: %s",
                                 mpu6050_motion_name(motion),
                                 mpu6050_orient_name(mpu6050_get_orientation()));
                        if (gyro_actions_enabled_) {
                            auto pick = [](const char* const* list, int n) -> const char* {
                                return list[rand() % n];
                            };
                            auto pickSound = [](const std::string_view* list, int n) {
                                Application::GetInstance().PlaySound(list[rand() % n]);
                            };
                            static const std::string_view kCuriousSounds[] = {
                                Lang::Sounds::OGG_EMOTION_SURPRISE_1, Lang::Sounds::OGG_EMOTION_SURPRISE_2,
                                Lang::Sounds::OGG_EMOTION_SURPRISE_3, Lang::Sounds::OGG_EMOTION_SURPRISE_4,
                                Lang::Sounds::OGG_EMOTION_SURPRISE_5,
                            };
                            static const std::string_view kSurpriseSounds[] = {
                                Lang::Sounds::OGG_EMOTION_SURPRISE_1, Lang::Sounds::OGG_EMOTION_SURPRISE_2,
                                Lang::Sounds::OGG_EMOTION_FEAR_1, Lang::Sounds::OGG_EMOTION_FEAR_2,
                                Lang::Sounds::OGG_EMOTION_FEAR_3,
                            };
                            static const std::string_view kComfySounds[] = {
                                Lang::Sounds::OGG_EMOTION_HAPPY_3, Lang::Sounds::OGG_EMOTION_HAPPY_4,
                                Lang::Sounds::OGG_EMOTION_HAPPY_5, Lang::Sounds::OGG_EMOTION_NEUTRAL_1,
                            };
                            static const std::string_view kSadSounds[] = {
                                Lang::Sounds::OGG_EMOTION_SAD_1, Lang::Sounds::OGG_EMOTION_SAD_2,
                                Lang::Sounds::OGG_EMOTION_SAD_3,
                            };
                            static constexpr int kCN = 5, kSN = 5, kPN = 4, kSAD = 3;

                            switch (motion) {
                                case MOTION_PICKED_UP: {
                                    static const char* pk_head[] = {"head_curious","head_scan","head_nod","head_left","head_right"};
                                    static const char* pk_hands[] = {"hands_sway","hands_breathe","hands_rest"};
                                    head_exec_.Run(pick(pk_head, 5), 0, 1);
                                    hands_exec_.Run(pick(pk_hands, 3), 0, 1);
                                    pickSound(kCuriousSounds, kCN);
                                    break;
                                }
                                case MOTION_SHAKEN: {
                                    static const char* sk_head[] = {"head_shake","head_scan","head_right","head_left"};
                                    static const char* sk_hands[] = {"hands_crawlboth","hands_breathe"};
                                    head_exec_.Run(pick(sk_head, 4), 0, 1);
                                    hands_exec_.Run(pick(sk_hands, 2), 0, 1);
                                    pickSound(kSurpriseSounds, kSN);
                                    break;
                                }
                                case MOTION_FLIPPED: {
                                    static const char* fp_head[] = {"head_shake","head_scan","head_curious"};
                                    static const char* fp_hands[] = {"hands_crawlboth","hands_crawlalt","hands_sway"};
                                    head_exec_.Run(pick(fp_head, 3), 0, 1);
                                    hands_exec_.Run(pick(fp_hands, 3), 0, 1);
                                    pickSound(kSurpriseSounds, kSN);
                                    break;
                                }
                                case MOTION_PETTING: {
                                    static const char* pt_head[] = {"head_nod","head_calm","head_tilt_l","head_tilt_r"};
                                    static const char* pt_hands[] = {"hands_breatheslow","hands_sway","hands_rest"};
                                    head_exec_.Run(pick(pt_head, 4), 0, 2);
                                    hands_exec_.Run(pick(pt_hands, 3), 0, 2);
                                    pickSound(kComfySounds, kPN);
                                    break;
                                }
                                case MOTION_DROPPED: {
                                    static const char* dp_head[] = {"head_calm","head_tilt_l"};
                                    static const char* dp_hands[] = {"hands_rest","hands_breatheslow"};
                                    head_exec_.Run(pick(dp_head, 2), 0, 1);
                                    hands_exec_.Run(pick(dp_hands, 2), 0, 1);
                                    pickSound(kSadSounds, kSAD);
                                    break;
                                }
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
        gpio_config_t pwr_ctrl_cfg = {
            .pin_bit_mask = 1ULL << POWER_CTRL_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&pwr_ctrl_cfg);
        gpio_set_level(POWER_CTRL_GPIO, 1);
        ESP_LOGI(TAG, "POWER_CTRL (IO%d) set HIGH, power latched", POWER_CTRL_GPIO);

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
                auto* board = static_cast<BreatheHeBoard*>(arg);
                uint32_t press_count = 0;
                uint32_t release_count = 0;
                const uint32_t poll_interval_ms = 10;
                const uint32_t long_press_ticks = POWER_LONG_PRESS_MS / poll_interval_ms;
                const uint32_t debounce_ticks = 50 / poll_interval_ms;

                vTaskDelay(pdMS_TO_TICKS(5000));

                uint32_t tick = 0;
                while (true) {
                    int adc_raw = 0;
                    if (xSemaphoreTake(board->adc_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                        adc_oneshot_read(board->adc_handle_, ADC_CHANNEL_5, &adc_raw);
                        xSemaphoreGive(board->adc_mutex_);
                    }
                    int adc_mv = board->AdcToMv(adc_raw);
                    bool pressed = (adc_mv < 1000);
                    tick++;

                    if (tick % 1000 == 0) {
                        ESP_LOGI(TAG, "POWER_OUT (IO6) adc=%dmV pressed=%d", adc_mv, pressed);

                        int bat_raw = 0;
                        if (xSemaphoreTake(board->adc_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                            adc_oneshot_read(board->adc_handle_, BATTERY_ADC_CHANNEL, &bat_raw);
                            xSemaphoreGive(board->adc_mutex_);
                        }
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
                        ESP_LOGI(TAG, "Battery (IO3) raw=%dmV filtered=%dmV level=%d%%",
                                 vbat_mv, vbat_f, board->battery_level_);
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
            .flags = {.enable_internal_pullup = 1},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));

        if (i2c_master_probe(codec_i2c_bus_, AUDIO_CODEC_ES8311_ADDR, pdMS_TO_TICKS(1000)) != ESP_OK) {
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
                if (ret == ESP_OK)
                    printf("%02x ", address);
                else if (ret == ESP_ERR_TIMEOUT)
                    printf("UU ");
                else
                    printf("-- ");
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
        adc_mutex_ = xSemaphoreCreateMutex();

        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };

        // Battery voltage channel (IO3 = ADC1_CH2)
        adc_oneshot_config_channel(adc_handle_, BATTERY_ADC_CHANNEL, &chan_cfg);

        // IO6 power button ADC channel (IO6 → ADC1_CH5)
        adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_5, &chan_cfg);

        // IO5 touch button ADC channel (IO5 → ADC1_CH4, 4-bit R-2R DAC)
        gpio_config_t io5_cfg = {
            .pin_bit_mask = 1ULL << GPIO_NUM_5,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io5_cfg);
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
        return raw * 3300 / 4096;
    }

    static void SetServoAngle(int servo_index, int angle) {
        // angle is in user coordinate system (-135..+135)
        // For 270° servo: -135→500μs, 0→1500μs, +135→2500μs
        // Apply breathing amplitude to hand servos (index 1=left, 2=right)
        if (servo_index >= 1 && breathing_amplitude_pct_ != 100) {
            angle = angle * breathing_amplitude_pct_ / 100;
        }
        angle = std::clamp(angle, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
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
                .duty = AngleToDuty(servo_angle_[i]),  // Start at 0° (center)
                .hpoint = 0,
                .flags = {.output_invert = 0},
            };
            ret = ledc_channel_config(&ch_cfg);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Servo IO%d init failed, skipped", kServoPins[i]);
                continue;
            }
            ESP_LOGI(TAG, "Servo %d (%s) initialized at 0° (center)", i, kServoNames[i]);
        }

        // Start all 4 action executors
        head_exec_.Start(SetServoAngle);
        left_hand_exec_.Start(SetServoAngle);
        right_hand_exec_.Start(SetServoAngle);
        hands_exec_.Start(SetServoAngle);
    }

    // Route action name to the correct executor based on group mask
    ActionExecutor* GetExecutorFor(const char* name) {
        const ServoAction* action = head_exec_.FindAction(name);
        if (!action) return nullptr;
        if (action->group_mask == GROUP_HEAD) return &head_exec_;
        if (action->group_mask == GROUP_LEFT_HAND) return &left_hand_exec_;
        if (action->group_mask == GROUP_RIGHT_HAND) return &right_hand_exec_;
        if (action->group_mask == GROUP_HANDS) return &hands_exec_;
        return nullptr;  // GROUP_ALL handled separately
    }

    // Run a GROUP_ALL action on all 4 executors (each picks its own servos)
    void RunAllAction(const char* name, uint16_t speed, uint8_t cycles) {
        head_exec_.Run(name, speed, cycles);
        left_hand_exec_.Run(name, speed, cycles);
        right_hand_exec_.Run(name, speed, cycles);
        hands_exec_.Run(name, speed, cycles);
    }

    void RunOnAllGroups(const char* name, uint16_t speed, uint8_t cycles, bool loop) {
        head_exec_.Run(name, speed, cycles, loop);
        left_hand_exec_.Run(name, speed, cycles, loop);
        right_hand_exec_.Run(name, speed, cycles, loop);
        hands_exec_.Run(name, speed, cycles, loop);
    }

    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        // === Action tools ===
        mcp.AddTool("self.action.run",
                    "Run ONE action by name or id. Call multiple times to combine groups! "
                    "speed: ms/step (0=100=1s/action). cycles: repeat count. loop: true=continuous.\n"
                    "Head(1001-1009) 头: left/right(左/右转) nod(点头) shake(摇头) "
                    "tilt_l/tilt_r(左/右歪) scan(扫视) curious(好奇) calm(平静).\n"
                    "LeftHand(2001-2004) 左手: crawl(爬行下压) breathe(压缩气囊) lift(轻抬) wave(挥手).\n"
                    "RightHand(3001-3004) 右手: crawl(爬行下压) breathe(压缩气囊) lift(轻抬) wave(挥手).\n"
                    "Hands(4001-4006) 双手: breathe(同时呼吸) breatheslow(慢呼吸) "
                    "crawlboth(同时爬) crawlalt(交替爬) rest(回中) sway(摇摆).\n"
                    "All(5001-5010) 全身: reset(回中) breathe(呼吸+点头) crawl(爬行) "
                    "happy(开心) curious(好奇) greeting(问候) sleep(入睡) wake(醒来) "
                    "stretch(伸懒腰) bow(鞠躬).",
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

        mcp.AddTool("self.action.combo",
                    "Run head+hands actions TOGETHER! Smart combo for natural commands.\n"
                    "Examples: 呼吸→hands=hands_breathe 开心→head=head_nod+hands=hands_sway "
                    "爬行→hands=hands_crawlalt 好奇→head=head_curious "
                    "睡觉→head=head_calm+hands=hands_rest "
                    "打招呼→head=head_nod+hands=hands_breathe 等等任意组合!"
                    "Leave a group empty to skip.",
                    PropertyList({Property("head", kPropertyTypeString, ""),
                                  Property("hands", kPropertyTypeString, ""),
                                  Property("speed", kPropertyTypeInteger, 0),
                                  Property("cycles", kPropertyTypeInteger, 0),
                                  Property("loop", kPropertyTypeBoolean, false)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        const auto& head = props["head"].value<std::string>();
                        const auto& hands = props["hands"].value<std::string>();
                        uint16_t speed = props["speed"].value<int>();
                        uint8_t cycles = props["cycles"].value<int>();
                        bool loop = props["loop"].value<bool>();
                        int count = 0;

                        if (!head.empty()) {
                            head_exec_.Run(head.c_str(), speed, cycles, loop);
                            count++;
                        }
                        if (!hands.empty()) {
                            hands_exec_.Run(hands.c_str(), speed, cycles, loop);
                            count++;
                        }

                        if (count == 0) return std::string("FAIL: no group specified");
                        char buf[32];
                        snprintf(buf, sizeof(buf), "OK: %d groups", count);
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

        mcp.AddTool("self.action.stop", "Stop all running actions across all groups.",
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

        // Breathing control
        // hands_breathe = 28 steps, default 100ms/step → 2800ms/cycle → ~21.4 BPM
        // frequency → speed: speed_ms = 60000 / (freq * 28)
        // Clamp: 5–60 BPM → speed 36–429 ms
        // amplitude: 0-100% of full hand throw (100=±90°, 50=±45°)
        static constexpr int kBreathingSteps = 28;  // hands_breathe steps
        mcp.AddTool("self.breathing.control",
                    "Control panda breathing. Default ~21 breaths/min, 100% amplitude.\n"
                    "cmd: start(loop) / stop / once / slow(loop, wide+slow).\n"
                    "frequency: breaths per minute (5-60), overrides speed.\n"
                    "amplitude: hand throw % (1-100, default 100). 100=full ±90°, 50=gentle ±45°.\n"
                    "speed: ms/step (0=100ms), lower=faster. Only used if frequency=0.",
                    PropertyList({Property("cmd", kPropertyTypeString, "start"),
                                  Property("frequency", kPropertyTypeInteger, 0),
                                  Property("amplitude", kPropertyTypeInteger, 100),
                                  Property("speed", kPropertyTypeInteger, 0)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        const auto& cmd = props["cmd"].value<std::string>();
                        int speed = props["speed"].value<int>();
                        int freq = props["frequency"].value<int>();
                        int amp = props["amplitude"].value<int>();
                        // Apply amplitude (hand throw scaling, 1-100%)
                        breathing_amplitude_pct_ = std::clamp(amp, 1, 100);
                        // frequency takes priority over speed
                        if (freq > 0) {
                            freq = std::clamp(freq, 5, 60);
                            speed = 60000 / (freq * kBreathingSteps);
                            ESP_LOGI(TAG, "Breathing freq=%d BPM amp=%d%% → speed=%d ms/step", freq,
                                     breathing_amplitude_pct_, speed);
                        } else {
                            ESP_LOGI(TAG, "Breathing amp=%d%% speed=%d", breathing_amplitude_pct_, speed);
                        }
                        char buf[64];
                        if (cmd == "start") {
                            hands_exec_.Run("hands_breathe", speed, 0, true);  // loop
                            head_exec_.Run("head_nod", speed, 0, true);
                            if (freq > 0)
                                snprintf(buf, sizeof(buf), "OK: breathing %d BPM amp=%d%%", freq,
                                         breathing_amplitude_pct_);
                            else
                                snprintf(buf, sizeof(buf), "OK: breathing amp=%d%%", breathing_amplitude_pct_);
                            return std::string(buf);
                        } else if (cmd == "stop") {
                            hands_exec_.Stop();
                            head_exec_.Stop();
                            return std::string("OK: breathing stopped");
                        } else if (cmd == "once") {
                            hands_exec_.Run("hands_breathe", speed, 1);
                            if (freq > 0)
                                snprintf(buf, sizeof(buf), "OK: one breath @ %d BPM amp=%d%%", freq,
                                         breathing_amplitude_pct_);
                            else
                                snprintf(buf, sizeof(buf), "OK: one breath amp=%d%%", breathing_amplitude_pct_);
                            return std::string(buf);
                        } else if (cmd == "slow") {
                            hands_exec_.Run("hands_breatheslow", speed, 0, true);
                            return std::string("OK: slow breathing");
                        }
                        return std::string("FAIL: unknown cmd");
                    });

        // Crawling control
        mcp.AddTool("self.crawling.control",
                    "Control panda crawling: start/stop/step. "
                    "alt=alternating hands, both=both hands together.",
                    PropertyList({Property("cmd", kPropertyTypeString, "start"),
                                  Property("mode", kPropertyTypeString, "alt")}),
                    [this](const PropertyList& props) -> ReturnValue {
                        const auto& cmd = props["cmd"].value<std::string>();
                        const auto& mode = props["mode"].value<std::string>();
                        if (cmd == "start") {
                            if (mode == "alt") {
                                hands_exec_.Run("hands_crawlalt", 0, 0, true);
                            } else {
                                hands_exec_.Run("hands_crawlboth", 0, 0, true);
                            }
                            head_exec_.Run("head_nod", 0, 0, true);
                            return std::string("OK: crawling started");
                        } else if (cmd == "stop") {
                            hands_exec_.Stop();
                            head_exec_.Stop();
                            return std::string("OK: crawling stopped");
                        } else if (cmd == "step") {
                            if (mode == "alt")
                                hands_exec_.Run("hands_crawlalt", 0, 1);
                            else
                                hands_exec_.Run("hands_crawlboth", 0, 1);
                            return std::string("OK: one crawl step");
                        }
                        return std::string("FAIL: unknown cmd");
                    });

        // Emotion-driven spontaneous movement during chat
        xTaskCreate(
            [](void* arg) {
                auto* board = static_cast<BreatheHeBoard*>(arg);
                vTaskDelay(pdMS_TO_TICKS(10000));
                while (true) {
                    if (board->emote_enabled_) {
                        auto state = Application::GetInstance().GetDeviceState();
                        if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
                            if ((rand() % 10) == 0) {
                                int g = rand() % 6;
                                if (g == 0)
                                    board->head_exec_.PlayRandomEmote();
                                else if (g == 1)
                                    board->left_hand_exec_.PlayRandomEmote();
                                else if (g == 2)
                                    board->right_hand_exec_.PlayRandomEmote();
                                else if (g == 3)
                                    board->hands_exec_.PlayRandomEmote();
                                else if (g == 4) {
                                    board->head_exec_.PlayRandomEmote();
                                    board->hands_exec_.PlayRandomEmote();
                                } else {
                                    board->head_exec_.PlayRandomEmote();
                                    board->left_hand_exec_.PlayRandomEmote();
                                    board->right_hand_exec_.PlayRandomEmote();
                                }
                                if ((rand() % 100) < 30) {
                                    static const std::string_view kEmoteSounds[] = {
                                        Lang::Sounds::OGG_EMOTION_HAPPY_1, Lang::Sounds::OGG_EMOTION_HAPPY_2,
                                        Lang::Sounds::OGG_EMOTION_HAPPY_3, Lang::Sounds::OGG_EMOTION_NEUTRAL_1,
                                        Lang::Sounds::OGG_EMOTION_NEUTRAL_2, Lang::Sounds::OGG_EMOTION_NEUTRAL_3,
                                    };
                                    Application::GetInstance().PlaySound(kEmoteSounds[rand() % 6]);
                                }
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
    BreatheHeBoard() : WifiBoard(), boot_button_(BOOT_BUTTON_GPIO) {
        esp_log_level_set("Display", ESP_LOG_NONE);
        InitializePowerManagement();
        InitializeCodecI2c();
        InitializeButtons();

        // MPU-6050 (I2C_NUM_1: IO43=SDA, IO44=SCL)
        esp_log_level_set("mpu6050", ESP_LOG_INFO);
        mpu_ok_ = mpu6050_init();
        if (mpu_ok_) {
            ESP_LOGI(TAG, "MPU-6050 OK (IO43=SDA, IO44=SCL)");
        } else {
            ESP_LOGW(TAG, "MPU-6050 not found, motion disabled");
        }

        InitializeBatteryAdc();
        InitializeServo();
        InitializePowerSaveTimer();
        InitializeTools();

        // Start touch + motion detection task
        xTaskCreate([](void* arg) { static_cast<BreatheHeBoard*>(arg)->TouchMotionTask(); },
                    "touch_motion", 4096, this, 2, &touch_task_);

        // Boot sound
        xTaskCreate([](void* arg) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            auto state = Application::GetInstance().GetDeviceState();
            if (state == kDeviceStateStarting || state == kDeviceStateIdle ||
                state == kDeviceStateConnecting) {
                Application::GetInstance().PlaySound(Lang::Sounds::OGG_ANIME_MEW1);
                ESP_LOGI(TAG, "🐼 开机啦~");
            }
            vTaskDelete(nullptr);
        }, "boot_sound", 2048, nullptr, 1, nullptr);
    }

    // ===== Action API =====
    bool RunActionById(uint16_t id, uint16_t speed = 0, uint8_t cycles = 1, bool loop = false) {
        const ServoAction* action = head_exec_.FindActionById(id);
        if (!action) {
            ESP_LOGW(TAG, "Action id=%d not found", id);
            return false;
        }
        if (action->group_mask == GROUP_ALL)
            RunOnAllGroups(action->name, speed, cycles, loop);
        else {
            ActionExecutor* exec = GetExecutorFor(action->name);
            if (!exec) return false;
            exec->Run(action->name, speed, cycles, loop);
        }
        return true;
    }

    bool RunAction(const char* name, uint16_t speed = 0, uint8_t cycles = 1, bool loop = false) {
        const ServoAction* action = head_exec_.FindAction(name);
        if (!action) {
            ESP_LOGW(TAG, "Action '%s' not found", name);
            return false;
        }
        if (action->group_mask == GROUP_ALL)
            RunOnAllGroups(name, speed, cycles, loop);
        else {
            ActionExecutor* exec = GetExecutorFor(name);
            if (!exec) return false;
            exec->Run(name, speed, cycles, loop);
        }
        return true;
    }

    void StopAllActions() {
        head_exec_.Stop();
        left_hand_exec_.Stop();
        right_hand_exec_.Stop();
        hands_exec_.Stop();
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

DECLARE_BOARD(BreatheHeBoard);

int BreatheHeBoard::breathing_amplitude_pct_ = 100;  // default full throw
