#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

// cat_tailAndNeck board configuration (audio + 5 servos)

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE 24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000
#define AUDIO_INPUT_REFERENCE true
#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_NC
#define AUDIO_I2S_GPIO_WS GPIO_NUM_13
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_48
#define AUDIO_I2S_GPIO_DIN GPIO_NUM_14
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_46

#define AUDIO_CODEC_GPIO_PA GPIO_NUM_NC
#define AUDIO_CODEC_I2C_SDA_PIN GPIO_NUM_2
#define AUDIO_CODEC_I2C_SCL_PIN GPIO_NUM_38
#define AUDIO_CODEC_ES8311_ADDR ES8311_CODEC_DEFAULT_ADDR

#define BUILTIN_LED_GPIO GPIO_NUM_NC
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// Power management: POWER_CTRL latches power on, POWER_OUT reads button state
// Long press POWER_OUT (low for 2s) → POWER_CTRL low → power off
#define POWER_CTRL_GPIO GPIO_NUM_7
#define POWER_OUT_GPIO GPIO_NUM_6  // ADC读取电压(1V分界)，避免数字GPIO被MSPI影响
#define POWER_LONG_PRESS_MS 2000
#define POWER_OUT_ACTIVE_LOW true

// Battery ADC: IO3 = ADC1_CH2
// Voltage divider: R_upper=2k, R_lower=4.7k
// Vpin = Vbat * 4.7 / (2 + 4.7) -> Vbat = Vpin * 1.426
// Battery voltage range: 3.2V (0%) ~ 4.2V (100%)
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_2
#define BATTERY_DIVIDER_RATIO 1.426f
#define BATTERY_EMPTY_VOLTAGE_MV 3200
#define BATTERY_FULL_VOLTAGE_MV 4200

// Servo PWM: 5 servos in 3 groups, power controlled by IO4
// Group Neck: IO18, IO17
// Group Tail: IO15, IO16
// Group Head: IO8
#define SERVO_POWER_GPIO GPIO_NUM_4
#define SERVO_0_GPIO GPIO_NUM_18  // Neck0
#define SERVO_1_GPIO GPIO_NUM_17  // Neck1
#define SERVO_2_GPIO GPIO_NUM_15  // Tail0
#define SERVO_3_GPIO GPIO_NUM_16  // Tail1
#define SERVO_4_GPIO GPIO_NUM_8   // Head

#endif  // _BOARD_CONFIG_H_
