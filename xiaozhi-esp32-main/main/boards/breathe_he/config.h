#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

// breathe_he board configuration (audio + 3x 270° servos)
// Panda: head + left hand + right hand (breathing & crawling)

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

// Servo PWM: 3x 270° servos, power controlled by IO4
// GPIO 15: Head (左右转)
// GPIO 16: Left hand (爬行/呼吸)
// GPIO 17: Right hand (爬行/呼吸)
#define SERVO_POWER_GPIO GPIO_NUM_4
#define SERVO_0_GPIO GPIO_NUM_15  // Head
#define SERVO_1_GPIO GPIO_NUM_16  // Left hand
#define SERVO_2_GPIO GPIO_NUM_17  // Right hand

#endif  // _BOARD_CONFIG_H_
