/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <esp_log.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_random.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "eyes_data.h"

#include "eye_display.h"

#include <stdbool.h>

static const char* TAG = "eye_display";
// 两个面板眼睛的句柄
esp_lcd_panel_io_handle_t lcd_io_eye = NULL;
esp_lcd_panel_handle_t lcd_panel_eye = NULL;
esp_lcd_panel_io_handle_t lcd_io_eye2 = NULL;
esp_lcd_panel_handle_t lcd_panel_eye2 = NULL;

TaskHandle_t task_update_eye_handler = NULL;  // 魔眼更新任务的句柄

// 封装的安全绘图函数
/*
    参数：lcd的句柄、
    x_start、y_start：位图的起始坐标。
    x_end、y_end：位图的结束坐标。
    color_data：指向位图颜色数据的指针。
// */
// esp_err_t esp_lcd_safe_draw_bitmap(int x_start, int y_start, int x_end, int y_end,
//                                    const void* color_data) {
//     if (xSemaphoreTake(lcd_mutex, portMAX_DELAY) != pdTRUE) {
//         ESP_LOGE("LCD", "Failed to acquire LCD mutex");
//         return ESP_FAIL;
//     }

//     esp_err_t ret =
//         esp_lcd_panel_draw_bitmap(lcd_panel_eye, x_start, y_start, x_end, y_end, color_data);
//     esp_err_t ret2 =
//         esp_lcd_panel_draw_bitmap(lcd_panel_eye2, x_start, y_start, x_end, y_end, color_data);

//     xSemaphoreGive(lcd_mutex);

//     return (ret == ESP_OK) ? ESP_OK : ESP_FAIL;
// }
