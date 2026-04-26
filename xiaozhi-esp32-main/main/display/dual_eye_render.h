#ifndef DUAL_EYE_RENDER_H
#define DUAL_EYE_RENDER_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

#if !defined(IRIS_MAX)
#define IRIS_MAX 280
#endif

#if !defined(IRIS_MIN)
#define IRIS_MIN 180
#endif

extern bool dual_eye_is_blink;
extern bool dual_eye_is_track;
extern int16_t dual_eye_new_x;
extern int16_t dual_eye_new_y;

extern SemaphoreHandle_t dual_eye_lcd_mutex;

int dual_eye_map(int x, int in_min, int in_max, int out_min, int out_max);
int dual_eye_random(int min, int max);
int dual_eye_random_max(int max);
esp_err_t dual_eye_draw_bitmap(esp_lcd_panel_handle_t panel1, esp_lcd_panel_handle_t panel2,
                               int x_start, int y_start, int x_end, int y_end,
                               const void* color_data);
esp_err_t dual_eye_draw_bitmap_single(esp_lcd_panel_handle_t panel, int x_start, int y_start,
                                      int x_end, int y_end, const void* color_data);
void task_dual_eye_update(void* pvParameters);

#endif
