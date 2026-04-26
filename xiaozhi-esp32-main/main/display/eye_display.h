#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include "esp_lcd_panel_ops.h"

extern esp_lcd_panel_io_handle_t lcd_io_eye;
extern esp_lcd_panel_handle_t lcd_panel_eye;
extern esp_lcd_panel_io_handle_t lcd_io_eye2;
extern esp_lcd_panel_handle_t lcd_panel_eye2;

extern TaskHandle_t task_update_eye_handler;  // 魔眼更新任务的句柄

// esp_err_t esp_lcd_safe_draw_bitmap(esp_lcd_panel_handle_t panel, int x_start, int y_start,
//                                    int x_end, int y_end, const void* color_data);
