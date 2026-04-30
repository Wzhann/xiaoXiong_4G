/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dual_eye_render.h"

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "eyes_data.h"
// #include "my_eyes_data.h"

namespace {

// Eye material selection.
// 1. If you generated a custom header, uncomment:
//    #include "my_eyes_data.h"
// 2. Then switch the two macros below.
// 3. Rebuild and flash.
#define EYE_SCLERA_MATERIAL sclera_n
#define EYE_IRIS_MATERIAL iris_blue1

// Eye layout tuning.
// These are the main values you may want to adjust later.
#define EYE_RENDER_WIDTH 120
#define EYE_RENDER_HEIGHT 120
#define LEFT_EYE_RENDER_OFFSET_X 0
#define RIGHT_EYE_RENDER_OFFSET_X (160 - EYE_RENDER_WIDTH)
#define EYE_RENDER_OFFSET_Y ((160 - EYE_RENDER_HEIGHT) / 2)
#define EYE_LEFT_SHIFT (-10)
#define EYE_RIGHT_SHIFT (10)
#define EYE_PUPIL_MOVE_X_MIN (-60)
#define EYE_PUPIL_MOVE_X_MAX (60)
#define EYE_PUPIL_MOVE_Y_MIN (-44)
#define EYE_PUPIL_MOVE_Y_MAX (44)

constexpr const char* kTag = "dual_eye_render";
constexpr int kLinesPerBatch = 10;
constexpr int kFrameDelayMs = 10;
constexpr int kNoBlink = 0;
constexpr int kEnBlink = 1;
constexpr int kDeBlink = 2;
constexpr int kNumEyes = 1;

// xiaoXiong_4G uses 160x160 displays.
// We render the eye only inside a centered viewport instead of filling the whole screen.
constexpr uint16_t kDisplayWidth = 160;
constexpr uint16_t kDisplayHeight = 160;
constexpr uint16_t kRenderWidth = EYE_RENDER_WIDTH;
constexpr uint16_t kRenderHeight = EYE_RENDER_HEIGHT;
constexpr uint16_t kLeftRenderOffsetX = LEFT_EYE_RENDER_OFFSET_X;
constexpr uint16_t kRightRenderOffsetX = RIGHT_EYE_RENDER_OFFSET_X;
constexpr uint16_t kRenderOffsetY = EYE_RENDER_OFFSET_Y;
constexpr uint16_t kSourceDisplayWidth = 240;
constexpr uint16_t kSourceDisplayHeight = 240;
constexpr uint16_t kDisplaySize = kSourceDisplayWidth;
constexpr int kLeftEyeShift = EYE_LEFT_SHIFT;
constexpr int kRightEyeShift = EYE_RIGHT_SHIFT;
constexpr uint16_t kMaskWidth = 240;
constexpr uint16_t kMaskHeight = 240;
constexpr uint16_t kScleraWidth = 375;
constexpr uint16_t kScleraHeight = 375;
constexpr uint16_t kIrisWidth = 150;
constexpr uint16_t kIrisHeight = 150;
constexpr uint16_t kIrisMapHeight = 64;
constexpr uint16_t kIrisMapWidth = 256;
constexpr size_t kDmaBufferAlign = 64;
constexpr size_t kLineBufferPixels = kLinesPerBatch * kRenderWidth;
constexpr size_t kBlackLinePixels = kDisplayWidth;
constexpr uint16_t kLeftEyeRightBorderWidth = kDisplayWidth - (kLeftRenderOffsetX + kRenderWidth);
constexpr uint16_t kRightEyeRightBorderWidth = kDisplayWidth - (kRightRenderOffsetX + kRenderWidth);
constexpr uint16_t kLeftEyeMaxBorderWidth =
    kLeftRenderOffsetX > kLeftEyeRightBorderWidth ? kLeftRenderOffsetX : kLeftEyeRightBorderWidth;
constexpr uint16_t kRightEyeMaxBorderWidth = kRightRenderOffsetX > kRightEyeRightBorderWidth
                                                 ? kRightRenderOffsetX
                                                 : kRightEyeRightBorderWidth;
constexpr uint16_t kMaxBorderWidth = kLeftEyeMaxBorderWidth > kRightEyeMaxBorderWidth
                                         ? kLeftEyeMaxBorderWidth
                                         : kRightEyeMaxBorderWidth;
constexpr size_t kMaxBorderPixels = kMaxBorderWidth * kRenderHeight;

const uint16_t* sclera = EYE_SCLERA_MATERIAL;
const uint8_t* upper = upper_default;
const uint8_t* lower = lower_default;
const uint16_t* polar = polar_default;
const uint16_t* iris = EYE_IRIS_MATERIAL;

const uint8_t ease[] = {
    0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   2,   2,   2,   3,   3,   3,   4,
    4,   4,   5,   5,   6,   6,   7,   7,   8,   9,   9,   10,  10,  11,  12,  12,  13,  14,  15,
    15,  16,  17,  18,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  27,  28,  29,  30,  31,
    33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  44,  45,  46,  47,  48,  50,  51,  52,  53,
    54,  56,  57,  58,  60,  61,  62,  63,  65,  66,  67,  69,  70,  72,  73,  74,  76,  77,  78,
    80,  81,  83,  84,  85,  87,  88,  90,  91,  93,  94,  96,  97,  98,  100, 101, 103, 104, 106,
    107, 109, 110, 112, 113, 115, 116, 118, 119, 121, 122, 124, 125, 127, 128, 130, 131, 133, 134,
    136, 137, 139, 140, 142, 143, 145, 146, 148, 149, 151, 152, 154, 155, 157, 158, 159, 161, 162,
    164, 165, 167, 168, 170, 171, 172, 174, 175, 177, 178, 179, 181, 182, 183, 185, 186, 188, 189,
    190, 192, 193, 194, 195, 197, 198, 199, 201, 202, 203, 204, 205, 207, 208, 209, 210, 211, 213,
    214, 215, 216, 217, 218, 219, 220, 221, 222, 224, 225, 226, 227, 228, 228, 229, 230, 231, 232,
    233, 234, 235, 236, 237, 237, 238, 239, 240, 240, 241, 242, 243, 243, 244, 245, 245, 246, 246,
    247, 248, 248, 249, 249, 250, 250, 251, 251, 251, 252, 252, 252, 253, 253, 253, 254, 254, 254,
    254, 254, 255, 255, 255, 255, 255, 255, 255,
};

typedef struct {
    uint8_t state;
    int32_t duration;
    uint32_t startTime;
} EyeBlink;

struct EyeState {
    EyeBlink blink;
};

EyeState eye[kNumEyes];
uint32_t time_of_last_blink = 0;
uint32_t time_to_next_blink = 0;
uint16_t old_iris = (IRIS_MIN + IRIS_MAX) / 2;
uint16_t new_iris = (IRIS_MIN + IRIS_MAX) / 2;
uint16_t* dma_black_line = nullptr;
uint16_t* dma_border_block = nullptr;
uint16_t* dma_line_buf[2][2] = {{nullptr, nullptr}, {nullptr, nullptr}};

uint16_t* alloc_dma_pixels(size_t pixels, bool clear = false) {
    uint16_t* buffer = static_cast<uint16_t*>(heap_caps_aligned_alloc(
        kDmaBufferAlign, pixels * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (buffer == nullptr) {
        ESP_LOGE(kTag, "Failed to allocate %u DMA pixels", static_cast<unsigned>(pixels));
        return nullptr;
    }
    if (clear) {
        memset(buffer, 0, pixels * sizeof(uint16_t));
    }
    return buffer;
}

uint16_t* get_dma_black_line() {
    if (dma_black_line == nullptr) {
        dma_black_line = alloc_dma_pixels(kBlackLinePixels, true);
    }
    return dma_black_line;
}

uint16_t* get_dma_border_block() {
    if (dma_border_block == nullptr) {
        dma_border_block = alloc_dma_pixels(kMaxBorderPixels, true);
    }
    return dma_border_block;
}

uint16_t* get_dma_line_buffer(size_t panel_index, size_t buffer_index) {
    if (panel_index >= 2 || buffer_index >= 2) {
        return nullptr;
    }
    if (dma_line_buf[panel_index][buffer_index] == nullptr) {
        dma_line_buf[panel_index][buffer_index] = alloc_dma_pixels(kLineBufferPixels);
    }
    return dma_line_buf[panel_index][buffer_index];
}

bool ensure_dma_buffers() {
    return get_dma_black_line() != nullptr && get_dma_border_block() != nullptr &&
           get_dma_line_buffer(0, 0) != nullptr && get_dma_line_buffer(0, 1) != nullptr &&
           get_dma_line_buffer(1, 0) != nullptr && get_dma_line_buffer(1, 1) != nullptr;
}

void clear_panel(esp_lcd_panel_handle_t panel) {
    if (panel == nullptr) {
        return;
    }

    uint16_t* black_line = get_dma_black_line();
    if (black_line == nullptr) {
        return;
    }
    for (int y = 0; y < kDisplayHeight; ++y) {
        esp_lcd_panel_draw_bitmap(panel, 0, y, kDisplayWidth, y + 1, black_line);
    }
}

void clear_eye_border(esp_lcd_panel_handle_t panel, uint16_t render_offset_x) {
    if (panel == nullptr) {
        return;
    }

    uint16_t* black_line = get_dma_black_line();
    if (black_line == nullptr) {
        return;
    }

    for (int y = 0; y < kRenderOffsetY; ++y) {
        esp_lcd_panel_draw_bitmap(panel, 0, y, kDisplayWidth, y + 1, black_line);
    }

    for (int y = kRenderOffsetY + kRenderHeight; y < kDisplayHeight; ++y) {
        esp_lcd_panel_draw_bitmap(panel, 0, y, kDisplayWidth, y + 1, black_line);
    }

    const uint16_t left_border_width = render_offset_x;
    const uint16_t right_border_x = render_offset_x + kRenderWidth;
    const uint16_t right_border_width =
        right_border_x < kDisplayWidth ? (kDisplayWidth - right_border_x) : 0;
    const uint16_t max_border_width =
        left_border_width > right_border_width ? left_border_width : right_border_width;

    if (max_border_width > 0) {
        uint16_t* black_block_side = get_dma_border_block();
        if (black_block_side == nullptr) {
            return;
        }
        if (left_border_width > 0) {
            esp_lcd_panel_draw_bitmap(panel, 0, kRenderOffsetY, left_border_width,
                                      kRenderOffsetY + kRenderHeight, black_block_side);
        }
        if (right_border_width > 0) {
            esp_lcd_panel_draw_bitmap(panel, right_border_x, kRenderOffsetY, kDisplayWidth,
                                      kRenderOffsetY + kRenderHeight, black_block_side);
        }
    }
}

void draw_eye_single(esp_lcd_panel_handle_t panel, size_t panel_index, uint16_t render_offset_x,
                     uint32_t iScale, int32_t scleraX, uint32_t scleraY, uint32_t uT,
                     uint32_t lT) {
    uint32_t scleraXsave = scleraX;
    int16_t irisYBase = scleraY - (kScleraHeight - kIrisHeight) / 2;

    uint16_t* line_buf[2];
    line_buf[0] = get_dma_line_buffer(panel_index, 0);
    line_buf[1] = get_dma_line_buffer(panel_index, 1);
    if (line_buf[0] == nullptr || line_buf[1] == nullptr) {
        return;
    }

    uint8_t buf_idx = 0;
    for (uint16_t screenY = 0; screenY < kRenderHeight; screenY += kLinesPerBatch) {
        uint16_t* current_buf = line_buf[buf_idx];
        buf_idx ^= 1;
        uint8_t lines_to_process =
            (kRenderHeight - screenY) < kLinesPerBatch ? (kRenderHeight - screenY) : kLinesPerBatch;

        for (uint8_t line = 0; line < lines_to_process; ++line) {
            uint16_t outY = screenY + line;
            uint16_t sourceY = (outY * kSourceDisplayHeight) / kRenderHeight;
            uint32_t scleraYLine = scleraY + sourceY;
            int16_t irisY = irisYBase + sourceY;
            int16_t irisXBase = scleraXsave - (kScleraWidth - kIrisWidth) / 2;

            for (uint16_t screenX = 0; screenX < kRenderWidth; ++screenX) {
                uint16_t sourceX = (screenX * kSourceDisplayWidth) / kRenderWidth;
                uint32_t scleraSampleX = scleraXsave + sourceX;
                int16_t irisX = irisXBase + sourceX;
                uint32_t mask_x = sourceX;
                uint32_t mask_y = sourceY;
                uint32_t screen_idx = mask_y * kMaskWidth + mask_x;
                uint32_t pixel_idx = line * kRenderWidth + screenX;
                uint16_t p;

                if ((lower[screen_idx] <= lT) || (upper[screen_idx] <= uT)) {
                    p = 0;
                } else if ((irisY < 0) || (irisY >= kIrisHeight) || (irisX < 0) ||
                           (irisX >= kIrisWidth)) {
                    p = sclera[scleraYLine * kScleraWidth + scleraSampleX];
                } else {
                    p = polar[irisY * kIrisWidth + irisX];
                    uint32_t d = (iScale * (p & 0x7F)) / 240;
                    if (d < kIrisMapHeight) {
                        uint16_t a = (kIrisMapWidth * (p >> 7)) / 512;
                        p = iris[d * kIrisMapWidth + a];
                    } else {
                        p = sclera[scleraYLine * kScleraWidth + scleraSampleX];
                    }
                }
                current_buf[pixel_idx] = (p >> 8) | (p << 8);
            }
        }

        dual_eye_draw_bitmap_single(panel, render_offset_x, kRenderOffsetY + screenY,
                                    render_offset_x + kRenderWidth,
                                    kRenderOffsetY + screenY + lines_to_process, current_buf);
    }

    clear_eye_border(panel, render_offset_x);
}

void frame_impl(esp_lcd_panel_handle_t panel1, esp_lcd_panel_handle_t panel2, uint16_t iScale) {
    static uint8_t eye_index = 0;
    static bool eye_in_motion = false;
    static int16_t eye_old_x = 512;
    static int16_t eye_old_y = 512;
    static uint32_t eye_move_start_time = 0;
    static int32_t eye_move_duration = 0;
    static uint8_t u_threshold = 240;

    int16_t eyeX;
    int16_t eyeY;
    uint32_t t = esp_timer_get_time();

    if (++eye_index >= kNumEyes) {
        eye_index = 0;
    }

    int32_t dt = t - eye_move_start_time;
    if (eye_in_motion) {
        if (dt >= eye_move_duration) {
            eye_in_motion = false;
            eye_move_duration = dual_eye_random_max(100000);
            eye_move_start_time = t;
            eyeX = eye_old_x = dual_eye_new_x;
            eyeY = eye_old_y = dual_eye_new_y;
        } else {
            int16_t e = ease[255 * dt / eye_move_duration] + 1;
            eyeX = eye_old_x + (((dual_eye_new_x - eye_old_x) * e) / 256);
            eyeY = eye_old_y + (((dual_eye_new_y - eye_old_y) * e) / 256);
        }
    } else {
        eyeX = eye_old_x;
        eyeY = eye_old_y;
        if (dt > eye_move_duration) {
            int16_t dx;
            int16_t dy;
            uint32_t d;
            do {
                dx = (dual_eye_new_x * 2) - 1023;
                dy = (dual_eye_new_y * 2) - 1023;
            } while ((d = (dx * dx + dy * dy)) > (1023 * 1023));
            eye_move_duration = dual_eye_random(72000, 144000);
            eye_move_start_time = t;
            eye_in_motion = true;
        }
    }

    if (dual_eye_is_blink && (t - time_of_last_blink) >= time_to_next_blink) {
        time_of_last_blink = t;
        uint32_t blink_duration = dual_eye_random(36000, 72000);
        for (uint8_t e = 0; e < kNumEyes; ++e) {
            if (eye[e].blink.state == kNoBlink) {
                eye[e].blink.state = kEnBlink;
                eye[e].blink.startTime = t;
                eye[e].blink.duration = blink_duration;
            }
        }
        time_to_next_blink = blink_duration * 3 + dual_eye_random_max(4000000);
    }

    if (eye[eye_index].blink.state &&
        (t - eye[eye_index].blink.startTime) >= eye[eye_index].blink.duration) {
        if (++eye[eye_index].blink.state > kDeBlink) {
            eye[eye_index].blink.state = kNoBlink;
        } else {
            eye[eye_index].blink.duration *= 2;
            eye[eye_index].blink.startTime = t;
        }
    }

    int16_t irisOffsetX = dual_eye_map(eyeX, 0, 1023, EYE_PUPIL_MOVE_X_MIN, EYE_PUPIL_MOVE_X_MAX);
    int16_t irisOffsetY = dual_eye_map(eyeY, 0, 1023, EYE_PUPIL_MOVE_Y_MIN, EYE_PUPIL_MOVE_Y_MAX);

    const int16_t scleraBaseX = (kScleraWidth - kDisplaySize) / 2;
    const int16_t scleraBaseY = (kScleraHeight - kDisplaySize) / 2;

    eyeX = scleraBaseX;
    eyeY = scleraBaseY;

    uint8_t l_threshold = 0;
    uint8_t n = 0;
    if (dual_eye_is_track) {
        int16_t sampleX = kScleraWidth / 2 - (irisOffsetX / 2);
        int16_t sampleY = kScleraHeight / 2 - ((kDisplaySize / 2) + irisOffsetY + kIrisHeight / 4);
        if (sampleY >= 0) {
            n = upper[sampleY * kMaskWidth + sampleX] +
                upper[sampleY * kMaskWidth + (kMaskWidth - 1 - sampleX)] / 2;
        }
        u_threshold = (u_threshold * 3 + n) / 4;
        l_threshold = 254 - u_threshold;
    } else {
        u_threshold = 0;
        l_threshold = 0;
    }

    if (eye[eye_index].blink.state) {
        uint32_t s = (t - eye[eye_index].blink.startTime);
        s = (s >= eye[eye_index].blink.duration) ? 255 : 255 * s / eye[eye_index].blink.duration;
        s = (eye[eye_index].blink.state == kDeBlink) ? 1 + s : 256 - s;
        n = (u_threshold * s + 254 * (257 - s)) / 256;
        l_threshold = (l_threshold * s + 254 * (257 - s)) / 256;
    } else {
        n = u_threshold;
    }

    // Keep the draw window centered on each physical panel and move the sampled eye content
    // inward independently. This avoids panel-window artifacts while preserving convergence.
    int32_t left_eye_x = eyeX + kLeftEyeShift + irisOffsetX;
    int32_t right_eye_x = eyeX + kRightEyeShift + irisOffsetX;
    int32_t left_eye_y = eyeY + irisOffsetY;
    int32_t right_eye_y = eyeY + irisOffsetY;

    auto clamp_x = [](int32_t value) -> int32_t {
        if (value < 0) {
            return 0;
        }
        if (value > (kScleraWidth - kDisplaySize)) {
            return kScleraWidth - kDisplaySize;
        }
        return value;
    };
    auto clamp_y = [](int32_t value) -> int32_t {
        if (value < 0) {
            return 0;
        }
        if (value > (kScleraHeight - kDisplaySize)) {
            return kScleraHeight - kDisplaySize;
        }
        return value;
    };

    draw_eye_single(panel1, 0, kLeftRenderOffsetX, iScale, clamp_x(left_eye_x),
                    clamp_y(left_eye_y), n, l_threshold);
    draw_eye_single(panel2, 1, kRightRenderOffsetX, iScale, clamp_x(right_eye_x),
                    clamp_y(right_eye_y), n, l_threshold);
}

void split_impl(esp_lcd_panel_handle_t panel1, esp_lcd_panel_handle_t panel2, int16_t startValue,
                int16_t endValue, uint64_t startTime, int32_t duration, int16_t range) {
    if (range >= 8) {
        range /= 2;
        duration /= 2;
        int16_t midValue = (startValue + endValue - range) / 2 + (esp_random() % range);
        uint64_t midTime = startTime + duration;
        split_impl(panel1, panel2, startValue, midValue, startTime, duration, range);
        split_impl(panel1, panel2, midValue, endValue, midTime, duration, range);
        return;
    }

    int32_t dt;
    while ((dt = (esp_timer_get_time() - startTime)) < duration) {
        int16_t v = startValue + (((endValue - startValue) * dt) / duration);
        if (v < IRIS_MIN) {
            v = IRIS_MIN;
        } else if (v > IRIS_MAX) {
            v = IRIS_MAX;
        }
        frame_impl(panel1, panel2, v);
        vTaskDelay(pdMS_TO_TICKS(kFrameDelayMs));
    }
}

}  // namespace

bool dual_eye_is_blink = true;
bool dual_eye_is_track = true;
int16_t dual_eye_new_x = 512;
int16_t dual_eye_new_y = 512;
SemaphoreHandle_t dual_eye_lcd_mutex = nullptr;

int dual_eye_map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int dual_eye_random(int min, int max) { return min + esp_random() % (max - min + 1); }

int dual_eye_random_max(int max) { return esp_random() % max; }

esp_err_t dual_eye_draw_bitmap(esp_lcd_panel_handle_t panel1, esp_lcd_panel_handle_t panel2,
                               int x_start, int y_start, int x_end, int y_end,
                               const void* color_data) {
    if (dual_eye_lcd_mutex != nullptr &&
        xSemaphoreTake(dual_eye_lcd_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(kTag, "Failed to acquire LCD mutex");
        return ESP_FAIL;
    }

    esp_err_t ret1 = panel1 == nullptr ? ESP_OK
                                       : esp_lcd_panel_draw_bitmap(panel1, x_start, y_start, x_end,
                                                                   y_end, color_data);
    esp_err_t ret2 = panel2 == nullptr ? ESP_OK
                                       : esp_lcd_panel_draw_bitmap(panel2, x_start, y_start, x_end,
                                                                   y_end, color_data);

    if (dual_eye_lcd_mutex != nullptr) {
        xSemaphoreGive(dual_eye_lcd_mutex);
    }

    return (ret1 == ESP_OK && ret2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t dual_eye_draw_bitmap_single(esp_lcd_panel_handle_t panel, int x_start, int y_start,
                                      int x_end, int y_end, const void* color_data) {
    if (dual_eye_lcd_mutex != nullptr &&
        xSemaphoreTake(dual_eye_lcd_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(kTag, "Failed to acquire LCD mutex");
        return ESP_FAIL;
    }

    esp_err_t ret = panel == nullptr ? ESP_OK
                                     : esp_lcd_panel_draw_bitmap(panel, x_start, y_start, x_end,
                                                                 y_end, color_data);

    if (dual_eye_lcd_mutex != nullptr) {
        xSemaphoreGive(dual_eye_lcd_mutex);
    }

    return ret;
}

void task_dual_eye_update(void* pvParameters) {
    auto* panels = static_cast<esp_lcd_panel_handle_t*>(pvParameters);
    esp_lcd_panel_handle_t panel1 = panels[0];
    esp_lcd_panel_handle_t panel2 = panels[1];

    if (dual_eye_lcd_mutex == nullptr) {
        dual_eye_lcd_mutex = xSemaphoreCreateMutex();
    }

    for (uint8_t e = 0; e < kNumEyes; ++e) {
        eye[e].blink.state = kNoBlink;
    }

    if (!ensure_dma_buffers()) {
        ESP_LOGE(kTag, "LCD DMA buffers unavailable, stopping eye task");
        vTaskDelete(nullptr);
        return;
    }

    clear_panel(panel1);
    clear_panel(panel2);

    while (1) {
        new_iris = dual_eye_random(IRIS_MIN, IRIS_MAX);
        dual_eye_new_x = dual_eye_random_max(1024);
        dual_eye_new_y = dual_eye_random_max(1024);
        split_impl(panel1, panel2, old_iris, new_iris, esp_timer_get_time(), 5000000L,
                   IRIS_MAX - IRIS_MIN);
        old_iris = new_iris;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
