#include "sdkconfig.h"

#include <esp_heap_caps.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <esp_log.h>
#include <img_converters.h>

#include "esp32_camera.h"
#include "board.h"
#include "display.h"
#include "lvgl_display.h"
#include "mcp_server.h"
#include "system_info.h"
#include "jpg/image_to_jpeg.h"
#include "esp_timer.h"

#define TAG "Esp32Camera"

Esp32Camera::Esp32Camera(const camera_config_t &config) {
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        if (s->id.PID == GC0308_PID) {
            s->set_hmirror(s, 0); // Control camera mirror: 1 for mirror, 0 for normal
        }
        ESP_LOGI(TAG, "Camera initialized: format=%d", config.pixel_format);
    }

    streaming_on_ = true;
}

Esp32Camera::~Esp32Camera() {
    if (streaming_on_) {
        if (current_fb_) {
            esp_camera_fb_return(current_fb_);
            current_fb_ = nullptr;
        }
        if (encode_buf_) {
            heap_caps_free(encode_buf_);
            encode_buf_ = nullptr;
            encode_buf_size_ = 0;
        }
        esp_camera_deinit();
        streaming_on_ = false;
    }
}

void Esp32Camera::SetExplainUrl(const std::string &url, const std::string &token) {
    explain_url_ = url;
    explain_token_ = token;
}

bool Esp32Camera::Capture() {
    if (encoder_thread_.joinable()) {
        encoder_thread_.join();
    }

    if (!streaming_on_) {
        return false;
    }

    // Get the latest frame, discard old frames for real-time performance
    for (int i = 0; i < 2; i++) {
        if (current_fb_) {
            esp_camera_fb_return(current_fb_);
        }
        current_fb_ = esp_camera_fb_get();
        if (!current_fb_) {
            ESP_LOGE(TAG, "Camera capture failed");
            return false;
        }
    }

    // Prepare encode buffer for RGB565 format (with optional byte swapping)
    if (current_fb_->format == PIXFORMAT_RGB565) {
        size_t pixel_count = current_fb_->width * current_fb_->height;
        size_t data_size = pixel_count * 2;

        // Allocate or reallocate encode buffer if needed
        if (encode_buf_size_ < data_size) {
            if (encode_buf_) {
                heap_caps_free(encode_buf_);
            }
            encode_buf_ = (uint8_t *)heap_caps_malloc(data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (encode_buf_ == nullptr) {
                ESP_LOGE(TAG, "Failed to allocate memory for encode buffer");
                encode_buf_size_ = 0;
                return false;
            }
            encode_buf_size_ = data_size;
        }

        // Copy data to encode buffer with optional byte swapping
        uint16_t *src = (uint16_t *)current_fb_->buf;
        uint16_t *dst = (uint16_t *)encode_buf_;
        if (swap_bytes_enabled_) {
            for (size_t i = 0; i < pixel_count; i++) {
                dst[i] = __builtin_bswap16(src[i]);
            }
        } else {
            memcpy(encode_buf_, current_fb_->buf, data_size);
        }

        // Allocate separate buffer for preview display
        uint8_t *preview_data = (uint8_t *)heap_caps_malloc(data_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (preview_data != nullptr) {
            memcpy(preview_data, encode_buf_, data_size);
            auto display = dynamic_cast<LvglDisplay *>(Board::GetInstance().GetDisplay());
            if (display != nullptr) {
                display->SetPreviewImage(std::make_unique<LvglAllocatedImage>(preview_data, data_size, current_fb_->width, current_fb_->height, current_fb_->width * 2, LV_COLOR_FORMAT_RGB565));
            } else {
                heap_caps_free(preview_data);
            }
        }
    } else if (current_fb_->format == PIXFORMAT_JPEG) {
        // JPEG format preview usually requires decoding, skip preview display for now, just log
        ESP_LOGW(TAG, "JPEG capture success, len=%u, but not supported for preview", (unsigned)current_fb_->len);
    }

    ESP_LOGI(TAG, "Captured frame: %dx%d, len=%u, format=%d",
             current_fb_->width, current_fb_->height, (unsigned)current_fb_->len, current_fb_->format);

    return true;
}

bool Esp32Camera::SetHMirror(bool enabled) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return false;
    }
    s->set_hmirror(s, enabled ? 1 : 0);
    return true;
}

bool Esp32Camera::SetVFlip(bool enabled) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        return false;
    }
    s->set_vflip(s, enabled ? 1 : 0);
    return true;
}

bool Esp32Camera::SetSwapBytes(bool enabled) {
    swap_bytes_enabled_ = enabled;
    return true;
}

std::string Esp32Camera::Explain(const std::string &question) {
    if (explain_url_.empty()) {
        throw std::runtime_error("Image explain URL or token is not set");
    }

    if (current_fb_ == nullptr) {
        throw std::runtime_error("No camera frame captured");
    }

    int64_t start_time = esp_timer_get_time();
    uint16_t w = current_fb_->width;
    uint16_t h = current_fb_->height;
    v4l2_pix_fmt_t enc_fmt;
    switch (current_fb_->format) {
        case PIXFORMAT_RGB565:
            enc_fmt = V4L2_PIX_FMT_RGB565;
            break;
        case PIXFORMAT_YUV422:
            enc_fmt = V4L2_PIX_FMT_YUYV;  // YUV422 is actually YUYV format
            break;
        case PIXFORMAT_YUV420:
            enc_fmt = V4L2_PIX_FMT_YUV420;
            break;
        case PIXFORMAT_GRAYSCALE:
            enc_fmt = V4L2_PIX_FMT_GREY;
            break;
        case PIXFORMAT_JPEG:
            enc_fmt = V4L2_PIX_FMT_JPEG;
            break;
        case PIXFORMAT_RGB888:
            enc_fmt = V4L2_PIX_FMT_RGB24;
            break;
        default:
            ESP_LOGE(TAG, "Unsupported pixel format: %d", current_fb_->format);
            throw std::runtime_error("Unsupported pixel format");
    }

    uint8_t *jpeg_src_buf = current_fb_->buf;
    size_t jpeg_src_len = current_fb_->len;
    if (current_fb_->format == PIXFORMAT_RGB565 && encode_buf_ != nullptr) {
        jpeg_src_buf = encode_buf_;
        jpeg_src_len = encode_buf_size_;
    }

    std::string jpeg_data;
    bool ok = image_to_jpeg_cb(jpeg_src_buf, jpeg_src_len, w, h, enc_fmt, 80,
        [](void* arg, size_t index, const void* data, size_t len) -> size_t {
            auto jpeg_data = static_cast<std::string*>(arg);
            if (data != nullptr && len > 0) {
                jpeg_data->append(static_cast<const char*>(data), len);
            }
            return len;
        }, &jpeg_data);

    int64_t end_time = esp_timer_get_time();
    ESP_LOGI(TAG, "JPEG encoding time: %ld ms, size=%u", int((end_time - start_time) / 1000), (unsigned)jpeg_data.size());
    if (!ok || jpeg_data.empty()) {
        ESP_LOGE(TAG, "JPEG encoder failed or produced empty output");
        throw std::runtime_error("Failed to encode image to JPEG");
    }

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(3);
    std::string boundary = "----ESP32_CAMERA_BOUNDARY";
    std::string question_field;
    question_field += "--" + boundary + "\r\n";
    question_field += "Content-Disposition: form-data; name=\"question\"\r\n";
    question_field += "\r\n";
    question_field += question + "\r\n";

    std::string file_header;
    file_header += "--" + boundary + "\r\n";
    file_header += "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n";
    file_header += "Content-Type: image/jpeg\r\n";
    file_header += "\r\n";

    std::string multipart_footer;
    multipart_footer += "\r\n--" + boundary + "--\r\n";

    size_t content_length = question_field.size() + file_header.size() + jpeg_data.size() + multipart_footer.size();
    size_t total_sent = jpeg_data.size();

    http->SetTimeout(8000);
    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_token_.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token_);
    }
    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Content-Length", std::to_string(content_length));
    http->SetKeepAlive(false);
    // Mark the request as having a body so HttpClient does not switch to chunked mode.
    http->SetContent(std::string());
    ESP_LOGI(TAG, "Opening explain URL, jpeg=%u bytes, body=%u bytes", (unsigned)jpeg_data.size(), (unsigned)content_length);
    if (!http->Open("POST", explain_url_)) {
        ESP_LOGE(TAG, "Failed to connect to explain URL");
        throw std::runtime_error("Failed to connect to explain URL");
    }
    ESP_LOGI(TAG, "Explain URL opened, uploading body");

    auto write_all = [&http](const char* data, size_t size) {
        const size_t max_chunk_size = 1400;
        size_t written = 0;
        while (written < size) {
            size_t chunk_size = std::min(max_chunk_size, size - written);
            int ret = http->Write(data + written, chunk_size);
            if (ret <= 0) {
                return false;
            }
            written += chunk_size;
        }
        return true;
    };

    if (!write_all(question_field.data(), question_field.size()) ||
        !write_all(file_header.data(), file_header.size()) ||
        !write_all(jpeg_data.data(), jpeg_data.size()) ||
        !write_all(multipart_footer.data(), multipart_footer.size())) {
        http->Close();
        ESP_LOGE(TAG, "Failed to upload photo body");
        throw std::runtime_error("Failed to upload photo body");
    }
    ESP_LOGI(TAG, "Photo body uploaded, waiting for response");

    int status_code = http->GetStatusCode();
    ESP_LOGI(TAG, "Uploaded photo payload: %u bytes, status=%d", (unsigned)total_sent, status_code);
    if (status_code != 200) {
        std::string error_body = http->ReadAll();
        http->Close();
        ESP_LOGE(TAG, "Failed to upload photo, status code: %d, body: %s", status_code, error_body.c_str());
        throw std::runtime_error("Failed to upload photo");
    }

    std::string result = http->ReadAll();
    http->Close();

    size_t remain_stack_size = uxTaskGetStackHighWaterMark(nullptr);
    ESP_LOGI(TAG, "Explain image size=%dx%d, compressed size=%d, remain stack size=%d, question=%s\n%s",
             current_fb_->width, current_fb_->height, (int)total_sent, (int)remain_stack_size, question.c_str(), result.c_str());
    return result;
}
