#include "application.h"
#include "assets/lang_config.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "display/lcd_display.h"
#include "esp_lcd_gc9d01n.h"
#include "eye_display.h"
#include "i2c_device.h"
#include "wifi_board.h"

#include <driver/i2c_master.h>
#include <esp_lcd_gc9a01.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_log.h>

#define TAG "AtomS3+EchoBase"

static constexpr int kImageWidth = 54;
static constexpr int kImageHeight = 54;
static uint16_t s_black_line[DISPLAY_WIDTH] = {0};

static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb2, (uint8_t[]){0x2f}, 1, 0},
    {0xb3, (uint8_t[]){0x03}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x01}, 1, 0},
    {0xac, (uint8_t[]){0xcb}, 1, 0},
    {0xab, (uint8_t[]){0x0e}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x19}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xe8, (uint8_t[]){0x24}, 1, 0},
    {0xe9, (uint8_t[]){0x48}, 1, 0},
    {0xea, (uint8_t[]){0x22}, 1, 0},
    {0xc6, (uint8_t[]){0x30}, 1, 0},
    {0xc7, (uint8_t[]){0x18}, 1, 0},
    {0xf0,
     (uint8_t[]){0x1f, 0x28, 0x04, 0x3e, 0x2a, 0x2e, 0x20, 0x00, 0x0c, 0x06, 0x00, 0x1c, 0x1f,
                 0x0f},
     14, 0},
    {0xf1,
     (uint8_t[]){0x00, 0x2d, 0x2f, 0x3c, 0x6f, 0x1c, 0x0b, 0x00, 0x00, 0x00, 0x07, 0x0d, 0x11,
                 0x0f},
     14, 0},
};

void DrawBootImage(void* pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "EYE_Task...");

        const int x = (DISPLAY_WIDTH - kImageWidth) / 2;
        const int y = (DISPLAY_HEIGHT - kImageHeight) / 2;

        for (int row = 0; row < DISPLAY_HEIGHT; ++row) {
            esp_lcd_panel_draw_bitmap(lcd_panel_eye, 0, row, DISPLAY_WIDTH, row + 1, s_black_line);
        }

        ESP_LOGI(TAG, "Draw static image without LVGL");
        esp_lcd_panel_draw_bitmap(lcd_panel_eye, x, y, x + kImageWidth, y + kImageHeight,
                                  (void*)gImage_new_eye_1_1);

        // esp_lcd_panel_draw_bitmap(lcd_panel_eye, 20, 20, 20 + 54, 20 + 54,
        //                           (uint16_t*)gImage_new_eye_1_1);

        // draw_image_rgb565(&screen1, 30, 0, 54, 54, (uint16_t*)gImage_new_eye_1_1);
        // draw_image_rgb565(&screen2, 12, 0, 54, 54, (uint16_t*)gImage_new_eye_1_1);
        // vTaskDelay(2000 / portTICK_PERIOD_MS);  //

        // draw_image_rgb565(&screen1, 30, 0, 54, 54, (uint16_t*)gImage_new_eye_1_2);
        // draw_image_rgb565(&screen2, 12, 0, 54, 54, (uint16_t*)gImage_new_eye_1_2);
        // vTaskDelay(100 / portTICK_PERIOD_MS);  //

        // // draw_image_rgb565(&screen1, 10, 0, 54, 45, (uint16_t*)gImage_new_eye_1_3);
        // // draw_image_rgb565(&screen2, 10, 0, 54, 45, (uint16_t*)gImage_new_eye_1_3);
        // // vTaskDelay(300 / portTICK_PERIOD_MS); //

        // st7735_fill_screen(&screen1, ST7735_BLACK);
        // st7735_fill_screen(&screen2, ST7735_BLACK);
        // vTaskDelay(100 / portTICK_PERIOD_MS);  //

        // draw_image_rgb565(&screen1, 30, 0, 54, 54, (uint16_t*)gImage_new_eye_1_2);
        // draw_image_rgb565(&screen2, 12, 0, 54, 54, (uint16_t*)gImage_new_eye_1_2);
        // vTaskDelay(100 / portTICK_PERIOD_MS);  //

        // draw_image_rgb565(&screen1, 30, 0, 54, 54, (uint16_t*)gImage_new_eye_1_1);
        // draw_image_rgb565(&screen2, 12, 0, 54, 54, (uint16_t*)gImage_new_eye_1_1);
        // // newIris = my_random(IRIS_MIN, IRIS_MAX);    //
        // // split(oldIris, newIris, esp_timer_get_time(), 5000000L, IRIS_MAX - IRIS_MIN);  //

        vTaskDelay(2500 / portTICK_PERIOD_MS);  // 确保任务不卡住
        vTaskDelay(1);
    }
}

class AtomS3EchoBaseBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    class DirectImageDisplay : public Display {
    public:
        explicit DirectImageDisplay(esp_lcd_panel_handle_t panel) : panel_(panel) {
            width_ = DISPLAY_WIDTH;
            height_ = DISPLAY_HEIGHT;
        }

        void SetupUI() override {
            xTaskCreatePinnedToCore(DrawBootImage, "task_eye_update", 1024 * 4, NULL, 4,
                                    &task_update_eye_handler, 0);
            // DrawBootImage();
        }

        void SetPowerSaveMode(bool on) override {
            // if (!on) {
            //     DrawBootImage();
            // }
        }

        void SetStatus(const char* status) override {}
        void ShowNotification(const char* notification, int duration_ms = 3000) override {}
        void SetEmotion(const char* emotion) override {}
        void SetChatMessage(const char* role, const char* content) override {}
        void ClearChatMessages() override {}
        void UpdateStatusBar(bool update_all = false) override {}

    private:
        esp_lcd_panel_handle_t panel_;

        // void DrawBootImage() {
        //     if (panel_ == nullptr) {
        //         return;
        //     }

        //     const int x = (width_ - kImageWidth) / 2;
        //     const int y = (height_ - kImageHeight) / 2;

        //     ESP_LOGI(TAG, "Draw static image without LVGL");
        //     esp_lcd_panel_draw_bitmap(panel_, x, y, x + kImageWidth, y + kImageHeight,
        //                               (void*)gImage_new_eye_1_1);
        // }

        bool Lock(int timeout_ms = 0) override { return true; }
        void Unlock() override {}
    };

    Display* display_;
    Button boot_button_;
    bool is_echo_base_connected_ = false;

    /* LCD IO and panel */
    // esp_lcd_panel_io_handle_t lcd_io1 = NULL;
    // esp_lcd_panel_handle_t lcd_panel1 = NULL;

    esp_lcd_panel_io_handle_t panel_io = nullptr;
    esp_lcd_panel_handle_t panel = nullptr;

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_1,
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
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void I2cDetect() {
        is_echo_base_connected_ = false;
        uint8_t echo_base_connected_flag = 0x00;
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                    if (address == 0x18) {
                        echo_base_connected_flag |= 0xF0;
                    } else if (address == 0x43) {
                        echo_base_connected_flag |= 0x0F;
                    }
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
        is_echo_base_connected_ = (echo_base_connected_flag == 0xFF);
    }

    void CheckEchoBaseConnection() {
        if (is_echo_base_connected_) {
            return;
        }

        // Pop error page
        InitializeSpi();
        // InitializeGc9107Display();
        InitializeButtons();
        GetBacklight()->SetBrightness(100);

        // Ensure UI is set up before displaying error
        display_->SetupUI();

        display_->SetStatus(Lang::Strings::ERROR);
        display_->SetEmotion("triangle_exclamation");
        display_->SetChatMessage("system", "Echo Base\nnot connected");

        while (1) {
            ESP_LOGE(TAG, "Atomic Echo Base is disconnected");
            vTaskDelay(pdMS_TO_TICKS(1000));

            // Rerun detection
            I2cDetect();
            if (is_echo_base_connected_) {
                vTaskDelay(pdMS_TO_TICKS(500));
                I2cDetect();
                if (is_echo_base_connected_) {
                    ESP_LOGI(TAG, "Atomic Echo Base is reconnected");
                    vTaskDelay(pdMS_TO_TICKS(200));
                    esp_restart();
                }
            }
        }
    }

    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize SPI bus");
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GC9A01_SPI1_LCD_GPIO_MOSI;
        buscfg.miso_io_num = GC9A01_SPI1_LCD_GPIO_MISO;
        buscfg.sclk_io_num = GC9A01_SPI1_LCD_GPIO_SCLK;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    // void InitializeGc9107Display() {
    //     ESP_LOGI(TAG, "Init GC9107 display");

    //     ESP_LOGI(TAG, "Install panel IO");
    //     esp_lcd_panel_io_handle_t io_handle = NULL;
    //     esp_lcd_panel_io_spi_config_t io_config = {};
    //     io_config.cs_gpio_num = GC9A01_SPI1_LCD_GPIO_CS;
    //     io_config.dc_gpio_num = GC9A01_SPI1_LCD_GPIO_DC;
    //     io_config.spi_mode = 0;
    //     io_config.pclk_hz = 40 * 1000 * 1000;
    //     io_config.trans_queue_depth = 10;
    //     io_config.lcd_cmd_bits = 8;
    //     io_config.lcd_param_bits = 8;
    //     ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &io_handle));

    //     ESP_LOGI(TAG, "Install GC9A01 panel driver");
    //     esp_lcd_panel_handle_t panel_handle = NULL;
    //     gc9a01_vendor_config_t gc9107_vendor_config = {
    //         .init_cmds = gc9107_lcd_init_cmds,
    //         .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
    //     };
    //     esp_lcd_panel_dev_config_t panel_config = {};
    //     panel_config.reset_gpio_num = GC9A01_SPI1_LCD_GPIO_RST;  // Set to -1 if not use
    //     panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;
    //     panel_config.bits_per_pixel = 16;  // Implemented by LCD command `3Ah` (16/18)
    //     panel_config.vendor_config = &gc9107_vendor_config;

    //     lcd_panel_eye = panel_handle;
    //     ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
    //     ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    //     ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    //     ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    //     display_ = new SpiLcdDisplay(io_handle, panel_handle, DISPLAY_WIDTH, DISPLAY_HEIGHT,
    //                                  DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
    //                                  DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    // }

    // // GC9A01初始化
    // void InitializeGc9107Display() {
    //     ESP_LOGI(TAG, "Init GC9A01 display1");

    //     ESP_LOGI(TAG, "Install panel IO1");
    //     ESP_LOGD(TAG, "Install panel IO1");
    //     const esp_lcd_panel_io_spi_config_t io_config = {
    //         .cs_gpio_num = GC9A01_SPI1_LCD_GPIO_CS,
    //         .dc_gpio_num = GC9A01_SPI1_LCD_GPIO_DC,
    //         .spi_mode = 0,
    //         .pclk_hz = GC9A01_LCD_PIXEL_CLK_HZ,
    //         .trans_queue_depth = 10,
    //         .lcd_cmd_bits = GC9A01_LCD_CMD_BITS,
    //         .lcd_param_bits = GC9A01_LCD_PARAM_BITS,

    //     };
    //     esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)GC9A01_LCD_SPI1_NUM, &io_config,
    //                              &lcd_io1);
    //     lcd_io_eye = lcd_io1;

    //     ESP_LOGD(TAG, "Install LCD1 driver");
    //     esp_lcd_panel_dev_config_t panel_config = {
    //         .reset_gpio_num = GC9A01_SPI1_LCD_GPIO_RST,
    //         .color_space = GC9A01_LCD_COLOR_SPACE,
    //         .bits_per_pixel = GC9A01_LCD_BITS_PER_PIXEL,

    //     };
    //     panel_config.rgb_endian = DISPLAY_RGB_ORDER;
    //     esp_lcd_new_panel_gc9a01(lcd_io1, &panel_config, &lcd_panel1);
    //     lcd_panel_eye = lcd_panel1;

    //     esp_lcd_panel_reset(lcd_panel1);
    //     esp_lcd_panel_init(lcd_panel1);
    //     esp_lcd_panel_invert_color(lcd_panel1, true);
    //     esp_lcd_panel_disp_on_off(lcd_panel1, true);

    //     display_ = new DirectImageDisplay(lcd_panel1);
    //     // display_->SetupUI();
    // }

    void InitGc9d01nDisplay() {
        ESP_LOGI(TAG, "Init GC9D01N");

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GC9A01_SPI1_LCD_GPIO_CS;
        io_config.dc_gpio_num = GC9A01_SPI1_LCD_GPIO_DC;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GC9A01_SPI1_LCD_GPIO_RST;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9d01n(panel_io, &panel_config, &panel));

        lcd_io_eye = panel_io;
        lcd_panel_eye = panel;

        esp_lcd_panel_reset(panel);

        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, false));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

        display_ = new DirectImageDisplay(panel);
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
    AtomS3EchoBaseBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        I2cDetect();
        // CheckEchoBaseConnection();
        InitializeSpi();
        InitGc9d01nDisplay();
        InitializeButtons();
        // GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            i2c_bus_, I2C_NUM_1, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_GPIO_PA, AUDIO_CODEC_ES8311_ADDR, false);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(AtomS3EchoBaseBoard);
