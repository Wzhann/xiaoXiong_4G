#include "mpu6050.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <math.h>
#include <string.h>

#define TAG "mpu6050"

// I2C pins — second bus, separate from audio codec
#define MPU_I2C_SDA    GPIO_NUM_43
#define MPU_I2C_SCL    GPIO_NUM_44
#define MPU_I2C_PORT I2C_NUM_1
#define MPU_I2C_ADDR 0x68

// MPU-6050 registers
#define REG_WHO_AM_I 0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_SIGNAL_PATH 0x68
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B // 14 bytes: accel(6) + temp(2) + gyro(6)

// Scale factors: ACCEL_CONFIG=0x00→±2g, GYRO_CONFIG=0x00→±250°/s
#define ACCEL_SCALE  16384.0f  // LSB per g
#define GYRO_SCALE   131.0f    // LSB per °/s

// Motion detection thresholds (in physical units)
#define SHAKE_ACCEL_G      2.5f   // accel magnitude > 2.5g = shaking
#define SHAKE_GYRO_DPS     180.0f // gyro magnitude > 180°/s = spinning
#define REST_ACCEL_VAR     0.005f // resting: accel mag variance < 0.005 (stddev <0.07g)
#define REST_GYRO_MAX      30.0f  // resting: gyro magnitude < 30°/s
#define PICKUP_PERSIST     4      // 4 frames ≈80ms not-resting = picked up
#define DROP_ACCEL_G       0.3f   // freefall < 0.3g
#define PETTING_ACCEL_LO   1.2f   // petting accel low
#define PETTING_ACCEL_HI   1.8f   // petting accel high
#define PETTING_GYRO_MAX   30.0f  // petting gyro must be low
#define PETTING_PERSIST    3      // consecutive petting hits
#define PETTING_WINDOW_MS  1000   // need 3+ spikes within 1s
#define HIST_LEN           8      // ring buffer for variance calculation

static i2c_master_bus_handle_t g_i2c_bus = NULL;
static i2c_master_dev_handle_t g_i2c_dev = NULL;

// Gyro bias (calibrated at startup)
static float g_gyro_bias_x = 0, g_gyro_bias_y = 0, g_gyro_bias_z = 0;

// Ring buffer for accel magnitude variance
static float g_a_mag_hist[HIST_LEN] = {0};
static int g_hist_idx = 0;
static int g_hist_count = 0;

// Compute variance of ring buffer
static float ring_variance(void) {
    if (g_hist_count < 2) return 999.0f;
    float mean = 0, m2 = 0;
    int n = (g_hist_count > HIST_LEN) ? HIST_LEN : g_hist_count;
    for (int i = 0; i < n; i++) mean += g_a_mag_hist[i];
    mean /= n;
    for (int i = 0; i < n; i++) {
        float d = g_a_mag_hist[i] - mean;
        m2 += d * d;
    }
    return m2 / n;
}

// Push accel magnitude into ring buffer
static void ring_push(float a_mag) {
    g_a_mag_hist[g_hist_idx] = a_mag;
    g_hist_idx = (g_hist_idx + 1) % HIST_LEN;
    if (g_hist_count < HIST_LEN) g_hist_count++;
}

// Previous readings for motion detection
static mpu6050_data_t g_prev_data;
static mpu6050_orient_t g_prev_orient = ORIENT_UP;
static bool g_has_prev = false;
static int g_stationary_frames = 0;  // shared by pickup & drop
static int g_moving_frames = 0;
static int g_petting_count = 0;
static int g_petting_spikes = 0;
static uint32_t g_petting_window_start = 0;

// ---------------------------------------------------------------------------
// Low-level I2C helpers
// ---------------------------------------------------------------------------
static bool mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    esp_err_t ret = i2c_master_transmit(g_i2c_dev, buf, 2, pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "write reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
        return false;
    }
    return true;
}

static bool mpu_read_reg(uint8_t reg, uint8_t *val)
{
    esp_err_t ret = i2c_master_transmit_receive(g_i2c_dev, &reg, 1, val, 1,
                                                pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "read reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
        return false;
    }
    return true;
}

static bool mpu_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    esp_err_t ret = i2c_master_transmit_receive(g_i2c_dev, &reg, 1, buf, len,
                                                pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "read %u bytes from 0x%02X failed: %s",
                 (unsigned)len, reg, esp_err_to_name(ret));
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Sensor reading
// ---------------------------------------------------------------------------
bool mpu6050_read(mpu6050_data_t *data)
{
    if (g_i2c_dev == NULL)
        return false;

    uint8_t raw[14];
    if (!mpu_read_bytes(REG_ACCEL_XOUT_H, raw, 14))
    {
        return false;
    }

    // Big-endian 16-bit values
    data->accel_x = (int16_t)((raw[0] << 8) | raw[1]);
    data->accel_y = (int16_t)((raw[2] << 8) | raw[3]);
    data->accel_z = (int16_t)((raw[4] << 8) | raw[5]);
    // Skip temperature: raw[6], raw[7]
    data->gyro_x = (int16_t)((raw[8] << 8) | raw[9]);
    data->gyro_y = (int16_t)((raw[10] << 8) | raw[11]);
    data->gyro_z = (int16_t)((raw[12] << 8) | raw[13]);

    return true;
}

// ---------------------------------------------------------------------------
// Orientation detection
// ---------------------------------------------------------------------------
mpu6050_orient_t mpu6050_get_orientation(void)
{
    mpu6050_data_t data;
    if (!mpu6050_read(&data))
    {
        return ORIENT_UP;
    }

    int16_t ax = data.accel_x;
    int16_t ay = data.accel_y;
    int16_t az = data.accel_z;

    // Check which axis gravity is aligned with
    int32_t ax_abs = abs(ax);
    int32_t ay_abs = abs(ay);
    int32_t az_abs = abs(az);

    if (az_abs > ax_abs && az_abs > ay_abs)
    {
        if (az > 6000)
            return ORIENT_UP;
        if (az < -6000)
            return ORIENT_DOWN;
    }
    if (ax_abs > ay_abs && ax_abs > az_abs)
    {
        if (ax > 6000)
            return ORIENT_RIGHT;
        if (ax < -6000)
            return ORIENT_LEFT;
    }
    if (ay_abs > ax_abs && ay_abs > az_abs)
    {
        if (ay > 6000)
            return ORIENT_TILTED;
        if (ay < -6000)
            return ORIENT_TILTED;
    }

    return ORIENT_TILTED;
}

// ---------------------------------------------------------------------------
// Motion detection — using physical units (g, °/s)
// Based on pet robot examples: shake >2g/>150dps, petting 1.2-1.8g low gyro
// ---------------------------------------------------------------------------
static float accel_magnitude(const mpu6050_data_t* d) {
    float x = d->accel_x / ACCEL_SCALE;
    float y = d->accel_y / ACCEL_SCALE;
    float z = d->accel_z / ACCEL_SCALE;
    return sqrtf(x*x + y*y + z*z);
}

static float gyro_magnitude(const mpu6050_data_t* d) {
    float x = d->gyro_x / GYRO_SCALE - g_gyro_bias_x;
    float y = d->gyro_y / GYRO_SCALE - g_gyro_bias_y;
    float z = d->gyro_z / GYRO_SCALE - g_gyro_bias_z;
    return sqrtf(x*x + y*y + z*z);
}

mpu6050_motion_t mpu6050_detect_motion(void)
{
    mpu6050_data_t data;
    if (!mpu6050_read(&data)) return MOTION_NONE;

    if (!g_has_prev) {
        g_prev_data = data;
        g_prev_orient = mpu6050_get_orientation();
        g_has_prev = true;
        return MOTION_NONE;
    }

    float a_mag = accel_magnitude(&data);
    float g_mag = gyro_magnitude(&data);

    mpu6050_motion_t detected = MOTION_NONE;

    // Update resting state (shared by pickup + drop)
    ring_push(a_mag);
    bool is_resting = (ring_variance() < REST_ACCEL_VAR) && (g_mag < REST_GYRO_MAX);

    static int was_stationary = 0;  // snapshot before motion reset
    if (is_resting) {
        g_stationary_frames++;
        if (g_stationary_frames > 300) g_stationary_frames = 300;
        was_stationary = g_stationary_frames;
        g_moving_frames = 0;
    } else {
        g_moving_frames++;
        g_stationary_frames = 0;
    }

    // 1. Shaken: accel > 2g OR gyro > 100°/s (instant)
    if (a_mag > SHAKE_ACCEL_G || g_mag > SHAKE_GYRO_DPS) {
        detected = MOTION_SHAKEN;
    }
    // 2. Freefall: accel < 0.3g (true freefall, rare)
    else if (a_mag < DROP_ACCEL_G) {
        detected = MOTION_DROPPED;
        g_stationary_frames = 0;
    }
    // 3. Picked up: was stationary, now moving
    else if (was_stationary >= 15 && g_moving_frames >= PICKUP_PERSIST) {
        detected = MOTION_PICKED_UP;
        was_stationary = 0;
        g_moving_frames = 0;
    }
    // 4. Placed down: was moving for a while, now became stationary.
    //    Track peak moving_frames. When it resets to 0, check if it was high.
    else {
        static int peak_moving = 0;
        if (g_moving_frames > peak_moving) peak_moving = g_moving_frames;
        if (g_stationary_frames == 4 && peak_moving >= 25) {  // was moving 500ms+
            detected = MOTION_DROPPED;
            peak_moving = 0;
        }
        if (g_stationary_frames > 10) peak_moving = 0;  // idle long enough, reset
    }

    // 5. Flipped: orientation stabilizes in a DIFFERENT direction
    //    Count consecutive frames in same non-tilted orientation.
    //    When stable for 5 frames AND different from last stable → flip.
    mpu6050_orient_t orient = mpu6050_get_orientation();
    static mpu6050_orient_t stable_orient = ORIENT_UP;
    static int stable_count = 0;

    if (orient == g_prev_orient && orient != ORIENT_TILTED) {
        stable_count++;
        if (stable_count == 5 && orient != stable_orient && detected == MOTION_NONE) {
            detected = MOTION_FLIPPED;
            stable_orient = orient;
            stable_count = 0;
        }
    } else {
        stable_count = 0;
    }
    g_prev_orient = orient;

    // 5. Petting: accel 1.2-1.8g + low gyro, need 3+ spikes in 1s window
    if (detected == MOTION_NONE &&
        a_mag > PETTING_ACCEL_LO && a_mag < PETTING_ACCEL_HI &&
        g_mag < PETTING_GYRO_MAX)
    {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (g_petting_spikes == 0)
            g_petting_window_start = now;
        g_petting_spikes++;
        if (g_petting_spikes >= PETTING_PERSIST &&
            (now - g_petting_window_start) <= PETTING_WINDOW_MS)
        {
            detected = MOTION_PETTING;
            g_petting_spikes = 0;
        }
    } else if (detected == MOTION_NONE) {
        // Reset petting window if no petting-like data for >200ms
        g_petting_spikes = 0;
    }

    g_prev_data = data;
    return detected;
}

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------
const char *mpu6050_motion_name(mpu6050_motion_t motion)
{
    switch (motion)
    {
    case MOTION_PICKED_UP:
        return "被抱起";
    case MOTION_SHAKEN:
        return "被摇晃";
    case MOTION_FLIPPED:
        return "被翻面";
    case MOTION_PETTING:
        return "被抚摸";
    case MOTION_DROPPED:
        return "被放下";
    default:
        return "无";
    }
}

const char *mpu6050_orient_name(mpu6050_orient_t orient)
{
    switch (orient)
    {
    case ORIENT_UP:
        return "正面朝上";
    case ORIENT_DOWN:
        return "正面朝下";
    case ORIENT_LEFT:
        return "左侧朝下";
    case ORIENT_RIGHT:
        return "右侧朝下";
    case ORIENT_TILTED:
        return "倾斜中";
    default:
        return "???";
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
bool mpu6050_init(void)
{
    // --- Release IO41/IO42 from JTAG and set as GPIO with pull-ups ---
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MPU_I2C_SDA) | (1ULL << MPU_I2C_SCL),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // --- Init second I2C bus (I2C_NUM_1) ---
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = MPU_I2C_PORT,
        .sda_io_num = MPU_I2C_SDA,
        .scl_io_num = MPU_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &g_i2c_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // --- I2C scan ---
    ESP_LOGI(TAG, "I2C%d scan (SDA=%d, SCL=%d):", MPU_I2C_PORT, MPU_I2C_SDA, MPU_I2C_SCL);
    int found_count = 0;
    for (int addr = 0; addr < 128; addr += 16)
    {
        char line[64] = {0};
        int pos = snprintf(line, sizeof(line), "  0x%02x: ", addr);
        for (int j = 0; j < 16; j++)
        {
            uint8_t a = addr + j;
            if (i2c_master_probe(g_i2c_bus, a, pdMS_TO_TICKS(100)) == ESP_OK)
            {
                pos += snprintf(line + pos, sizeof(line) - pos, "%02x ", a);
                found_count++;
            }
            else
            {
                pos += snprintf(line + pos, sizeof(line) - pos, "-- ");
            }
        }
        ESP_LOGI(TAG, "%s", line);
    }
    ESP_LOGI(TAG, "I2C%d scan done: %d device(s) found", MPU_I2C_PORT, found_count);

    if (found_count == 0)
    {
        ESP_LOGE(TAG, "No I2C devices found on bus %d — check wiring (SDA=%d, SCL=%d)",
                 MPU_I2C_PORT, MPU_I2C_SDA, MPU_I2C_SCL);
        return false;
    }

    // --- Try to find MPU-6050 at 0x68 or 0x69 ---
    uint8_t mpu_addr = 0;
    if (i2c_master_probe(g_i2c_bus, 0x68, pdMS_TO_TICKS(100)) == ESP_OK)
    {
        mpu_addr = 0x68;
    }
    else if (i2c_master_probe(g_i2c_bus, 0x69, pdMS_TO_TICKS(100)) == ESP_OK)
    {
        mpu_addr = 0x69;
    }

    if (mpu_addr == 0)
    {
        ESP_LOGE(TAG, "MPU-6050 not found at 0x68 or 0x69");
        return false;
    }

    // --- Add device at found address ---
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = mpu_addr,
        .scl_speed_hz = 400000,
        .scl_wait_us = 5,  // small wait for clock stretching
    };
    ret = i2c_master_bus_add_device(g_i2c_bus, &dev_cfg, &g_i2c_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
        return false;
    }

    // --- WHO_AM_I check ---
    uint8_t whoami = 0;
    if (mpu_read_reg(REG_WHO_AM_I, &whoami))
    {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", whoami);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to read WHO_AM_I, continuing anyway...");
    }

    // --- Reset I2C bus (scan timeouts may have put it in invalid state) ---
    i2c_master_bus_reset(g_i2c_bus);

    // --- Wake up MPU-6050 (clear sleep bit) ---
    if (!mpu_write_reg(REG_PWR_MGMT_1, 0x00))
    {
        ESP_LOGE(TAG, "Failed to wake up MPU-6050");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // --- Reset signal path ---
    mpu_write_reg(REG_SIGNAL_PATH, 0x07);
    vTaskDelay(pdMS_TO_TICKS(50));

    // --- Config: DLPF = 5 (10Hz accel, 10Hz gyro bandwidth) ---
    mpu_write_reg(REG_CONFIG, 0x05);

    // --- Gyro config: ±250°/s (default) ---
    mpu_write_reg(REG_GYRO_CONFIG, 0x00);

    // --- Accel config: ±2g (default) ---
    mpu_write_reg(REG_ACCEL_CONFIG, 0x00);

    ESP_LOGI(TAG, "MPU-6050 initialized at 0x%02X (I2C%d, SDA=%d, SCL=%d)",
             mpu_addr, MPU_I2C_PORT, MPU_I2C_SDA, MPU_I2C_SCL);

    return true;
}

// Gyro bias calibration (call after system is stable, ~5s after boot)
void mpu6050_calibrate(void) {
    ESP_LOGI(TAG, "Calibrating gyro bias (keep still)...");
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int valid = 0;
    const int calib_samples = 100;
    for (int i = 0; i < calib_samples; i++) {
        mpu6050_data_t calib_data;
        if (mpu6050_read(&calib_data)) {
            sum_gx += calib_data.gyro_x / GYRO_SCALE;
            sum_gy += calib_data.gyro_y / GYRO_SCALE;
            sum_gz += calib_data.gyro_z / GYRO_SCALE;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (valid > 0) {
        g_gyro_bias_x = sum_gx / valid;
        g_gyro_bias_y = sum_gy / valid;
        g_gyro_bias_z = sum_gz / valid;
    }
    ESP_LOGI(TAG, "Gyro bias (%d samples): x=%.2f y=%.2f z=%.2f°/s",
             valid, g_gyro_bias_x, g_gyro_bias_y, g_gyro_bias_z);
}

// ---------------------------------------------------------------------------
// Angles
// ---------------------------------------------------------------------------
static float g_yaw = 0.0f;

mpu6050_angles_t mpu6050_get_angles(void)
{
    mpu6050_data_t data;
    mpu6050_angles_t angles = {0};
    if (!mpu6050_read(&data)) return angles;

    float ax = (float)data.accel_x;
    float ay = (float)data.accel_y;
    float az = (float)data.accel_z;

    // Pitch: rotation around Y axis (nose up/down)
    // Roll:  rotation around X axis (tilt left/right)
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float roll  = atan2f(ay, az);

    angles.pitch = pitch * 180.0f / M_PI;
    angles.roll  = roll  * 180.0f / M_PI;

    // Yaw: integrate gyro Z (will drift, resets on startup)
    float gyro_z_dps = (float)data.gyro_z / 131.0f;  // ±250°/s scale → dps
    g_yaw += gyro_z_dps * 0.02f;  // assume ~50ms interval
    // Wrap to ±180
    while (g_yaw > 180.0f)  g_yaw -= 360.0f;
    while (g_yaw < -180.0f) g_yaw += 360.0f;
    angles.yaw = g_yaw;

    return angles;
}

// ---------------------------------------------------------------------------
// Print raw data + angles for debugging (does NOT consume motion prev-values)
// ---------------------------------------------------------------------------
void mpu6050_print_raw(void)
{
    mpu6050_data_t data;
    if (!mpu6050_read(&data)) return;

    mpu6050_angles_t angles = mpu6050_get_angles();
    mpu6050_orient_t orient = mpu6050_get_orientation();

    ESP_LOGI(TAG,
             "MPU | accel:%6d %6d %6d | gyro:%6d %6d %6d | "
             "pitch:%6.1f° roll:%6.1f° yaw:%6.1f° | %s",
             data.accel_x, data.accel_y, data.accel_z,
             data.gyro_x, data.gyro_y, data.gyro_z,
             angles.pitch, angles.roll, angles.yaw,
             mpu6050_orient_name(orient));
}
