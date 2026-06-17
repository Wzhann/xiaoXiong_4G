#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
   MPU-6050 6-axis IMU (Accelerometer + Gyroscope)
   I2C: IO42=SDA, IO41=SCL (I2C_NUM_1)
   ============================================================================ */

// 原始传感器数据
typedef struct {
    int16_t accel_x, accel_y, accel_z;   // 加速度 (raw, ±2g scale)
    int16_t gyro_x, gyro_y, gyro_z;      // 角速度 (raw, ±250°/s scale)
} mpu6050_data_t;

// 欧拉角 (度)
typedef struct {
    float pitch;   // 俯仰角, -180~180°  (前后倾斜)
    float roll;    // 横滚角, -90~90°    (左右倾斜)
    float yaw;     // 偏航角, -180~180°  (水平旋转, 陀螺仪积分, 有漂移)
} mpu6050_angles_t;

// 姿态方向
typedef enum {
    ORIENT_UP     = 0,   // 正面朝上 (正常)
    ORIENT_DOWN   = 1,   // 正面朝下 (被翻过来)
    ORIENT_LEFT   = 2,   // 左侧朝下
    ORIENT_RIGHT  = 3,   // 右侧朝下
    ORIENT_TILTED = 4,   // 倾斜中
} mpu6050_orient_t;

// 动作事件
typedef enum {
    MOTION_NONE      = 0,
    MOTION_PICKED_UP,     // 被抱起 (突然加速)
    MOTION_SHAKEN,        // 被摇动 (快速角速度变化)
    MOTION_FLIPPED,       // 被翻面 (方向改变)
    MOTION_PETTING,       // 被抚摸 (小幅规律振动)
    MOTION_DROPPED,       // 被放下/掉落 (自由落体, 失重)
} mpu6050_motion_t;

/**
 * @brief Initialize MPU-6050 on second I2C bus (IO42=SDA, IO41=SCL).
 * @return true on success.
 */
bool mpu6050_init(void);

/**
 * @brief Calibrate gyro bias. Call after system is stable (~5s after boot).
 *        Must hold sensor still during calibration.
 */
void mpu6050_calibrate(void);

/**
 * @brief Read raw sensor data.
 */
bool mpu6050_read(mpu6050_data_t *data);

/**
 * @brief Get current orientation.
 */
mpu6050_orient_t mpu6050_get_orientation(void);

/**
 * @brief Detect motion event (call periodically, ~50ms).
 *        Returns the detected motion type, or MOTION_NONE.
 */
mpu6050_motion_t mpu6050_detect_motion(void);

/**
 * @brief Get pitch/roll/yaw angles in degrees.
 *        pitch/roll from accelerometer, yaw from gyro integration (drifts over time).
 */
mpu6050_angles_t mpu6050_get_angles(void);

/**
 * @brief Print raw sensor data + angles to log (for debugging).
 */
void mpu6050_print_raw(void);

/**
 * @brief Get human-readable name for a motion event.
 */
const char *mpu6050_motion_name(mpu6050_motion_t motion);

/**
 * @brief Get human-readable name for an orientation.
 */
const char *mpu6050_orient_name(mpu6050_orient_t orient);

#ifdef __cplusplus
}
#endif

#endif // _MPU6050_H_
