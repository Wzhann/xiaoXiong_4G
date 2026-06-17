#ifndef _ACTION_LIST_H_
#define _ACTION_LIST_H_

#include <cstdint>

// Servo selection: A=IO15(right ear), B=IO16(left ear)
#define SERVO_A 0
#define SERVO_B 1
#define SERVO_BOTH 2

// Action flags
#define ACTION_FLAG_BACK_FORTH 0x01
#define ACTION_FLAG_ALT       0x02  // Opposite: B = 180 - A

struct ServoAction {
    const char* name;
    uint8_t servo;
    uint8_t start_angle;
    uint8_t end_angle;
    uint16_t default_ms;
    uint8_t default_cycles;
    uint8_t flags;
};

// 180° = ear up, 0° = ear down
// A = IO15 = right ear, B = IO16 = left ear
static const ServoAction kActionList[] = {
    // ===== 双耳 (SERVO_BOTH) =====
    {"heart",          SERVO_BOTH, 0,    180, 500,  1,     0},
    {"heart_slow",     SERVO_BOTH, 0,    180, 1000, 1,     0},
    {"heart_fast",     SERVO_BOTH, 0,    180, 250,  1,     0},
    {"heart_back",     SERVO_BOTH, 180,  0,   500,  1,     0},
    {"ears_up",        SERVO_BOTH, 0,    180, 400,  1,     0},
    {"ears_down",      SERVO_BOTH, 180,  0,   400,  1,     0},
    {"ears_up_slow",   SERVO_BOTH, 0,    180, 800,  1,     0},
    {"ears_down_slow", SERVO_BOTH, 180,  0,   800,  1,     0},
    {"both_sweep",     SERVO_BOTH, 0,    180, 600,  1,     ACTION_FLAG_BACK_FORTH},
    {"both_sweep_fast",SERVO_BOTH, 0,    180, 300,  2,     ACTION_FLAG_BACK_FORTH},
    {"both_sweep_x2",  SERVO_BOTH, 0,    180, 400,  2,     ACTION_FLAG_BACK_FORTH},
    {"both_sweep_x3",  SERVO_BOTH, 0,    180, 350,  3,     ACTION_FLAG_BACK_FORTH},
    {"both_nod",       SERVO_BOTH, 135,  180, 250,  3,     ACTION_FLAG_BACK_FORTH},
    {"both_nod_fast",  SERVO_BOTH, 135,  180, 120,  5,     ACTION_FLAG_BACK_FORTH},
    {"both_nod_wide",  SERVO_BOTH, 90,   180, 350,  3,     ACTION_FLAG_BACK_FORTH},
    {"both_nod_tiny",  SERVO_BOTH, 160,  180, 150,  4,     ACTION_FLAG_BACK_FORTH},
    {"both_tiny",      SERVO_BOTH, 60,   120, 150,  4,     ACTION_FLAG_BACK_FORTH},
    {"both_tiny_fast", SERVO_BOTH, 75,   105, 80,   6,     ACTION_FLAG_BACK_FORTH},
    {"both_shake",     SERVO_BOTH, 80,   100, 60,   8,     ACTION_FLAG_BACK_FORTH},
    {"both_tremble",   SERVO_BOTH, 85,   95,  40,   12,    ACTION_FLAG_BACK_FORTH},
    {"both_mid",       SERVO_BOTH, 30,   150, 350,  2,     ACTION_FLAG_BACK_FORTH},
    {"both_mid_fast",  SERVO_BOTH, 30,   150, 180,  4,     ACTION_FLAG_BACK_FORTH},
    {"both_mid_slow",  SERVO_BOTH, 30,   150, 600,  1,     ACTION_FLAG_BACK_FORTH},
    {"both_mid_x5",    SERVO_BOTH, 45,   135, 250,  5,     ACTION_FLAG_BACK_FORTH},
    {"both_wide",      SERVO_BOTH, 0,    180, 800,  1,     ACTION_FLAG_BACK_FORTH},
    {"both_wide_x3",   SERVO_BOTH, 0,    180, 700,  3,     ACTION_FLAG_BACK_FORTH},
    {"both_up_hold",   SERVO_BOTH, 0,    180, 600,  1,     0},
    {"both_half_up",   SERVO_BOTH, 0,    90,  500,  1,     0},
    {"both_quarter",   SERVO_BOTH, 45,   135, 400,  1,     ACTION_FLAG_BACK_FORTH},

    // ===== 左耳 (IO16, SERVO_B) =====
    {"left_up",        SERVO_B,    0,    180, 400,  1,     0},
    {"left_down",      SERVO_B,    180,  0,   400,  1,     0},
    {"left_up_slow",   SERVO_B,    0,    180, 800,  1,     0},
    {"left_down_slow", SERVO_B,    180,  0,   800,  1,     0},
    {"left_wave",      SERVO_B,    0,    180, 300,  2,     ACTION_FLAG_BACK_FORTH},
    {"left_wave_fast", SERVO_B,    30,   150, 150,  4,     ACTION_FLAG_BACK_FORTH},
    {"left_wave_x3",   SERVO_B,    0,    180, 350,  3,     ACTION_FLAG_BACK_FORTH},
    {"left_wave_slow", SERVO_B,    0,    180, 600,  1,     ACTION_FLAG_BACK_FORTH},
    {"left_nod",       SERVO_B,    135,  180, 200,  3,     ACTION_FLAG_BACK_FORTH},
    {"left_nod_fast",  SERVO_B,    150,  180, 100,  5,     ACTION_FLAG_BACK_FORTH},
    {"left_tap",       SERVO_B,    150,  180, 100,  5,     ACTION_FLAG_BACK_FORTH},
    {"left_tap_x3",    SERVO_B,    160,  180, 80,   3,     ACTION_FLAG_BACK_FORTH},
    {"left_mid",       SERVO_B,    30,   150, 300,  2,     ACTION_FLAG_BACK_FORTH},
    {"left_mid_x5",    SERVO_B,    45,   135, 200,  5,     ACTION_FLAG_BACK_FORTH},
    {"left_shake",     SERVO_B,    80,   100, 60,   6,     ACTION_FLAG_BACK_FORTH},
    {"left_flick",     SERVO_B,    0,    90,  150,  3,     ACTION_FLAG_BACK_FORTH},

    // ===== 右耳 (IO15, SERVO_A) =====
    {"right_up",       SERVO_A,    0,    180, 400,  1,     0},
    {"right_down",     SERVO_A,    180,  0,   400,  1,     0},
    {"right_up_slow",  SERVO_A,    0,    180, 800,  1,     0},
    {"right_down_slow",SERVO_A,    180,  0,   800,  1,     0},
    {"right_wave",     SERVO_A,    0,    180, 300,  2,     ACTION_FLAG_BACK_FORTH},
    {"right_wave_fast",SERVO_A,    30,   150, 150,  4,     ACTION_FLAG_BACK_FORTH},
    {"right_wave_x3",  SERVO_A,    0,    180, 350,  3,     ACTION_FLAG_BACK_FORTH},
    {"right_wave_slow",SERVO_A,    0,    180, 600,  1,     ACTION_FLAG_BACK_FORTH},
    {"right_nod",      SERVO_A,    135,  180, 200,  3,     ACTION_FLAG_BACK_FORTH},
    {"right_nod_fast", SERVO_A,    150,  180, 100,  5,     ACTION_FLAG_BACK_FORTH},
    {"right_tap",      SERVO_A,    150,  180, 100,  5,     ACTION_FLAG_BACK_FORTH},
    {"right_tap_x3",   SERVO_A,    160,  180, 80,   3,     ACTION_FLAG_BACK_FORTH},
    {"right_mid",      SERVO_A,    30,   150, 300,  2,     ACTION_FLAG_BACK_FORTH},
    {"right_mid_x5",   SERVO_A,    45,   135, 200,  5,     ACTION_FLAG_BACK_FORTH},
    {"right_shake",    SERVO_A,    80,   100, 60,   6,     ACTION_FLAG_BACK_FORTH},
    {"right_flick",    SERVO_A,    0,    90,  150,  3,     ACTION_FLAG_BACK_FORTH},

    // ===== 扭动 (ALT: 双耳反向) =====
    {"twist",          SERVO_BOTH, 0,    180, 500,  1,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"twist_fast",     SERVO_BOTH, 0,    180, 250,  2,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"twist_slow",     SERVO_BOTH, 0,    180, 800,  1,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"twist_mid",      SERVO_BOTH, 30,   150, 350,  3,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"twist_tiny",     SERVO_BOTH, 60,   120, 200,  4,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"twist_shake",    SERVO_BOTH, 80,   100, 80,   6,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"twist_nod",      SERVO_BOTH, 135,  180, 200,  3,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},

    // 舞蹈
    {"dance",          SERVO_BOTH, 30,   150, 200,  4,     ACTION_FLAG_BACK_FORTH},
    {"dance_fast",     SERVO_BOTH, 50,   130, 130,  5,     ACTION_FLAG_BACK_FORTH},
    {"dance_wide",     SERVO_BOTH, 0,    180, 350,  3,     ACTION_FLAG_BACK_FORTH},
    {"dance_twist",    SERVO_BOTH, 30,   150, 250,  4,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},

    // wink
    {"wink_left",      SERVO_B,    90,   180, 150,  1,     ACTION_FLAG_BACK_FORTH},
    {"wink_right",     SERVO_A,    90,   180, 150,  1,     ACTION_FLAG_BACK_FORTH},
    {"wink_both",      SERVO_BOTH, 90,   180, 200,  2,     ACTION_FLAG_BACK_FORTH},

    // 情绪
    {"think",          SERVO_BOTH, 60,   120, 500,  2,     ACTION_FLAG_BACK_FORTH},
    {"think_left",     SERVO_B,    60,   120, 450,  2,     ACTION_FLAG_BACK_FORTH},
    {"think_right",    SERVO_A,    60,   120, 450,  2,     ACTION_FLAG_BACK_FORTH},
    {"happy",          SERVO_BOTH, 0,    180, 200,  3,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"excited",        SERVO_BOTH, 0,    180, 150,  4,     ACTION_FLAG_BACK_FORTH},
    {"greeting",       SERVO_BOTH, 0,    180, 400,  2,     ACTION_FLAG_BACK_FORTH},

    // 单次状态
    {"reset",          SERVO_BOTH, 90,   90,  300,  1,     0},
    {"sleep",          SERVO_BOTH, 180,  0,   800,  1,     0},
    {"wake",           SERVO_BOTH, 0,    180, 600,  1,     0},

    // ===== 组合表演 (更丰富的动作) =====
    // 左右交替摆动: A和B差180度，朝同一方向扭
    {"swing",          SERVO_BOTH, 0,    180, 400,  3,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"swing_fast",     SERVO_BOTH, 0,    180, 200,  5,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"swing_slow",     SERVO_BOTH, 0,    180, 700,  2,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"swing_wide",     SERVO_BOTH, 30,   150, 500,  3,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},

    // 波浪: 双耳同向但不同步(小延迟)
    {"wave_both",      SERVO_BOTH, 0,    180, 350,  3,     ACTION_FLAG_BACK_FORTH},
    {"wave_double",    SERVO_BOTH, 45,   135, 250,  4,     ACTION_FLAG_BACK_FORTH},

    // 快慢交替: 先快后慢的效果
    {"bounce",         SERVO_BOTH, 0,    180, 150,  3,     ACTION_FLAG_BACK_FORTH},
    {"bounce_twist",   SERVO_BOTH, 0,    180, 200,  3,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},

    // 一侧快一侧慢(ALT twist不同速度)
    {"flutter",        SERVO_BOTH, 60,   120, 100,  6,     ACTION_FLAG_BACK_FORTH},
    {"flutter_alt",    SERVO_BOTH, 60,   120, 120,  5,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},

    // 大动作
    {"cheer",          SERVO_BOTH, 0,    180, 300,  3,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"surprise",       SERVO_BOTH, 0,    180, 600,  1,     ACTION_FLAG_BACK_FORTH},
    {"shy",            SERVO_BOTH, 0,    45,  400,  1,     ACTION_FLAG_BACK_FORTH},
    {"proud",          SERVO_BOTH, 90,   180, 500,  1,     ACTION_FLAG_BACK_FORTH},
    {"curious",        SERVO_BOTH, 45,   135, 300,  2,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
    {"playful",        SERVO_BOTH, 0,    180, 200,  4,     ACTION_FLAG_BACK_FORTH | ACTION_FLAG_ALT},
};

static const int kActionCount = sizeof(kActionList) / sizeof(kActionList[0]);

#endif
