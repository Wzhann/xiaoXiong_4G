#ifndef _CAT_ACTION_LIST_H_
#define _CAT_ACTION_LIST_H_

#include <cstdint>

// ============================================================
// Action system — 3 independent groups, 5 servos
//
// Each group uses its own step struct size:
//   Neck: ServoActionStep<2>  → {angle0, angle1}
//   Tail: ServoActionStep<2>  → {angle0, angle1}
//   Head: ServoActionStep<1>  → {angle}
//   All:  ServoActionStep<5>  → {n0, n1, t0, t1, head}
//
// Runtime uses flat int16_t* pointer. Timing: 100ms/step.
// ============================================================

#define STEP_MS 100

// Group masks
#define GROUP_NECK  0x03
#define GROUP_TAIL  0x0C
#define GROUP_HEAD  0x10
#define GROUP_ALL   0x1F

// Step struct — nice {{a,b},{c,d}} init syntax
template<int N>
struct ServoActionStep {
    int16_t angles[N];
};

// Runtime descriptor — flat int16_t data pointer
struct ServoActionSeries {
    const int16_t* data;     // = steps[0].angles
    uint8_t total_steps;
};

struct ServoAction {
    const char* name;
    uint16_t id;                   // numeric ID for easy calling
    const ServoActionSeries* series;
    uint8_t series_count;
    uint8_t group_mask;
};

// Action IDs
enum ActionID : uint16_t {
    // Neck: 10xx
    ACT_NECK_LEFT   = 1001,
    ACT_NECK_RIGHT  = 1002,
    ACT_NECK_NOD    = 1003,
    ACT_NECK_SHAKE  = 1004,
    ACT_NECK_WAVE   = 1005,
    ACT_NECK_LOOP   = 1006,

    // Tail: 20xx
    ACT_TAIL_WAG       = 2001,
    ACT_TAIL_WAG_FAST  = 2002,
    ACT_TAIL_UP        = 2003,
    ACT_TAIL_DOWN      = 2004,
    ACT_TAIL_TREMBLE   = 2005,
    ACT_TAIL_CURL      = 2006,
    ACT_TAIL_LOOP      = 2007,

    // Head: 30xx
    ACT_HEAD_LEFT   = 3001,
    ACT_HEAD_RIGHT  = 3002,
    ACT_HEAD_NOD    = 3003,
    ACT_HEAD_SHAKE  = 3004,
    ACT_HEAD_TILT   = 3005,
    ACT_HEAD_LOOP   = 3006,

    // All: 40xx
    ACT_ALL_RESET    = 4001,
    ACT_ALL_HAPPY    = 4002,
    ACT_ALL_GREETING = 4003,
    ACT_ALL_SLEEP    = 4004,
    ACT_ALL_WAKE     = 4005,
    ACT_ALL_DANCE    = 4006,
    ACT_ALL_CURIOUS  = 4007,
};

// ============================================================
// Neck: ServoActionStep<2> → 2値/step
// ============================================================

// neck_left: 10 steps = 1s
static const ServoActionStep<2> kNeckLeftSteps[] = {
    {{ 90, 90 }}, {{ 82, 82 }}, {{ 73, 73 }}, {{ 57, 57 }}, {{ 40, 40 }},
    {{ 30, 30 }}, {{ 40, 40 }}, {{ 57, 57 }}, {{ 73, 73 }}, {{ 90, 90 }},
};
static const ServoActionSeries kNeckLeftSeries[] = {
    {kNeckLeftSteps[0].angles, 10},
};

// neck_right: 10 steps = 1s
static const ServoActionStep<2> kNeckRightSteps[] = {
    {{ 90, 90 }}, {{ 98, 98 }}, {{107,107}}, {{123,123}}, {{140,140}},
    {{150,150}}, {{140,140}}, {{123,123}}, {{107,107}}, {{ 90, 90 }},
};
static const ServoActionSeries kNeckRightSeries[] = {
    {kNeckRightSteps[0].angles, 10},
};

// neck_nod: 21 steps ≈ 2.1s
static const ServoActionStep<2> kNeckNodSteps[] = {
    {{ 90, 90 }}, {{ 84, 84 }}, {{ 78, 78 }}, {{ 72, 72 }}, {{ 66, 66 }},
    {{ 60, 60 }}, {{ 66, 66 }}, {{ 72, 72 }}, {{ 78, 78 }}, {{ 84, 84 }},
    {{ 90, 90 }}, {{ 96, 96 }}, {{102,102}}, {{108,108}}, {{114,114}},
    {{120,120}}, {{114,114}}, {{108,108}}, {{102,102}}, {{ 96, 96 }},
    {{ 90, 90 }},
};
static const ServoActionSeries kNeckNodSeries[] = {
    {kNeckNodSteps[0].angles, 21},
};

// neck_shake: 10 steps = 1s
static const ServoActionStep<2> kNeckShakeSteps[] = {
    {{ 90, 90 }}, {{ 70,110}}, {{110, 70}}, {{ 70,110}}, {{110, 70}},
    {{ 70,110}}, {{110, 70}}, {{ 70,110}}, {{110, 70}}, {{ 90, 90 }},
};
static const ServoActionSeries kNeckShakeSeries[] = {
    {kNeckShakeSteps[0].angles, 10},
};

// neck_wave: 17 steps = 1.7s
static const ServoActionStep<2> kNeckWaveSteps[] = {
    {{ 90, 90 }}, {{ 78,102}}, {{ 66,114}}, {{ 54,126}}, {{ 42,138}},
    {{ 54,126}}, {{ 66,114}}, {{ 78,102}}, {{ 90, 90 }}, {{102, 78}},
    {{114, 66}}, {{126, 54}}, {{138, 42}}, {{126, 54}}, {{114, 66}},
    {{102, 78}}, {{ 90, 90 }},
};
static const ServoActionSeries kNeckWaveSeries[] = {
    {kNeckWaveSteps[0].angles, 17},
};

// neck_loop: 4 steps/cycle
static const ServoActionStep<2> kNeckLoopSteps[] = {
    {{ 90, 90 }}, {{ 60, 60 }}, {{ 90, 90 }}, {{120,120}},
};
static const ServoActionSeries kNeckLoopSeries[] = {
    {kNeckLoopSteps[0].angles, 4},
};

// ============================================================
// Tail: ServoActionStep<2> → 2値/step
// ============================================================

// tail_wag: 17 steps = 1.7s
static const ServoActionStep<2> kTailWagSteps[] = {
    {{ 90, 90 }}, {{ 78, 78 }}, {{ 66, 66 }}, {{ 54, 54 }}, {{ 42, 42 }},
    {{ 54, 54 }}, {{ 66, 66 }}, {{ 78, 78 }}, {{ 90, 90 }}, {{102,102}},
    {{114,114}}, {{126,126}}, {{138,138}}, {{126,126}}, {{114,114}},
    {{102,102}}, {{ 90, 90 }},
};
static const ServoActionSeries kTailWagSeries[] = {
    {kTailWagSteps[0].angles, 17},
};

// tail_wag_fast: 17 steps = 1.7s
static const ServoActionStep<2> kTailWagFastSteps[] = {
    {{ 90, 90 }}, {{ 60, 60 }}, {{ 30, 30 }}, {{ 60, 60 }}, {{ 90, 90 }},
    {{120,120}}, {{150,150}}, {{120,120}}, {{ 90, 90 }}, {{ 60, 60 }},
    {{ 30, 30 }}, {{ 60, 60 }}, {{ 90, 90 }}, {{120,120}}, {{150,150}},
    {{120,120}}, {{ 90, 90 }},
};
static const ServoActionSeries kTailWagFastSeries[] = {
    {kTailWagFastSteps[0].angles, 17},
};

// tail_up: 21 steps = 2.1s
static const ServoActionStep<2> kTailUpSteps[] = {
    {{ 90, 90 }}, {{ 84, 84 }}, {{ 78, 78 }}, {{ 72, 72 }}, {{ 66, 66 }},
    {{ 60, 60 }}, {{ 54, 54 }}, {{ 48, 48 }}, {{ 42, 42 }}, {{ 36, 36 }},
    {{ 30, 30 }}, {{ 36, 36 }}, {{ 42, 42 }}, {{ 48, 48 }}, {{ 54, 54 }},
    {{ 60, 60 }}, {{ 66, 66 }}, {{ 72, 72 }}, {{ 78, 78 }}, {{ 84, 84 }},
    {{ 90, 90 }},
};
static const ServoActionSeries kTailUpSeries[] = {
    {kTailUpSteps[0].angles, 21},
};

// tail_down: 21 steps = 2.1s
static const ServoActionStep<2> kTailDownSteps[] = {
    {{ 90, 90 }}, {{ 96, 96 }}, {{102,102}}, {{108,108}}, {{114,114}},
    {{120,120}}, {{126,126}}, {{132,132}}, {{138,138}}, {{144,144}},
    {{150,150}}, {{144,144}}, {{138,138}}, {{132,132}}, {{126,126}},
    {{120,120}}, {{114,114}}, {{108,108}}, {{102,102}}, {{ 96, 96 }},
    {{ 90, 90 }},
};
static const ServoActionSeries kTailDownSeries[] = {
    {kTailDownSteps[0].angles, 21},
};

// tail_tremble: 8 steps = 0.8s
static const ServoActionStep<2> kTailTrembleSteps[] = {
    {{ 90, 90 }}, {{ 75,105}}, {{105, 75}}, {{ 75,105}},
    {{105, 75}}, {{ 75,105}}, {{105, 75}}, {{ 90, 90 }},
};
static const ServoActionSeries kTailTrembleSeries[] = {
    {kTailTrembleSteps[0].angles, 8},
};

// tail_curl: 27 steps = 2.7s
static const ServoActionStep<2> kTailCurlSteps[] = {
    {{ 90, 90 }}, {{ 81, 99 }}, {{ 72,108}}, {{ 63,117}}, {{ 54,126}},
    {{ 45,135}}, {{ 36,144}}, {{ 27,153}}, {{ 36,144}}, {{ 45,135}},
    {{ 54,126}}, {{ 63,117}}, {{ 72,108}}, {{ 81, 99 }}, {{ 90, 90 }},
    {{ 99, 81 }}, {{108, 72}}, {{117, 63}}, {{126, 54}}, {{135, 45}},
    {{144, 36}}, {{135, 45}}, {{126, 54}}, {{117, 63}}, {{108, 72}},
    {{ 99, 81 }}, {{ 90, 90 }},
};
static const ServoActionSeries kTailCurlSeries[] = {
    {kTailCurlSteps[0].angles, 27},
};

// tail_loop: 4 steps/cycle
static const ServoActionStep<2> kTailLoopSteps[] = {
    {{ 90, 90 }}, {{ 50, 50 }}, {{ 90, 90 }}, {{130,130}},
};
static const ServoActionSeries kTailLoopSeries[] = {
    {kTailLoopSteps[0].angles, 4},
};

// ============================================================
// Head: ServoActionStep<1> → 1値/step
// ============================================================

// head_left: 10 steps = 1s
static const ServoActionStep<1> kHeadLeftSteps[] = {
    {{ 90 }}, {{ 78 }}, {{ 66 }}, {{ 54 }}, {{ 42 }},
    {{ 30 }}, {{ 42 }}, {{ 54 }}, {{ 78 }}, {{ 90 }},
};
static const ServoActionSeries kHeadLeftSeries[] = {
    {kHeadLeftSteps[0].angles, 10},
};

// head_right: 11 steps = 1.1s
static const ServoActionStep<1> kHeadRightSteps[] = {
    {{  90 }}, {{ 102 }}, {{ 114 }}, {{ 126 }}, {{ 138 }},
    {{ 150 }}, {{ 138 }}, {{ 126 }}, {{ 114 }}, {{ 102 }}, {{ 90 }},
};
static const ServoActionSeries kHeadRightSeries[] = {
    {kHeadRightSteps[0].angles, 11},
};

// head_nod: 17 steps = 1.7s
static const ServoActionStep<1> kHeadNodSteps[] = {
    {{ 90 }}, {{ 80 }}, {{ 70 }}, {{ 60 }}, {{ 50 }},
    {{ 60 }}, {{ 70 }}, {{ 80 }}, {{ 90 }}, {{100}},
    {{110}}, {{120}}, {{130}}, {{120}}, {{110}},
    {{100}}, {{ 90 }},
};
static const ServoActionSeries kHeadNodSeries[] = {
    {kHeadNodSteps[0].angles, 17},
};

// head_shake: 10 steps = 1s
static const ServoActionStep<1> kHeadShakeSteps[] = {
    {{ 90 }}, {{ 70 }}, {{110}}, {{ 70 }}, {{110}},
    {{ 70 }}, {{110}}, {{ 70 }}, {{110}}, {{ 90 }},
};
static const ServoActionSeries kHeadShakeSeries[] = {
    {kHeadShakeSteps[0].angles, 10},
};

// head_tilt: 21 steps = 2.1s
static const ServoActionStep<1> kHeadTiltSteps[] = {
    {{ 90 }}, {{ 82 }}, {{ 74 }}, {{ 66 }}, {{ 58 }},
    {{ 50 }}, {{ 58 }}, {{ 66 }}, {{ 74 }}, {{ 82 }},
    {{ 90 }}, {{ 98 }}, {{106}}, {{114}}, {{122}},
    {{130}}, {{122}}, {{114}}, {{106}}, {{ 98 }}, {{ 90 }},
};
static const ServoActionSeries kHeadTiltSeries[] = {
    {kHeadTiltSteps[0].angles, 21},
};

// head_loop: 4 steps/cycle
static const ServoActionStep<1> kHeadLoopSteps[] = {
    {{ 90 }}, {{ 60 }}, {{ 90 }}, {{120}},
};
static const ServoActionSeries kHeadLoopSeries[] = {
    {kHeadLoopSteps[0].angles, 4},
};

// ============================================================
// Combined: ServoActionStep<5> → 5値/step
// ============================================================

// all_reset: 3 steps
static const ServoActionStep<5> kAllResetSteps[] = {
    {{ 90, 90, 90, 90, 90 }},
    {{ 90, 90, 90, 90, 90 }},
    {{ 90, 90, 90, 90, 90 }},
};
static const ServoActionSeries kAllResetSeries[] = {
    {kAllResetSteps[0].angles, 3},
};

// all_happy: 31 steps ≈ 3.1s
static const ServoActionStep<5> kAllHappySteps[] = {
    {{ 90, 90, 90, 90, 90 }}, {{ 78,102, 78, 78, 80 }},
    {{ 66,114, 66, 66,100 }}, {{ 54,126, 54, 54, 80 }},
    {{ 66,114, 66, 66,100 }}, {{ 78,102, 78, 78, 80 }},
    {{ 90, 90, 90, 90, 90 }}, {{102, 78,102,102,100}},
    {{114, 66,114,114, 80 }}, {{126, 54,126,126,100}},
    {{114, 66,114,114, 80 }}, {{102, 78,102,102,100}},
    {{ 90, 90, 90, 90, 90 }}, {{ 78,102, 78, 78, 80 }},
    {{ 66,114, 66, 66,100 }}, {{ 54,126, 54, 54, 80 }},
    {{ 66,114, 66, 66,100 }}, {{ 78,102, 78, 78, 80 }},
    {{ 90, 90, 90, 90, 90 }}, {{102, 78,102,102,100}},
    {{114, 66,114,114, 80 }}, {{126, 54,126,126,100}},
    {{114, 66,114,114, 80 }}, {{102, 78,102,102,100}},
    {{ 90, 90, 90, 90, 90 }}, {{ 78,102, 78, 78, 80 }},
    {{ 66,114, 66, 66,100 }}, {{ 54,126, 54, 54, 80 }},
    {{ 66,114, 66, 66,100 }}, {{ 78,102, 78, 78, 80 }},
    {{ 90, 90, 90, 90, 90 }},
};
static const ServoActionSeries kAllHappySeries[] = {
    {kAllHappySteps[0].angles, 31},
};

// all_greeting: 31 steps ≈ 3.1s
static const ServoActionStep<5> kAllGreetingSteps[] = {
    {{ 90, 90, 90, 90, 90 }}, {{ 86, 86, 86, 86, 86 }},
    {{ 82, 82, 82, 82, 82 }}, {{ 78, 78, 78, 78, 78 }},
    {{ 74, 74, 74, 74, 74 }}, {{ 70, 70, 70, 70, 70 }},
    {{ 66, 66, 66, 66, 66 }}, {{ 62, 62, 62, 62, 62 }},
    {{ 58, 58, 58, 58, 58 }}, {{ 54, 54, 54, 54, 54 }},
    {{ 50, 50, 50, 50, 50 }}, {{ 46, 46, 46, 46, 46 }},
    {{ 42, 42, 42, 42, 42 }}, {{ 38, 38, 38, 38, 38 }},
    {{ 34, 34, 34, 34, 34 }}, {{ 30, 30, 30, 30, 30 }},
    {{ 34, 34, 34, 34, 34 }}, {{ 38, 38, 38, 38, 38 }},
    {{ 42, 42, 42, 42, 42 }}, {{ 46, 46, 46, 46, 46 }},
    {{ 50, 50, 50, 50, 50 }}, {{ 54, 54, 54, 54, 54 }},
    {{ 58, 58, 58, 58, 58 }}, {{ 62, 62, 62, 62, 62 }},
    {{ 66, 66, 66, 66, 66 }}, {{ 70, 70, 70, 70, 70 }},
    {{ 74, 74, 74, 74, 74 }}, {{ 78, 78, 78, 78, 78 }},
    {{ 82, 82, 82, 82, 82 }}, {{ 86, 86, 86, 86, 86 }},
    {{ 90, 90, 90, 90, 90 }},
};
static const ServoActionSeries kAllGreetingSeries[] = {
    {kAllGreetingSteps[0].angles, 31},
};

// all_sleep: 16 steps = 1.6s
static const ServoActionStep<5> kAllSleepSteps[] = {
    {{ 90, 90, 90, 90, 90 }}, {{ 87, 87, 87, 87, 88 }},
    {{ 84, 84, 84, 84, 86 }}, {{ 81, 81, 81, 81, 84 }},
    {{ 78, 78, 78, 78, 82 }}, {{ 75, 75, 75, 75, 80 }},
    {{ 72, 72, 72, 72, 78 }}, {{ 69, 69, 69, 69, 76 }},
    {{ 66, 66, 66, 66, 74 }}, {{ 63, 63, 63, 63, 72 }},
    {{ 60, 60, 60, 60, 70 }}, {{ 57, 57, 57, 57, 68 }},
    {{ 54, 54, 54, 54, 66 }}, {{ 51, 51, 51, 51, 64 }},
    {{ 48, 48, 48, 48, 62 }}, {{ 45, 45, 45, 45, 60 }},
};
static const ServoActionSeries kAllSleepSeries[] = {
    {kAllSleepSteps[0].angles, 16},
};

// all_wake: 16 steps = 1.6s
static const ServoActionStep<5> kAllWakeSteps[] = {
    {{ 45, 45, 45, 45, 60 }}, {{ 48, 48, 48, 48, 62 }},
    {{ 51, 51, 51, 51, 64 }}, {{ 54, 54, 54, 54, 66 }},
    {{ 57, 57, 57, 57, 68 }}, {{ 60, 60, 60, 60, 70 }},
    {{ 63, 63, 63, 63, 72 }}, {{ 66, 66, 66, 66, 74 }},
    {{ 69, 69, 69, 69, 76 }}, {{ 72, 72, 72, 72, 78 }},
    {{ 75, 75, 75, 75, 80 }}, {{ 78, 78, 78, 78, 82 }},
    {{ 81, 81, 81, 81, 84 }}, {{ 84, 84, 84, 84, 86 }},
    {{ 87, 87, 87, 87, 88 }}, {{ 90, 90, 90, 90, 90 }},
};
static const ServoActionSeries kAllWakeSeries[] = {
    {kAllWakeSteps[0].angles, 16},
};

// all_dance: 29 steps = 2.9s
static const ServoActionStep<5> kAllDanceSteps[] = {
    {{ 90, 90, 90, 90, 90 }}, {{ 70,110, 70,110, 70 }},
    {{110, 70,110, 70,110 }}, {{ 70,110, 70,110, 70 }},
    {{ 90, 90, 90, 90, 90 }}, {{110, 70,110, 70,110 }},
    {{ 70,110, 70,110, 70 }}, {{110, 70,110, 70,110 }},
    {{ 90, 90, 90, 90, 90 }}, {{ 70,110, 70,110, 70 }},
    {{110, 70,110, 70,110 }}, {{ 70,110, 70,110, 70 }},
    {{ 90, 90, 90, 90, 90 }}, {{110, 70,110, 70,110 }},
    {{ 70,110, 70,110, 70 }}, {{110, 70,110, 70,110 }},
    {{ 90, 90, 90, 90, 90 }}, {{ 70,110, 60,120, 70 }},
    {{110, 70,120, 60,110 }}, {{ 70,110, 60,120, 70 }},
    {{ 90, 90, 90, 90, 90 }}, {{110, 70,120, 60,110 }},
    {{ 70,110, 60,120, 70 }}, {{110, 70,120, 60,110 }},
    {{ 90, 90, 90, 90, 90 }}, {{ 70,110, 60,120, 70 }},
    {{110, 70,120, 60,110 }}, {{ 70,110, 60,120, 70 }},
    {{ 90, 90, 90, 90, 90 }},
};
static const ServoActionSeries kAllDanceSeries[] = {
    {kAllDanceSteps[0].angles, 29},
};

// all_curious: 29 steps = 2.9s
static const ServoActionStep<5> kAllCuriousSteps[] = {
    {{ 90, 90, 90, 90, 90 }}, {{ 84, 84, 84, 84, 84 }},
    {{ 78, 78, 78, 78, 78 }}, {{ 72, 72, 72, 72, 72 }},
    {{ 66, 66, 66, 66, 66 }}, {{ 60, 60, 60, 60, 60 }},
    {{ 54, 54, 54, 54, 54 }}, {{ 48, 48, 48, 48, 48 }},
    {{ 54, 54, 54, 54, 54 }}, {{ 60, 60, 60, 60, 60 }},
    {{ 66, 66, 66, 66, 66 }}, {{ 72, 72, 72, 72, 72 }},
    {{ 78, 78, 78, 78, 78 }}, {{ 84, 84, 84, 84, 84 }},
    {{ 90, 90, 90, 90, 90 }}, {{ 84, 84, 84, 84, 96 }},
    {{ 78, 78, 78, 78,102 }}, {{ 72, 72, 72, 72,108 }},
    {{ 66, 66, 66, 66,114 }}, {{ 60, 60, 60, 60,120 }},
    {{ 54, 54, 54, 54,126 }}, {{ 48, 48, 48, 48,132 }},
    {{ 54, 54, 54, 54,126 }}, {{ 60, 60, 60, 60,120 }},
    {{ 66, 66, 66, 66,114 }}, {{ 72, 72, 72, 72,108 }},
    {{ 78, 78, 78, 78,102 }}, {{ 84, 84, 84, 84, 96 }},
    {{ 90, 90, 90, 90, 90 }},
};
static const ServoActionSeries kAllCuriousSeries[] = {
    {kAllCuriousSteps[0].angles, 29},
};

// ============================================================
// Master list
// ============================================================
static const ServoAction kActionList[] = {
    // Neck: 1001-1006
    {"neck_left",       ACT_NECK_LEFT,       kNeckLeftSeries,    1, GROUP_NECK},
    {"neck_right",      ACT_NECK_RIGHT,      kNeckRightSeries,   1, GROUP_NECK},
    {"neck_nod",        ACT_NECK_NOD,        kNeckNodSeries,     1, GROUP_NECK},
    {"neck_shake",      ACT_NECK_SHAKE,      kNeckShakeSeries,   1, GROUP_NECK},
    {"neck_wave",       ACT_NECK_WAVE,       kNeckWaveSeries,    1, GROUP_NECK},
    {"neck_loop",       ACT_NECK_LOOP,       kNeckLoopSeries,    1, GROUP_NECK},

    // Tail: 2001-2007
    {"tail_wag",        ACT_TAIL_WAG,        kTailWagSeries,     1, GROUP_TAIL},
    {"tail_wag_fast",   ACT_TAIL_WAG_FAST,   kTailWagFastSeries, 1, GROUP_TAIL},
    {"tail_up",         ACT_TAIL_UP,         kTailUpSeries,      1, GROUP_TAIL},
    {"tail_down",       ACT_TAIL_DOWN,       kTailDownSeries,    1, GROUP_TAIL},
    {"tail_tremble",    ACT_TAIL_TREMBLE,    kTailTrembleSeries, 1, GROUP_TAIL},
    {"tail_curl",       ACT_TAIL_CURL,       kTailCurlSeries,    1, GROUP_TAIL},
    {"tail_loop",       ACT_TAIL_LOOP,       kTailLoopSeries,    1, GROUP_TAIL},

    // Head: 3001-3006
    {"head_left",       ACT_HEAD_LEFT,       kHeadLeftSeries,    1, GROUP_HEAD},
    {"head_right",      ACT_HEAD_RIGHT,      kHeadRightSeries,   1, GROUP_HEAD},
    {"head_nod",        ACT_HEAD_NOD,        kHeadNodSeries,     1, GROUP_HEAD},
    {"head_shake",      ACT_HEAD_SHAKE,      kHeadShakeSeries,   1, GROUP_HEAD},
    {"head_tilt",       ACT_HEAD_TILT,       kHeadTiltSeries,    1, GROUP_HEAD},
    {"head_loop",       ACT_HEAD_LOOP,       kHeadLoopSeries,    1, GROUP_HEAD},

    // Combined: 4001-4007
    {"all_reset",       ACT_ALL_RESET,       kAllResetSeries,    1, GROUP_ALL},
    {"all_happy",       ACT_ALL_HAPPY,       kAllHappySeries,    1, GROUP_ALL},
    {"all_greeting",    ACT_ALL_GREETING,    kAllGreetingSeries, 1, GROUP_ALL},
    {"all_sleep",       ACT_ALL_SLEEP,       kAllSleepSeries,    1, GROUP_ALL},
    {"all_wake",        ACT_ALL_WAKE,        kAllWakeSeries,     1, GROUP_ALL},
    {"all_dance",       ACT_ALL_DANCE,       kAllDanceSeries,    1, GROUP_ALL},
    {"all_curious",     ACT_ALL_CURIOUS,     kAllCuriousSeries,  1, GROUP_ALL},
};

static const int kActionCount = sizeof(kActionList) / sizeof(kActionList[0]);

#endif  // _CAT_ACTION_LIST_H_
