# breathe_he — 呼吸爬行 AI 玩具熊猫

ESP32S3 驱动的 3 自由度 AI 玩具熊猫（趴姿）。支持呼吸（压缩气囊）、爬行（双手交替/同步）、
触摸情绪交互、陀螺仪运动检测、MCP 远程控制、语音对话时自动表情动作。

---

## 硬件架构

```
ESP32S3 (16MB Flash)
  ├── 音频: ES8311 (I2C0) → 录音/播放
  ├── 舵机: 3路 PWM 50Hz (LEDC) → 头×1 + 左手×1 + 右手×1 (270°舵机)
  ├── 触摸: IO5 (ADC1_CH4) → 4-bit R-2R DAC → 16级电压→按键组合
  ├── 陀螺: MPU-6050 (I2C1, 0x68) → 运动检测
  ├── 电池: IO3 (ADC1_CH2) → 电量
  └── 电源: IO7 锁存 + IO6 按键长按关机
```

## 引脚

| GPIO        | 功能           | 说明                                     |
| ----------- | -------------- | ---------------------------------------- |
| 15          | Head           | 头部左右转                               |
| 16          | Left Hand      | 左手，拉绳驱动                           |
| 17          | Right Hand     | 右手，拉绳驱动                           |
| 5           | Touch          | ADC1_CH4, 4-bit R-2R DAC, 禁用内部上下拉 |
| 4           | Servo PWR      | 舵机电源 HIGH=通                         |
| 43/44       | MPU SDA/SCL    | I2C_NUM_1                                |
| 2/38        | ES8311 SDA/SCL | I2C_NUM_0                                |
| 13/48/14/46 | I2S            | WS/BCLK/DIN/DOUT                         |
| 7/6/3       | 电源           | 锁存/按键ADC/电池ADC                     |
| 0           | Boot           | 短按切换聊天                             |

## 舵机角度 (270° 舵机)

三个舵机均为 270° 舵机，使用 -135° ~ +135° 用户坐标系（0° 为中值）。
PWM 映射：`pulse_us = 1500 + angle × 1000 / 135`（500-2500μs）。

| 舵机           | GPIO | -135° | 0° | +135° | 说明 |
| -------------- | ---- | ----- | -- | ----- | ---- |
| Head           | IO15 | 左转  | 正中 | 右转  |      |
| Left Hand      | IO16 | —     | 中立 | —     | -95°=爬行下压, +90°=压缩气囊 |
| Right Hand     | IO17 | —     | 中立 | —     | +95°=爬行下压, -90°=压缩气囊 |

### 运动逻辑

```
呼吸 (Breathing):
  左手 +90° ─┐
              ├─ 同时向中间压缩气囊 → 释放回 0°
  右手 -90° ─┘

爬行 (Crawling):
  交替模式: 左手 -95°(下压) → 回中 → 右手 +95°(下压) → 回中 → ...
  同步模式: 左手 -95° + 右手 +95° 同时下压 → 回中
```

---

## 运行逻辑

### 1. 动作系统

**四个独立执行器** 并行运行，各管一组舵机：

| 执行器             | 舵机           | 组掩码 | 队列深度 |
| ------------------ | -------------- | ------ | -------- |
| `head_exec_`     | Head           | 0x01   | 8        |
| `left_hand_exec_` | Left Hand      | 0x02   | 8        |
| `right_hand_exec_` | Right Hand    | 0x04   | 8        |
| `hands_exec_`     | Left + Right   | 0x06   | 8        |

`ActionExecutor` 内部是 FreeRTOS 任务 + Queue (深度 8):

1. `Run()` → `Enqueue()` → `xQueueSend(queue, req, 0)` **非阻塞**，队满丢弃 + warning
2. Task `xQueueReceive(portMAX_DELAY)` 永久阻塞等队 → `ExecuteAction()` → 逐帧 `PlayData()`
3. 每步: 写角度 → `vTaskDelay(step_ms)` → 一个动作完整执行完，不可抢占
4. `Stop()` → 设 `stop_requested_` + 清空队列 → 正在跑的动作立即中断
5. `ClearPending()` → 只清排队的，**不打断正在跑的**
6. `IsBusy()` → `running_` 或队列非空

**33 个动作**（全部使用 cubic ease-in-out 缓动）:

| 组        | ID 范围   | 数量 | 平均时长 |
| --------- | --------- | ---- | -------- |
| Head      | 1001-1009 | 9    | 2.5s     |
| LeftHand  | 2001-2004 | 4    | 2.3s     |
| RightHand | 3001-3004 | 4    | 2.3s     |
| Hands     | 4001-4006 | 6    | 2.8s     |
| All       | 5001-5010 | 10   | 3.0s     |

每组内部独立: head / left_hand / right_hand / hands 可同时各自执行不同动作。
`all_*` 动作是同一个步进数据广播到四个执行器。

每个动作的结构: **缓出→峰值停留→缓入**，不再生硬跳变。
步进帧率固定 100ms/步 (STEP_MS)。

#### 动作列表

**Head (头):**

| ID   | 名称          | 说明         |
| ---- | ------------- | ------------ |
| 1001 | head_left     | 头向左转     |
| 1002 | head_right    | 头向右转     |
| 1003 | head_nod      | 点头         |
| 1004 | head_shake    | 摇头         |
| 1005 | head_tilt_l   | 头左歪       |
| 1006 | head_tilt_r   | 头右歪       |
| 1007 | head_scan     | 左右扫视     |
| 1008 | head_curious  | 好奇歪头     |
| 1009 | head_calm     | 平静回中     |

**LeftHand (左手):**

| ID   | 名称       | 说明               |
| ---- | ---------- | ------------------ |
| 2001 | lh_crawl   | 下压爬行 (-95°)    |
| 2002 | lh_breathe | 压缩气囊呼吸 (+90°) |
| 2003 | lh_lift    | 轻抬               |
| 2004 | lh_wave    | 挥手               |

**RightHand (右手):**

| ID   | 名称       | 说明               |
| ---- | ---------- | ------------------ |
| 3001 | rh_crawl   | 下压爬行 (+95°)    |
| 3002 | rh_breathe | 压缩气囊呼吸 (-90°) |
| 3003 | rh_lift    | 轻抬               |
| 3004 | rh_wave    | 挥手               |

**Hands (双手协作):**

| ID   | 名称               | 说明           |
| ---- | ------------------ | -------------- |
| 4001 | hands_breathe      | 同步呼吸       |
| 4002 | hands_breatheslow  | 慢速呼吸       |
| 4003 | hands_crawlboth    | 双手同时爬     |
| 4004 | hands_crawlalt     | 双手交替爬     |
| 4005 | hands_rest         | 双手回中       |
| 4006 | hands_sway         | 双手摇摆       |

**All (全身组合):**

| ID   | 名称          | 说明               |
| ---- | ------------- | ------------------ |
| 5001 | all_reset     | 全身回中           |
| 5002 | all_breathe   | 呼吸 + 点头        |
| 5003 | all_crawl     | 爬行 + 头动        |
| 5004 | all_happy     | 开心摆动           |
| 5005 | all_curious   | 好奇探索           |
| 5006 | all_greeting  | 打招呼             |
| 5007 | all_sleep     | 入睡（手内收+低头）|
| 5008 | all_wake      | 醒来伸懒腰         |
| 5009 | all_stretch   | 大伸展             |
| 5010 | all_bow       | 鞠躬（手内收+点头）|

### 2. 触摸-情绪系统

```
IO5 电压 (16级R-2R DAC)
    │
    ▼
TouchRead() — ADC1_CH4, 过采样256次, 取平均电压
    │
    ▼
TouchLookup(mv) → 4-bit code (0x00~0x0F)
    │
    ▼
TouchMotionTask() — 20ms轮询, 5次稳定=按下, 8次稳定=释放
    │
    ▼
OnTouch(code) — 组合锁 + 情绪查表
    │
    ├── ClearPending()×4     — 清排队（不动正在跑的）
    ├── 等 IsBusy() 全 false — 四组当前动作跑完
    ├── pickSound()           — 2.5s冷却防重叠
    ├── head_exec_.Run()   ─┐
    ├── left_hand_exec_.Run()┤ 同时入队，同步启动
    ├── right_hand_exec_.Run()┤
    └── hands_exec_.Run()  ─┘
```

**组合锁**: 连续触摸→旧排队丢弃→等当前动作优雅跑完(2-3s)→新动作四组同步启动。停手后最多一个动作延迟就安静。

**触摸映射表:**

| 触摸      | Code | 情绪   | 音频池                | 动作特征              |
| --------- | ---- | ------ | --------------------- | --------------------- |
| 摸头      | 0x01 | 快乐   | emotion_happy_1~5     | 点头/歪头 + 呼吸/摇摆 |
| 摸背      | 0x02 | 舒服   | emotion_happy混合     | 平静+慢呼吸(cycles=2) |
| 摸左爪    | 0x04 | 好奇   | emotion_surprise_1~5  | 好奇头 + 左手探索     |
| 摸右爪    | 0x08 | 惊讶   | emotion_surprise_1~5  | 快速转头 + 右手探索   |
| 头+背     | 0x03 | 超开心 | emotion_happy_1~5     | 全组开心/打招呼       |
| 双爪      | 0x0C | 紧张   | emotion_fear_1~5      | 快速呼吸 + 摇头       |
| 头+左爪   | 0x05 | 撒娇   | emotion_sad_1~5       | 歪头+左手挥手(cycles=2) |
| 头+右爪   | 0x09 | 兴奋   | emotion_angry_1~5     | 快摇头+爬行(cycles=2)  |
| 背+左爪   | 0x06 | 困倦   | emotion_disgust+angry | 平静+慢呼吸(cycles=2)  |
| 背+右爪   | 0x0A | 平静   | emotion_neutral_1~3   | 平静+慢呼吸(cycles=2)  |
| 多点/其他 | —   | 默认   | emotion_neutral       | 平静呼吸              |

音频源: `assets/common/emotion_*.ogg` (29个情绪音效)
→ `gen_lang.py` 自动生成 `Lang::Sounds::OGG_EMOTION_*` 符号

**音效冷却**: 2.5 秒。前一个音频播放中，新触摸跳音频但动作立即执行。

### 3. 陀螺仪运动检测

```
MPU-6050 (I2C1, 100Hz)
    │
    ▼
mpu6050_detect_motion() — 加速度+角速度阈值判断
    │
    ▼
MOTION_PICKED_UP / MOTION_SHAKEN / MOTION_FLIPPED
/ MOTION_PETTING / MOTION_DROPPED
    │
    ▼
TouchMotionTask() — 每3次循环检测一次, 1500ms冷却
    │
    ├── pickSound(kXxxSounds)
    ├── head_exec_.Run(随机)
    └── hands_exec_.Run(随机)
```

可通过 MCP `self.action.emote` 开关。
陀螺仪日志每 3 秒打印一次 (tick%150)。

### 4. 对话自动表情

```
emote_task (4s±4s 随机间隔)
    │
    ▼
检测 DeviceState == Listening/Speaking
    │
    ▼
10% 概率触发 → 随机选择 单组/双组/三组/四组 → PlayRandomEmote()
    │
    ▼
30% 概率播放轻音效 (emotion_happy/neutral)
```

### 5. 电源管理

- IO7 上电立即拉高 → 锁存供电
- IO6 (ADC1_CH5) 检测按键: 电压<1V = 按下
- 长按 2 秒 → IO7 拉低 → 断电
- 电池 IO3 (ADC1_CH2), 分压比 1.426, 3.2V=0% / 4.2V=100%

---

## 小智框架对接

- `WifiBoard` 基类: 配网、OTA、网络连接
- `AudioCodec (ES8311)`: 16kHz 录音, 24kHz 播放
- `AudioService`: 云端 Opus 编解码 → 对话流
- `McpServer`: WebSocket MCP 协议 → 云端 AI 调用工具
- `DeviceStateMachine`: Idle→Connecting→Listening→Speaking 状态机
- `Application`: 事件循环 (FreeRTOS event group), `Schedule()` 线程安全投递

### MCP 工具

| 工具                      | 功能                             |
| ------------------------- | -------------------------------- |
| `self.action.run`       | 按名称或ID执行单个动作           |
| `self.action.combo`     | 同时控制 head + hands 两组       |
| `self.action.list`      | 列出所有33个动作名称             |
| `self.action.stop`      | 停止全部动作                     |
| `self.action.emote`     | 开关自动表情(对话时随机动作)     |
| `self.battery.status`   | 获取电量 mV + %                  |
| `self.breathing.control` | 呼吸控制 (频率+幅度+开关) |
| `self.crawling.control`  | 爬行控制 (start/stop/step, alt/both) |

---

## 关键参数

| 参数             | 值                             |
| ---------------- | ------------------------------ |
| 芯片             | ESP32S3 16MB Flash             |
| 步进时间 STEP_MS | 100ms                          |
| 舵机 PWM         | 50Hz, 13-bit, 500-2500μs      |
| 舵机类型         | 270° 舵机                     |
| 用户角度范围     | -135° ~ +135° (0°=中值)      |
| 触摸过采样       | 256x                           |
| 触摸防抖         | 按下5次(100ms), 释放8次(160ms) |
| 运动冷却         | 1500ms                         |
| 音效冷却         | 2500ms                         |
| 动作队列         | 8 (每组)                       |
| Opus 帧长        | 60ms                           |
| Opus 码率        | 64kbps                         |
| OpusHead SR      | 24000Hz (匹配输出)             |
| 陀螺仪日志       | 每3秒                          |
| OTA 分区         | 各 4MB                         |
| Assets 分区      | ~7.9MB                         |

## 固件

- 芯片: ESP32S3 16MB Flash
- OTA 分区: 各 4MB (自定义 `16m_large_ota.csv`)
- Assets 分区: ~7.9MB
- 当前固件: ~3.9MB (含29个 emotion OGG)
- 分区表: `partitions/v2/16m_large_ota.csv`
