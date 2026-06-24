# cat_tailAndNeck — 五舵机 AI 玩具猫

ESP32S3 驱动的 5 自由度机械猫。支持触摸情绪交互、陀螺仪运动检测、MCP 远程控制、
语音对话时自动表情动作。

---

## 硬件架构

```
ESP32S3 (16MB Flash)
  ├── 音频: ES8311 (I2C0) → 录音/播放
  ├── 舵机: 5路 PWM 50Hz (LEDC) → 脖子×2 + 尾巴×2 + 头×1
  ├── 触摸: IO5 (ADC1_CH4) → 4-bit R-2R DAC → 16级电压→按键组合
  ├── 陀螺: MPU-6050 (I2C1, 0x68) → 运动检测
  ├── 电池: IO3 (ADC1_CH2) → 电量
  └── 电源: IO7 锁存 + IO6 按键长按关机
```

## 引脚

| GPIO        | 功能           | 说明                                     |
| ----------- | -------------- | ---------------------------------------- |
| 18          | Neck0          | 脖子左右弯, 0°=左弯                     |
| 17          | Neck1          | 脖子前后倾, 0°=前倾                     |
| 15          | Tail0          | 尾巴上翘/平放, 0°=上翘~160°=平放       |
| 16          | Tail1          | 尾巴左右弯, 0°=左弯                     |
| 8           | Head           | 头左右转, 0°=左转                       |
| 5           | Touch          | ADC1_CH4, 4-bit R-2R DAC, 禁用内部上下拉 |
| 4           | Servo PWR      | 舵机电源 HIGH=通                         |
| 43/44       | MPU SDA/SCL    | I2C_NUM_1                                |
| 2/38        | ES8311 SDA/SCL | I2C_NUM_0                                |
| 13/48/14/46 | I2S            | WS/BCLK/DIN/DOUT                         |
| 7/6/3       | 电源           | 锁存/按键ADC/电池ADC                     |
| 0           | Boot           | 短按切换聊天                             |

## 舵机角度

| 舵机         | 0°      | 90° | 180°      | 反转 |
| ------------ | -------- | ---- | ---------- | ---- |
| Neck0 (IO18) | 左弯     | 直立 | 右弯       | ✓   |
| Neck1 (IO17) | 前倾     | 直立 | 后仰       | ✓   |
| Tail0 (IO15) | 上翘最高 | 中间 | ~160°平放 |      |
| Tail1 (IO16) | 左弯     | 中间 | 右弯       |      |
| Head (IO8)   | 左转     | 中间 | 右转       | ✓   |

存储角度 → `SetServoAngle()` 反转 → 物理 PWM, 500-2500μs → 0-180°.

---

## 运行逻辑（自研部分）

### 1. 动作系统

**三个独立执行器** 并行运行，各管一组舵机：

| 执行器         | 舵机         | 组掩码 | 队列深度 |
| -------------- | ------------ | ------ | -------- |
| `neck_exec_` | Neck0, Neck1 | 0x03   | 8        |
| `tail_exec_` | Tail0, Tail1 | 0x0C   | 8        |
| `head_exec_` | Head         | 0x10   | 8        |

`ActionExecutor` 内部是 FreeRTOS 任务 + Queue (深度 8):

1. `Run()` → `Enqueue()` → `xQueueSend(queue, req, 0)` **非阻塞**，队满丢弃 + warning
2. Task `xQueueReceive(portMAX_DELAY)` 永久阻塞等队 → `ExecuteAction()` → 逐帧 `PlayData()`
3. 每步: 写角度 → `vTaskDelay(step_ms)` → 一个动作完整执行完，不可抢占
4. `Stop()` → 设 `stop_requested_` + 清空队列 → 正在跑的动作立即中断
5. `ClearPending()` → 只清排队的，**不打断正在跑的**
6. `IsBusy()` → `running_` 或队列非空

**91 个动作** (V3, 全部使用 cubic ease-in-out 缓动):

| 组   | ID 范围   | 数量 | 平均时长 |
| ---- | --------- | ---- | -------- |
| Neck | 1001-1023 | 23   | 2.9s     |
| Tail | 2001-2023 | 23   | 2.5s     |
| Head | 3001-3025 | 25   | 2.2s     |
| All  | 4001-4020 | 20   | 2.3s     |

每组内部独立: neck/tail/head 可同时各自执行不同动作。
`all_*` 动作是同一个步进数据广播到三个执行器。

每个动作的结构: **缓出→峰值停留→缓入**, 不再生硬跳变.
步进帧率固定 100ms/步 (STEP_MS).

### 2. 触摸-情绪系统

```C
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
    ├── ClearPending()×3     — 清排队（不动正在跑的）
    ├── 等 IsBusy() 全 false — 三组当前动作跑完
    ├── pickSound()           — 2.5s冷却防重叠
    ├── neck_exec_.Run()  ─┐
    ├── tail_exec_.Run()  ─┤ 同时入队，同步启动
    └── head_exec_.Run()  ─┘
```

**组合锁**: 连续触摸→旧排队丢弃→等当前动作优雅跑完(2-3s)→新动作三组同步启动。停手后最多一个动作延迟就安静。

**触摸映射表:**

| 触摸      | Code | 情绪   | 音频池                | 动作特征              |
| --------- | ---- | ------ | --------------------- | --------------------- |
| 摸头      | 0x01 | 快乐   | emotion_happy_1~5     | 点头+摇尾+波浪        |
| 摸背      | 0x02 | 喜爱   | emotion_happy混合     | 慢动作(cycles=2)+呼噜 |
| 摸左爪    | 0x04 | 好奇   | emotion_surprise_1~5  | 歪头+问号尾+左顾      |
| 摸右爪    | 0x08 | 惊讶   | emotion_surprise_1~5  | 快速转头+弹尾         |
| 头+背     | 0x03 | 超开心 | emotion_happy_1~5     | 全组组合动作          |
| 双爪      | 0x0C | 恐惧   | emotion_fear_1~5      | 颤抖+缩尾+摇头        |
| 头+左爪   | 0x05 | 悲伤   | emotion_sad_1~5       | 低头+垂尾(cycles=2)   |
| 头+右爪   | 0x09 | 愤怒   | emotion_angry_1~5     | 快甩+猛摇(cycles=2)   |
| 背+左爪   | 0x06 | 厌恶   | emotion_disgust+angry | 后仰+甩尾+摇头        |
| 背+右爪   | 0x0A | 中性   | emotion_neutral_1~3   | 平静慢动作            |
| 多点/其他 | —   | 默认   | emotion_neutral       | 慢眨眼+轻摇           |

音频源: `/home/wzh/audio_mao/emotion_sorted/` (动画片猫叫 MP3)
→ ffmpeg 转 Opus 64kbps / 60ms帧 / 24kHz OpusHead → `assets/common/emotion_*.ogg`
→ `gen_lang.py` 自动生成 `Lang::Sounds::OGG_EMOTION_*` 符号

**音效冷却**: 2.5 秒. 前一个音频播放中, 新触摸跳音频但动作立即执行.
**解码队列**: 120 包(7.2秒), 防止长音频丢包.

### 3. 陀螺仪运动检测

```C
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
    ├── pickSound(kXxxSounds2)
    ├── neck_exec_.Run(随机)
    ├── tail_exec_.Run(随机)
    └── head_exec_.Run(随机)
```

可通过 MCP `self.action.emote` 的 `gyro` 参数开关.
陀螺仪日志每 3 秒打印一次 (tick%150).

### 4. 对话自动表情

```C
emote_task (4s±4s 随机间隔)
    │
    ▼
检测 DeviceState == Listening/Speaking
    │
    ▼
10% 概率触发 → 随机选择 单组/双组/三组 → PlayRandomEmote()
    │
    ▼
30% 概率播放轻喵 (emotion_happy/neutral)
```

### 5. 电源管理

- IO7 上电立即拉高 → 锁存供电
- IO6 (ADC1_CH5) 检测按键: 电压<1V = 按下
- 长按 2 秒 → IO7 拉低 → 断电
- 电池 IO3 (ADC1_CH2), 分压比 1.426, 3.2V=0% / 4.2V=100%

---

## 小智框架对接（简略）

- `WifiBoard` 基类: 配网、OTA、网络连接
- `AudioCodec (ES8311)`: 16kHz 录音, 24kHz 播放
- `AudioService`: 云端 Opus 编解码 → 对话流
- `McpServer`: WebSocket MCP 协议 → 云端 AI 调用工具
- `DeviceStateMachine`: Idle→Connecting→Listening→Speaking 状态机
- `Application`: 事件循环 (FreeRTOS event group), `Schedule()` 线程安全投递

### MCP 工具

| 工具                    | 功能                         |
| ----------------------- | ---------------------------- |
| `self.action.run`     | 按名称或ID执行单个动作       |
| `self.action.combo`   | 同时控制 neck+tail+head 三组 |
| `self.action.list`    | 列出所有91个动作名称         |
| `self.action.stop`    | 停止全部动作                 |
| `self.action.emote`   | 开关自动表情(对话时随机动作) |
| `self.battery.status` | 获取电量 mV + %              |

---

## 关键参数

| 参数             | 值                             |
| ---------------- | ------------------------------ |
| 步进时间 STEP_MS | 100ms                          |
| 舵机 PWM         | 50Hz, 13-bit, 500-2500μs      |
| 触摸过采样       | 256x                           |
| 触摸防抖         | 按下5次(100ms), 释放8次(160ms) |
| 运动冷却         | 1500ms                         |
| 音效冷却         | 2500ms                         |
| 解码队列         | 120 包 (7.2s@60ms帧)           |
| 动作队列         | 8 (每组)                       |
| Opus 帧长        | 60ms                           |
| Opus 码率        | 64kbps                         |
| OpusHead SR      | 24000Hz (匹配输出)             |
| 陀螺仪日志       | 每3秒                          |

## 固件

- 芯片: ESP32S3 16MB Flash
- OTA 分区: 各 7MB (自定义 `16m_large_ota.csv`)
- Assets 分区: 1.9MB
- 当前固件: ~4.0MB (含29个 emotion OGG, 43%空闲)
