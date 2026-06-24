# CLAUDE.md

本仓库基于 **xiaozhi-esp32** (v2.2.4) — ESP32 AI 语音对话固件框架。在此基础上定制了 3 个板子。

---

## 构建

```bash
. /home/wzh/.espressif/v5.5.4/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
idf.py fullclean && idf.py build
```

Python 环境：
```
export IDF_PYTHON_ENV_PATH=/home/wzh/.espressif/python_env/idf5.5_py3.10_env
export PATH="$IDF_PYTHON_ENV_PATH/bin:$PATH"
```

---

## 三个自研板子

| 板子 | 目录 | 芯片 | 硬件特点 |
|------|------|------|----------|
| **xiaoXiong_4G** | `boards/xiaoXiong_4G/` | ESP32 + ML307 | 4G 通信, GC9D01N LCD |
| **cat_tailAndNeck** | `boards/cat_tailAndNeck/` | ESP32S3 | 5舵机猫, 触摸+陀螺仪+情绪 |
| **tuzi_SD** | `boards/tuzi_SD/` | ESP32S3 | 2舵机兔子, Web动作控制 |

切换板子：修改 `sdkconfig` 中的 `CONFIG_BOARD_TYPE_*`，当前默认 `cat_tailAndNeck`。

---

## cat_tailAndNeck — 五舵机 AI 玩具猫

### 硬件

| GPIO | 功能 | 说明 |
|------|------|------|
| 18 | Neck0 | 脖子左右弯, 0°=左弯, 180°=右弯 (反转) |
| 17 | Neck1 | 脖子前后倾, 0°=前倾, 180°=后仰 (反转) |
| 15 | Tail0 | 尾巴上翘/平放, 0°=上翘, ~160°=平放 |
| 16 | Tail1 | 尾巴左右弯, 0°=左弯, 180°=右弯 |
| 8 | Head | 头左右转, 0°=左转, 180°=右转 (反转) |
| 5 | Touch | ADC1_CH4, 4-bit R-2R DAC, 16级电压, 禁用内部上下拉 |
| 4 | Servo PWR | 舵机电源 |
| 43/44 | MPU SDA/SCL | I2C1, MPU-6050 陀螺仪 |
| 2/38 | ES8311 SDA/SCL | I2C0, 音频 |
| 7/6/3 | 电源 | 锁存/按键ADC/电池ADC |

音频文件：
- `assets/common/emotion_*.ogg` (29个) — 情绪音效，来自 `/home/wzh/audio_mao/emotion_sorted/`
- `assets/common/maojiao/` (21个) — YouTube 真实猫叫，备用
- `assets/common/anime_mew1.ogg` — 开机音

### 动作系统 (V3)

91 个动作，使用 cubic ease-in-out 缓动曲线。

| 组 | ID 范围 | 数量 | 平均时长 | 舵机 |
|----|---------|------|----------|------|
| Neck | 1001-1023 | 23 | 2.9s | Neck0, Neck1 |
| Tail | 2001-2023 | 23 | 2.5s | Tail0, Tail1 |
| Head | 3001-3025 | 25 | 2.2s | Head |
| All | 4001-4020 | 20 | 2.3s | 全部5个 |

关键文件：
- `action_list.h` — 动作数据 (`ServoActionStep<N>` 数组, 100ms/步)
- `action_executor.h` — `ActionExecutor` 类, FreeRTOS 任务+队列(深度8)

三组独立并行。每个动作结构: 缓出 → 峰值停留(6-12步) → 缓入。

### 触摸-情绪

4-bit R-2R DAC 支持 10 种触摸组合：

| 触摸 | Code | 情绪 | 音频 |
|------|------|------|------|
| 摸头 | 0x01 | 快乐 | emotion_happy_1~5 |
| 摸背 | 0x02 | 喜爱 | emotion_happy混合 |
| 摸左爪 | 0x04 | 好奇 | emotion_surprise_1~5 |
| 摸右爪 | 0x08 | 惊讶 | emotion_surprise_1~5 |
| 头+背 | 0x03 | 超开心 | emotion_happy_1~5 |
| 双爪 | 0x0C | 恐惧 | emotion_fear_1~5 |
| 头+左爪 | 0x05 | 悲伤 | emotion_sad_1~5 |
| 头+右爪 | 0x09 | 愤怒 | emotion_angry_1~5 |
| 背+左爪 | 0x06 | 厌恶 | emotion_disgust_1 |
| 背+右爪 | 0x0A | 中性 | emotion_neutral_1~3 |

### 组合锁

新触摸时: `ClearPending()` 清排队 → 等三组当前动作跑完(`IsBusy()`) → 新动作同时入队、同步启动。停手后最多一个动作延迟就安静。

音效 2.5s 冷却防重叠。解码队列 120 包(7.2s)防长音频丢包。

### 关键参数

| 参数 | 值 |
|------|-----|
| 步进 | 100ms |
| 舵机 PWM | 50Hz, 500-2500μs |
| 动作队列深度 | 8 (每组) |
| 解码队列 | 120 包 ≈ 7.2s |
| 音效冷却 | 2500ms |
| 运动冷却 | 1500ms |
| Opus | 64kbps, 60ms帧, 24kHz OpusHead |
| 分区 | OTA 7MB×2, 固件 ~4MB |

### MCP 工具

- `self.action.run` / `self.action.combo` / `self.action.list` / `self.action.stop`
- `self.action.emote` — 开关对话时自动表情
- `self.battery.status` — 电量

---

## tuzi_SD — 双舵机兔子

| GPIO | 功能 |
|------|------|
| 15 | 舵机A (右耳) |
| 16 | 舵机B (左耳) |
| 4 | 舵机电源 |
| 2/38 | ES8311 I2C |

关键文件：
- `action_list.h` — 动作数据
- `action_executor.h` — 动作执行器
- `action_web.h` — Web 页面 (内嵌 HTTP 服务器, 浏览器控制动作)
- `config.h` — `XIAOZHI_CHAT_ENABLED` 宏切换动作测试模式/AI聊天模式

配网后浏览器访问 `http://192.168.4.1/action` 直接控制兔子动作。

---

## 框架概述 (xiaozhi-esp32)

### Board 抽象

```
Board → WifiBoard / Ml307Board (4G)
```

每个板子 `.cc` 末尾 `DECLARE_BOARD(ClassName)` 注册工厂函数。

### 音频流水线

```
AudioCodec (I2S) → AudioProcessor (VAD/AEC) → WakeWord → AudioService
  ├── Input Task:  PCM → Opus → 云端
  └── Output Task: 云端 Opus → 解码 → 播放
```

Opus 16kHz 编码, 60ms 帧, 24kHz 输出。

### MCP Server

`McpServer::GetInstance()` 单例。板子构造函数中通过 `mcp.AddTool()` 注册工具。协议: WebSocket / MQTT。

### 设备状态

`Starting → Idle → Connecting → Listening → Speaking`

### 重要

- 板子 TAG 宏匹配目录名
- `gen_lang.py` 自动扫描 `assets/common/*.ogg` 生成 `lang_config.h` 符号
- 每个板子必须同时注册在 `main/CMakeLists.txt` 和 `main/Kconfig.projbuild`
- 分区表: `partitions/v2/16m_large_ota.csv` (cat_tailAndNeck 专用, OTA 7MB×2)
