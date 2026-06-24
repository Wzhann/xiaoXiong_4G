# xiaoXiong_4G

基于 [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) (v2.2.4) 的 AI 语音对话机器人项目，包含 3 个定制板子。

---

## 板子

### 1. cat_tailAndNeck — 五舵机 AI 玩具猫 🐱

ESP32S3 + 5 路舵机 + 触摸 + 陀螺仪。仿动画片猫的机械结构。

| 功能 | 说明 |
|------|------|
| 舵机 | Neck×2(脖子) + Tail×2(尾巴) + Head×1(头), 50Hz PWM |
| 触摸 | IO5 4-bit R-2R DAC, 10 种手势组合 |
| 运动 | MPU-6050 陀螺仪, 抱起/摇晃/翻转/抚摸/掉落 检测 |
| 音频 | ES8311 codec, 29 个情绪 OGG (快乐/悲伤/愤怒/恐惧/惊讶/厌恶/中性) |
| 动作 | 91 个预设动作, cubic 缓动, 平均 2-3 秒 |

**交互**: 摸不同部位 → 情绪音频 + 三组舵机同步动作。组合锁防止动作堆积。

[MCP 工具]: `self.action.run/combo/list/stop`, `self.action.emote`, `self.battery.status`

[详细文档](xiaozhi-esp32-main/main/boards/cat_tailAndNeck/README.md)

### 2. tuzi_SD — 双舵机兔子 🐰

ESP32S3 + 2 路舵机(耳朵)。Web 页面直接控制。

| 功能 | 说明 |
|------|------|
| 舵机 | 2 路 PWM (右耳/左耳) |
| 控制 | Web 页面 `http://192.168.4.1/action` + MCP |
| 模式 | `config.h` 切换动作测试模式 / AI 聊天模式 |

[详细文档](xiaozhi-esp32-main/main/boards/tuzi_SD/README.md)

### 3. xiaoXiong_4G — 小熊 4G 版 🐻

ESP32 + ML307 4G 模组 + GC9D01N 圆形 LCD。

| 功能 | 说明 |
|------|------|
| 网络 | 4G (ML307) |
| 显示 | GC9D01N 圆形 LCD |
| 音频 | ES8311 |

[详细文档](xiaozhi-esp32-main/main/boards/xiaoXiong_4G/README.md)

---

## 构建

```bash
. /home/wzh/.espressif/v5.5.4/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

板子切换：修改 `sdkconfig` 中 `CONFIG_BOARD_TYPE_*`。

---

## 关键文件

| 文件 | 说明 |
|------|------|
| [CLAUDE.md](CLAUDE.md) | AI 编程助手指南 |
| `xiaozhi-esp32-main/main/boards/*/` | 板子代码 |
| `xiaozhi-esp32-main/main/assets/common/emotion_*.ogg` | 情绪音频 |
| `partitions/v2/16m_large_ota.csv` | cat_tailAndNeck 分区表 (OTA 7MB×2) |

## 音频来源

- 情绪音频: `/home/wzh/audio_mao/emotion_sorted/` (动画猫叫 MP3 → Opus OGG)
- 备用猫叫: `assets/common/maojiao/` (YouTube 真实猫叫)
