# cat_tailAndNeck

ESP32S3 五舵机猫板（脖子+尾巴+头），支持语音/MCP/触摸/陀螺仪控制。

## 快速开始

1. 烧录上电 → WiFi 自动连接或热点 `Xiaozhi-XXXX`
2. MCP WebSocket 控制动作
3. 触摸猫的不同部位触发动作
4. 摇晃/翻转猫触发动作

## 引脚分配

| 功能 | GPIO | 说明 |
|------|------|------|
| **舵机组** | | |
| 舵机电源 | 4 | HIGH=使能 |
| Neck0 (脖子) | 15 | LEDC CH0, PWM 50Hz |
| Neck1 (脖子) | 16 | LEDC CH1, PWM 50Hz |
| Tail0 (尾巴) | 17 | LEDC CH2, PWM 50Hz |
| Tail1 (尾巴) | 18 | LEDC CH3, PWM 50Hz |
| Head (头部) | 8 | LEDC CH4, PWM 50Hz |
| **音频** | | |
| I2S WS | 13 | |
| I2S BCLK | 48 | |
| I2S DIN | 14 | |
| I2S DOUT | 46 | |
| I2C SDA | 2 | ES8311 Codec |
| I2C SCL | 38 | ES8311 Codec |
| **触摸按键** | | |
| Touch (4-bit DAC) | 5 | ADC1_CH4, 过采样256x |
| **陀螺仪** | | |
| MPU-6050 SDA | 43 | I2C_NUM_1 |
| MPU-6050 SCL | 44 | I2C_NUM_1 |
| **电源管理** | | |
| 电源锁存 | 7 | HIGH=保持供电 |
| 电源按键 | 6 | ADC1_CH5, 长按2s关机 |
| 电池检测 | 3 | ADC1_CH2, 分压1.426 |
| **其他** | | |
| Boot 按键 | 0 | 短按切换聊天 |

## 舵机分组

| 组 | 舵机 | MCP 调用 |
|----|------|----------|
| **Neck** (脖子) | IO15, IO16 | `neck_*` |
| **Tail** (尾巴) | IO17, IO18 | `tail_*` |
| **Head** (头部) | IO8 | `head_*` |
| **All** (全部) | 全部5个 | `all_*` |

三组完全独立并行，可同时播放不同动作。

## 动作库 (25个)

### Neck 脖子 (1001-1006)
| ID | 名称 | 步数 | 时长 |
|----|------|------|------|
| 1001 | `neck_left` | 10 | 1s |
| 1002 | `neck_right` | 10 | 1s |
| 1003 | `neck_nod` | 21 | 2.1s |
| 1004 | `neck_shake` | 10 | 1s |
| 1005 | `neck_wave` | 17 | 1.7s |
| 1006 | `neck_loop` | 4 | 0.4s/循环 |

### Tail 尾巴 (2001-2007)
| ID | 名称 | 步数 | 时长 |
|----|------|------|------|
| 2001 | `tail_wag` | 17 | 1.7s |
| 2002 | `tail_wag_fast` | 17 | 1.7s |
| 2003 | `tail_up` | 21 | 2.1s |
| 2004 | `tail_down` | 21 | 2.1s |
| 2005 | `tail_tremble` | 8 | 0.8s |
| 2006 | `tail_curl` | 27 | 2.7s |
| 2007 | `tail_loop` | 4 | 0.4s/循环 |

### Head 头部 (3001-3006)
| ID | 名称 | 步数 | 时长 |
|----|------|------|------|
| 3001 | `head_left` | 10 | 1s |
| 3002 | `head_right` | 11 | 1.1s |
| 3003 | `head_nod` | 17 | 1.7s |
| 3004 | `head_shake` | 10 | 1s |
| 3005 | `head_tilt` | 21 | 2.1s |
| 3006 | `head_loop` | 4 | 0.4s/循环 |

### All 组合 (4001-4007)
| ID | 名称 | 步数 | 时长 |
|----|------|------|------|
| 4001 | `all_reset` | 3 | 0.3s |
| 4002 | `all_happy` | 31 | 3.1s |
| 4003 | `all_greeting` | 31 | 3.1s |
| 4004 | `all_sleep` | 16 | 1.6s |
| 4005 | `all_wake` | 16 | 1.6s |
| 4006 | `all_dance` | 29 | 2.9s |
| 4007 | `all_curious` | 29 | 2.9s |

## MCP 工具

### 动作控制

```json
// 按名字运行动作
{"tool":"self.action.run", "name":"tail_wag", "speed":0, "cycles":3, "loop":false}

// 按 ID 运行动作
{"tool":"self.action.run", "id":2001, "speed":0, "cycles":3}

// 列出所有动作
{"tool":"self.action.list"}

// 停止全部三组
{"tool":"self.action.stop"}

// 对话时自动表情
{"tool":"self.action.emote", "on":true}
```

### 电池

```json
{"tool":"self.battery.status"}  // → "3680mV 85%"
```

## 触摸按键

4 个触摸传感器通过 4-bit R-2R DAC 接入 IO5 (ADC1_CH4)：

| SW | 部位 | 触发动作 |
|----|------|----------|
| SW0 (bit0) | 下巴 | `neck_nod` |
| SW1 (bit1) | 头顶 | `neck_shake` |
| SW2 (bit2) | 左脸 | `tail_wag` |
| SW3 (bit3) | 右脸 | `head_nod` |
| SW0+SW1 | 头+下巴 | `all_happy` |
| SW2+SW3 | 双脸 | `all_dance` |

## 陀螺仪 (MPU-6050)

| 动作 | 触发 | 响应 |
|------|------|------|
| 抱起 | 突然加速 | `all_greeting` |
| 摇晃 | 快速角速度 | `all_dance` |
| 翻转 | 方向改变 | `all_curious` |
| 抚摸 | 小幅规律振动 | `neck_wave` + `tail_wag` |
| 放下/掉落 | 失重 | `all_sleep` |

## 添加自定义动作

使用 `/home/wzh/program/ESP32/action_cat` 录制动作用具：
1. 烧录 action_cat → 连 WiFi `CatRecorder` / `12345678`
2. 浏览器 `http://192.168.4.1`
3. 选择组 (Neck/Tail/Head/All)，设定时长
4. 摇动摇杆录制 → 每100ms记录一帧
5. 生成代码 → 复制粘贴到 `action_list.h`

## 硬件

- 芯片: ESP32S3
- 音频: ES8311
- 陀螺仪: MPU-6050 (I2C addr 0x68)
- 舵机: 5路 PWM 50Hz
- 触摸: 4-bit R-2R DAC
