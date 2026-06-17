# tuzi_SD

ESP32S3 双舵机动作板，支持语音/MCP/WEB 控制。

## 快速开始

1. 烧录后板子上电 → WiFi 热点 `Xiaozhi-XXXX` 自动启动
2. 手机连接热点 → 浏览器打开 `http://192.168.4.1/action`
3. 或通过 MCP WebSocket 调用动作

## 引脚分配

| 功能 | GPIO | 说明 |
|------|------|------|
| I2S WS | 13 | 音频 |
| I2S BCLK | 48 | 音频 |
| I2S DIN | 14 | 音频 |
| I2S DOUT | 46 | 音频 |
| I2C SDA | 2 | ES8311 |
| I2C SCL | 38 | ES8311 |
| 舵机A | 15 | LEDC PWM 50Hz |
| 舵机B | 16 | LEDC PWM 50Hz |
| 舵机电源 | 4 | 高电平使能 |
| Boot 按键 | 0 | 短按切换聊天 |
| 电源按键 | 6 | ADC 读取, 1V 阈值, 长按2秒关机 |
| 电源锁存 | 7 | 上电拉高锁存 |
| 电池检测 | 3 | ADC 分压 1.426 |

## 动作库

### 预定义动作 (action_list.h)

| 名称 | 舵机 | 角度 | 默认速度 | 说明 |
|------|------|------|----------|------|
| center | AB | 90→90 | 500ms | 归中 |
| sweep | AB | 0↔180 | 500ms | 来回扫 |
| wave | A | 0↔180 | 300ms | A舵机招手 |
| nod | B | 45↔135 | 300ms | B舵机点头 |
| left | AB | 90→0 | 400ms | 向左 |
| right | AB | 90→180 | 400ms | 向右 |
| alternate | A | 0↔180 | 400ms×2 | 交替 |
| wag | A | 30↔150 | 200ms×3 | 快速摆 |
| shake | B | 60↔120 | 150ms×4 | 快速抖 |

### 添加自定义动作

通过 Web 页面 `http://192.168.4.1/action` 添加，存 NVS 重启不丢失。

或直接改 `action_list.h` 编译进去。

## MCP 工具

### 动作控制

| 工具 | 参数 | 示例 |
|------|------|------|
| `self.action.run` | `name`(必填), `speed`(ms,0=默认), `cycles`(次,0=默认) | `{"name":"wave","speed":300,"cycles":3}` |
| `self.action.list` | 无 | 返回动作名称列表 |
| `self.action.stop` | 无 | 停止当前动作 |

### 舵机控制

| 工具 | 参数 | 示例 |
|------|------|------|
| `self.servo.set_angle` | `servo`("A"/"B"), `angle`(0-180) | `{"servo":"A","angle":90}` |
| `self.servo.reset` | 无 | 双舵机归中 90° |
| `self.servo.sweep` | `on`(true/false) | 1Hz 扫动开关 |
| `self.servo.power` | `on`(true/false) | 舵机电源 IO4 开关 |

### 电池

| 工具 | 参数 | 返回 |
|------|------|------|
| `self.battery.status` | 无 | `"3680mV 85%"` |

## Web 管理页面

`http://192.168.4.1/action`

- 查看所有动作（预定义 + 自定义）
- 运行动作（支持速度/循环参数）
- 添加/删除自定义动作（NVS 存储）
- API 接口: `GET/POST /api/actions`, `POST /api/run`, `POST /api/stop`

## 配置开关 (config.h)

| 宏 | 默认 | 说明 |
|----|------|------|
| `XIAOZHI_CHAT_ENABLED` | 0 | 0=测试模式(无AI对话), 1=完整AI |
| `SERVO_SWEEP_ENABLED` | 0 | 0=上电不动, 1=上电自动1Hz扫 |
| `POWER_OUT_ACTIVE_LOW` | true | 电源按键低电平有效 |
| `POWER_LONG_PRESS_MS` | 2000 | 长按关机时间(ms) |
| `SERVO_POWER_GPIO` | 4 | 舵机电源控制脚 |
| `BATTERY_DIVIDER_RATIO` | 1.426 | 电池分压比 |

## 硬件

- 芯片: ESP32S3 (v0.2)
- PSRAM: 8MB Octal 80MHz (AP 64Mbit)
- Flash: 16MB QIO
- 音频: ES8311
