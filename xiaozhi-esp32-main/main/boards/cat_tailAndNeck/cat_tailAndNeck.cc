#include "action_executor.h"
#include "action_list.h"
#include "application.h"
#include "assets/lang_config.h"
#include "button.h"
#include "cat_audio.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "demo_actions.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "mpu6050.h"
#include "power_save_timer.h"
#include "wifi_board.h"
#include "wifi_configuration_ap.h"
#include "wifi_manager.h"
#include <esp_http_server.h>
#include <esp_wifi.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <cstdlib>

#define TAG "cat_tailAndNeck"

// Servo PWM config: 50Hz, 1MHz timer resolution, 13-bit duty
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_FREQ_HZ 50
#define SERVO_DUTY_RES LEDC_TIMER_13_BIT
#define SERVO_MAX_DUTY ((1 << 13) - 1)  // 8191
#define SERVO_PERIOD_US 20000           // 20ms

static constexpr int kServoCount = 5;
static constexpr ledc_channel_t kServoChannels[kServoCount] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};
static constexpr gpio_num_t kServoPins[kServoCount] = {SERVO_0_GPIO, SERVO_1_GPIO, SERVO_2_GPIO,
                                                       SERVO_3_GPIO, SERVO_4_GPIO};
static constexpr const char* kServoNames[kServoCount] = {"Neck0_IO18", "Neck1_IO17", "Tail0_IO15",
                                                         "Tail1_IO16", "Head_IO8"};

// Invert matching cat_actionRecord physical kit:
//   Neck0(IO18) 左右弯: false,  Neck1(IO17) 前后倾: false,
//   Tail0(IO15) 上翘/平放: false,  Tail1(IO16) 左右弯: true,
//   Head(IO8) 左右转: true
static constexpr bool kServoInvert[kServoCount] = {false, false, false, true, true};

// Convert angle (0-180) to LEDC duty for 500-2500us pulse
static uint32_t AngleToDuty(int angle) {
    angle = std::clamp(angle, 0, 180);
    // pulse_us = 500 + angle * 2000 / 180
    uint32_t pulse_us = 500 + angle * 2000 / 180;
    return pulse_us * (SERVO_MAX_DUTY + 1) / SERVO_PERIOD_US;
}

static const char kDemoHtml[] = R"raw(<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Cat Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#0d1117;color:#c9d1d9;padding:8px;max-width:440px;margin:auto}
h1{text-align:center;font-size:16px;color:#58a6ff;margin:4px 0 8px}
.card{background:#161b22;border-radius:8px;padding:8px;margin-bottom:6px;border:1px solid #30363d}
h2{font-size:13px;margin-bottom:4px;color:#58a6ff}
.btn{display:inline-block;padding:8px 12px;border:none;border-radius:5px;font-size:13px;cursor:pointer;color:#fff;margin:2px;min-width:56px}
.btn:active{opacity:.7}
.btn-demo{background:#238636}
.btn-stop{background:#da3633}
.btn-preset{background:#30363d;font-size:11px;padding:6px 10px}
.row{display:flex;gap:6px;flex-wrap:wrap;align-items:center}
.val{font-weight:bold;color:#58a6ff;font-size:15px}
.lbl{font-size:10px;color:#8b949e}
.stat-row{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px solid #21262d;font-size:12px}
.bar-bg{background:#21262d;border-radius:4px;height:16px;flex:1;overflow:hidden;margin:0 6px}
.bar-fill{background:linear-gradient(90deg,#238636,#2ea043);height:100%;border-radius:4px;transition:width .5s}
.toggle-sw{position:relative;display:inline-block;width:48px;height:26px}
.toggle-sw input{opacity:0;width:0;height:0}
.toggle-sl{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#30363d;border-radius:26px;transition:.3s}
.toggle-sl:before{position:absolute;content:"";height:20px;width:20px;left:3px;bottom:3px;background:#8b949e;border-radius:50%;transition:.3s}
input:checked+.toggle-sl{background:#238636}
input:checked+.toggle-sl:before{transform:translateX(22px);background:#fff}
.status-dot{display:inline-block;width:6px;height:6px;border-radius:50%;margin-right:3px;background:#30363d}
.status-dot.on{background:#2ea043;animation:pulse 1.5s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
.joy-row{display:flex;gap:10px;justify-content:center;align-items:center}
canvas{border-radius:50%;background:#0a0f1e;touch-action:none;display:block}
canvas.joy-neck{border:2px solid #4ecca3;box-shadow:0 0 10px rgba(78,204,163,.2)}
canvas.joy-tail{border:2px solid #4ea3cc;box-shadow:0 0 10px rgba(78,163,204,.2)}
.joy-info{text-align:center;min-width:70px}
.joy-info .jval{font-size:20px;font-weight:bold}
.joy-info .jlbl{font-size:10px;color:#8b949e}
.head-row{display:flex;align-items:center;gap:8px;justify-content:center}
.head-row input[type=range]{flex:1;max-width:260px}
.head-row .jval{font-size:18px;font-weight:bold;color:#a34ecc;min-width:40px;text-align:center}
#log{background:#0d1117;color:#8b949e;font-size:10px;padding:4px;border-radius:4px;max-height:80px;overflow-y:auto;font-family:monospace;line-height:1.4}
</style>
</head><body>
<h1>Cat Control</h1>

<div class="card">
<h2>Mode</h2>
<div class="row">
<label class="toggle-sw"><input type="checkbox" id="modeToggle" onchange="setMode(this.checked)"><span class="toggle-sl"></span></label>
<span style="font-size:13px;margin-left:8px" id="modeLabel">Touch/IMU Auto</span>
</div>
</div>

<div class="card" id="joyCard">
<h2>Joystick (Neck + Tail + Head)</h2>
<div class="joy-row">
<div>
<canvas class="joy-neck" id="joyN" width="150" height="150"></canvas>
<div class="joy-info"><span class="jlbl">Neck</span><br><span class="jval" style="color:#4ecca3" id="jN0">90</span>° <span class="jval" style="color:#4ecca3" id="jN1">90</span>°</div>
</div>
<div>
<canvas class="joy-tail" id="joyT" width="150" height="150"></canvas>
<div class="joy-info"><span class="jlbl">Tail</span><br><span class="jval" style="color:#4ea3cc" id="jT0">90</span>° <span class="jval" style="color:#4ea3cc" id="jT1">90</span>°</div>
</div>
</div>
<div class="head-row" style="margin-top:6px">
<span class="jlbl">Head</span>
<input type="range" id="sHead" min="0" max="180" value="90" oninput="onHead(this.value)">
<span class="jval" style="color:#a34ecc" id="jHead">90</span>°
</div>
<div class="row" style="margin-top:4px;justify-content:center">
<button class="btn btn-preset" onclick="preset(90,90,90,90,90)">Home</button>
<button class="btn btn-preset" onclick="preset(60,120,120,60,90)">左看</button>
<button class="btn btn-preset" onclick="preset(120,60,60,120,90)">右看</button>
<button class="btn btn-preset" onclick="preset(90,110,40,90,90)">上翘</button>
<button class="btn btn-preset" onclick="preset(90,70,140,90,90)">下垂</button>
</div>
</div>

<div class="card">
<h2>Demo Actions</h2>
<div class="row">
<button class="btn btn-demo" onclick="runDemo(0)">orig</button>
<button class="btn btn-demo" onclick="runDemo(1)">A3</button>
<button class="btn btn-demo" onclick="runDemo(2)">A6</button>
<button class="btn btn-demo" onclick="runDemo(3)">A7</button>
<button class="btn btn-demo" onclick="runDemo(4)">A9</button>
<button class="btn btn-stop" onclick="stopAll()">Stop</button>
</div>
</div>

<div class="card">
<h2>Status</h2>
<div id="statusPane">Loading...</div>
</div>

<script>
let lastSend=0,angles=[90,90,90,90,90],pollTimer;
const $=id=>document.getElementById(id);

async function api(url,body){
  try{
    let o=body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{method:'GET'};
    let r=await fetch(url,o);
    return await r.json();
  }catch(e){return null}
}

function sendAngles(){
  let now=Date.now();
  if(now-lastSend<50)return;
  lastSend=now;
  api('/api/servo',{angles:angles});
}

// Joystick: Neck
new function(){
  let c=$('joyN'),ctx=c.getContext('2d'),cx=75,cy=75,R=65,kr=18,kx=75,ky=75,active=false;
  function draw(){
    ctx.clearRect(0,0,150,150);
    ctx.beginPath();ctx.arc(cx,cy,R,0,6.28);ctx.strokeStyle='rgba(255,255,255,.15)';ctx.lineWidth=2;ctx.stroke();
    ctx.beginPath();ctx.moveTo(cx-R,cy);ctx.lineTo(cx+R,cy);ctx.moveTo(cx,cy-R);ctx.lineTo(cx,cy+R);
    ctx.strokeStyle='rgba(255,255,255,.06)';ctx.lineWidth=1;ctx.stroke();
    ctx.beginPath();ctx.arc(kx,ky,kr,0,6.28);
    let g=ctx.createRadialGradient(kx-4,ky-4,2,kx,ky,kr);
    g.addColorStop(0,'#5eeaaf');g.addColorStop(1,'#2d8a5e');
    ctx.fillStyle=g;ctx.fill();ctx.strokeStyle='rgba(255,255,255,.3)';ctx.lineWidth=2;ctx.stroke();
  }
  function move(e){
    e.preventDefault();
    let r=c.getBoundingClientRect(),mx=(e.clientX-r.left)*150/r.width,my=(e.clientY-r.top)*150/r.height;
    let dx=mx-cx,dy=my-cy,dist=Math.sqrt(dx*dx+dy*dy);
    if(dist>R){dx*=R/dist;dy*=R/dist;}
    kx=cx+dx;ky=cy+dy;
    angles[0]=Math.round(90+(kx-cx)/R*90);angles[1]=Math.round(90+(ky-cy)/R*90);
    angles[0]=Math.max(0,Math.min(180,angles[0]));angles[1]=Math.max(0,Math.min(180,angles[1]));
    $('jN0').textContent=angles[0];$('jN1').textContent=angles[1];
    sendAngles();draw();
  }
  function up(){
    if(!active)return;
    active=false;
    let t0=performance.now(),sx=kx,sy=ky,dx=cx-sx,dy=cy-sy;
    function anim(ts){
      let t=Math.min(1,(ts-t0)/200),ease=t<.5?4*t*t*t:1-Math.pow(-2*t+2,3)/2;
      kx=sx+dx*ease;ky=sy+dy*ease;
      angles[0]=Math.round(90+(kx-cx)/R*90);angles[1]=Math.round(90+(ky-cy)/R*90);
      angles[0]=Math.max(0,Math.min(180,angles[0]));angles[1]=Math.max(0,Math.min(180,angles[1]));
      $('jN0').textContent=angles[0];$('jN1').textContent=angles[1];
      sendAngles();draw();
      if(t<1)requestAnimationFrame(anim);else{kx=cx;ky=cy;draw();setTimeout(()=>{angles[0]=90;angles[1]=90;sendAngles();$('jN0').textContent='90';$('jN1').textContent='90'},100)}
    }
    requestAnimationFrame(anim);
  }
  c.addEventListener('pointerdown',e=>{active=true;move(e)});
  c.addEventListener('pointermove',e=>{if(active)move(e)});
  c.addEventListener('pointerup',up);
  c.addEventListener('pointerleave',up);
  draw();
}

// Joystick: Tail
new function(){
  let c=$('joyT'),ctx=c.getContext('2d'),cx=75,cy=75,R=65,kr=18,kx=75,ky=75,active=false;
  function draw(){
    ctx.clearRect(0,0,150,150);
    ctx.beginPath();ctx.arc(cx,cy,R,0,6.28);ctx.strokeStyle='rgba(255,255,255,.15)';ctx.lineWidth=2;ctx.stroke();
    ctx.beginPath();ctx.moveTo(cx-R,cy);ctx.lineTo(cx+R,cy);ctx.moveTo(cx,cy-R);ctx.lineTo(cx,cy+R);
    ctx.strokeStyle='rgba(255,255,255,.06)';ctx.lineWidth=1;ctx.stroke();
    ctx.beginPath();ctx.arc(kx,ky,kr,0,6.28);
    let g=ctx.createRadialGradient(kx-4,ky-4,2,kx,ky,kr);
    g.addColorStop(0,'#5eb8ea');g.addColorStop(1,'#2d6a8e');
    ctx.fillStyle=g;ctx.fill();ctx.strokeStyle='rgba(255,255,255,.3)';ctx.lineWidth=2;ctx.stroke();
  }
  function move(e){
    e.preventDefault();
    let r=c.getBoundingClientRect(),mx=(e.clientX-r.left)*150/r.width,my=(e.clientY-r.top)*150/r.height;
    let dx=mx-cx,dy=my-cy,dist=Math.sqrt(dx*dx+dy*dy);
    if(dist>R){dx*=R/dist;dy*=R/dist;}
    kx=cx+dx;ky=cy+dy;
    angles[2]=Math.round(90+(kx-cx)/R*90);angles[3]=Math.round(90+(ky-cy)/R*90);
    angles[2]=Math.max(0,Math.min(180,angles[2]));angles[3]=Math.max(0,Math.min(180,angles[3]));
    $('jT0').textContent=angles[2];$('jT1').textContent=angles[3];
    sendAngles();draw();
  }
  function up(){
    if(!active)return;
    active=false;
    let t0=performance.now(),sx=kx,sy=ky,dx=cx-sx,dy=cy-sy;
    function anim(ts){
      let t=Math.min(1,(ts-t0)/200),ease=t<.5?4*t*t*t:1-Math.pow(-2*t+2,3)/2;
      kx=sx+dx*ease;ky=sy+dy*ease;
      angles[2]=Math.round(90+(kx-cx)/R*90);angles[3]=Math.round(90+(ky-cy)/R*90);
      angles[2]=Math.max(0,Math.min(180,angles[2]));angles[3]=Math.max(0,Math.min(180,angles[3]));
      $('jT0').textContent=angles[2];$('jT1').textContent=angles[3];
      sendAngles();draw();
      if(t<1)requestAnimationFrame(anim);else{kx=cx;ky=cy;draw();setTimeout(()=>{angles[2]=90;angles[3]=90;sendAngles();$('jT0').textContent='90';$('jT1').textContent='90'},100)}
    }
    requestAnimationFrame(anim);
  }
  c.addEventListener('pointerdown',e=>{active=true;move(e)});
  c.addEventListener('pointermove',e=>{if(active)move(e)});
  c.addEventListener('pointerup',up);
  c.addEventListener('pointerleave',up);
  draw();
}

function onHead(v){
  angles[4]=parseInt(v);
  $('jHead').textContent=angles[4];
  sendAngles();
}

function preset(a0,a1,a2,a3,a4){
  angles=[a0,a1,a2,a3,a4];
  $('jN0').textContent=a0;$('jN1').textContent=a1;
  $('jT0').textContent=a2;$('jT1').textContent=a3;
  $('sHead').value=a4;$('jHead').textContent=a4;
  sendAngles();
}

async function runDemo(i){await api('/api/demo',{id:i})}
async function stopAll(){await api('/api/stop')}
async function setMode(on){
  await api('/api/mode',{demo_mode:on});
  $('modeLabel').textContent=on?'Phone Control':'Touch/IMU Auto';
}

async function pollStatus(){
  let s=await api('/api/status');
  if(!s){return}
  let bc=s.bat_pct>30?'#2ea043':(s.bat_pct>15?'#d29922':'#da3633');
  $('statusPane').innerHTML=
    '<div class="stat-row"><span>Battery</span><span class="val" style="color:'+bc+'">'+s.bat_pct+'% '+s.bat_mv+'mV</span></div>'+
    '<div class="stat-row"><span></span><div class="bar-bg"><div class="bar-fill" style="width:'+s.bat_pct+'%"></div></div></div>'+
    '<div class="stat-row"><span>Touch</span><span class="val">0x'+(s.touch_code||0).toString(16).padStart(2,'0').toUpperCase()+' ('+s.touch_mv+'mV)</span></div>'+
    '<div class="stat-row"><span>Gyro</span><span>'+s.gyro_ok+'</span></div>'+
    '<div class="stat-row"><span>Orient</span><span class="val">'+s.orient+'</span></div>'+
    '<div class="stat-row"><span>Action</span><span>'+(s.action_running?'<span class="status-dot on"></span>Running':'Idle')+'</span></div>';
  // Only update mode text, NOT the checkbox (user controls it)
  $('modeLabel').textContent=s.demo_mode?'Phone Control':'Touch/IMU Auto';
}

pollTimer=setInterval(pollStatus,1000);
pollStatus();
</script>
</body></html>)raw";
class CatTailAndNeckBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    PowerSaveTimer* power_save_timer_ = nullptr;
    TaskHandle_t power_monitor_task_ = nullptr;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_handle_ = nullptr;
    int battery_level_ = 0;
    int battery_vbat_filtered_mv_ = 0;
    int servo_angle_[kServoCount] = {90, 90, 90, 90, 90};
    bool emote_enabled_ = true;
    TickType_t last_sound_tick_ = 0;  // sound cooldown to prevent overlap
    static constexpr uint32_t SOUND_COOLDOWN_MS = 2500;  // wait for current sound to finish
    // 3 independent executors — each group runs its own actions in parallel
    ActionExecutor neck_exec_{GROUP_NECK, "neck"};
    ActionExecutor tail_exec_{GROUP_TAIL, "tail"};
    ActionExecutor head_exec_{GROUP_HEAD, "head"};


    // ===== Demo system: WiFi AP + web control =====
    bool demo_mode_ = false;
    httpd_handle_t web_server_ = nullptr;
    int last_touch_code_ = 0;
    int last_touch_mv_ = 0;
    static constexpr int kDemoCount = 5;

    // ===== Touch button + MPU-6050 gyro =====
    TaskHandle_t touch_task_ = nullptr;
    SemaphoreHandle_t adc_mutex_ = nullptr;
    QueueHandle_t cat_sound_queue_ = nullptr;
    bool mpu_ok_ = false;
    bool gyro_actions_enabled_ = true;  // MCP toggle for motion→action
    static constexpr int TOUCH_OVERSAMPLE = 256;
    static constexpr uint32_t TOUCH_DEBOUNCE_MS = 100;
    static constexpr uint32_t MOTION_COOLDOWN_MS = 1500;

    // Touch voltage windows (4-bit DAC: 16 levels, 206mV step)
    struct TouchLut {
        uint16_t lo, hi;
        uint8_t code;
    };
    static constexpr TouchLut kTouchLut[] = {
        {0, 130, 0x00},     {130, 290, 0x01},   {290, 480, 0x02},   {480, 670, 0x03},
        {670, 870, 0x04},   {870, 1070, 0x05},  {1070, 1280, 0x06}, {1280, 1490, 0x07},
        {1490, 1690, 0x08}, {1690, 1890, 0x09}, {1890, 2100, 0x0A}, {2100, 2310, 0x0B},
        {2310, 2510, 0x0C}, {2510, 2710, 0x0D}, {2710, 2910, 0x0E}, {2910, 3150, 0x0F},
    };
    static constexpr int kTouchLutN = sizeof(kTouchLut) / sizeof(kTouchLut[0]);

    int TouchLookup(int mv) {
        for (int i = 0; i < kTouchLutN; i++)
            if (mv >= kTouchLut[i].lo && mv <= kTouchLut[i].hi)
                return kTouchLut[i].code;
        return -1;
    }

    // Non-blocking: queue a cat sound for the playback task
    void TriggerCatSound(cat_sound_type_t type) {
        if (cat_sound_queue_) {
            xQueueSend(cat_sound_queue_, &type, 0);  // drop if full
        }
    }

    // Dedicated task: waits for sound events and plays PCM through codec
    void CatSoundTask() {
        auto* codec = GetAudioCodec();
        cat_sound_type_t type;
        while (true) {
            if (xQueueReceive(cat_sound_queue_, &type, portMAX_DELAY) != pdTRUE)
                continue;

            codec->EnableOutput(true);
            size_t total = 0;
            int16_t* samples = cat_sound_generate(type, AUDIO_OUTPUT_SAMPLE_RATE, &total);
            if (!samples || !total) continue;

            ESP_LOGI(TAG, "🔊 Cat: %s (%zu samples)", cat_sound_name(type), total);
            constexpr size_t kChunk = 240;
            size_t offset = 0;
            while (offset < total) {
                size_t chunk = total - offset;
                if (chunk > kChunk) chunk = kChunk;
                std::vector<int16_t> buf(samples + offset, samples + offset + chunk);
                codec->OutputData(buf);
                offset += chunk;
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            free(samples);
            ESP_LOGI(TAG, "✅ Done: %s", cat_sound_name(type));
        }
    }

    int TouchRead() {
        int64_t sum = 0;
        bool got_lock = (xSemaphoreTake(adc_mutex_, pdMS_TO_TICKS(50)) == pdTRUE);
        if (!got_lock) {
            static int lock_fail_cnt = 0;
            if (++lock_fail_cnt % 100 == 1)
                ESP_LOGW(TAG, "TouchRead: ADC mutex timeout (#%d)", lock_fail_cnt);
            return -1;  // skip this cycle
        }
        for (int i = 0; i < TOUCH_OVERSAMPLE; i++) {
            int raw;
            adc_oneshot_read(adc_handle_, ADC_CHANNEL_4, &raw);
            sum += raw;
        }
        xSemaphoreGive(adc_mutex_);
        int avg = sum / TOUCH_OVERSAMPLE;
        int mv = 0;
        esp_err_t r = adc_cali_raw_to_voltage(adc_cali_handle_, avg, &mv);
        if (r != ESP_OK) {
            mv = avg * 3300 / 4096;  // fallback
        }
        // Log raw ADC value periodically
        static int raw_log_tick = 0;
        if (++raw_log_tick % 50 == 0) {
            ESP_LOGI(TAG, "ADC raw avg=%d mv=%d code=%d", avg, mv, TouchLookup(mv));
        }
        return TouchLookup(mv);
    }

    // Touch → emotion → audio + action
    //   SW0 (0x01) = 摸头     → 快乐 (happy)
    //   SW1 (0x02) = 摸背     → 喜爱 (love/purr)
    //   SW2 (0x04) = 摸左爪   → 好奇 (curious)
    //   SW3 (0x08) = 摸右爪   → 惊讶 (surprised)
    //   SW0+SW1    = 摸头+背  → 快乐强化 (intense happy)
    //   SW2+SW3    = 摸双爪   → 恐惧 (fear)
    //   SW0+SW2    = 摸头+左爪 → 悲伤 (sad)
    //   SW0+SW3    = 摸头+右爪 → 愤怒 (angry)
    //   SW1+SW2    = 摸背+左爪 → 厌恶 (disgust)
    //   SW1+SW3    = 摸背+右爪 → 中性 (neutral)

    // ===== Demo entries: 5 custom actions paired with cat audio =====
    struct DemoEntry {
        const char* name;
        const ServoAction* action;
        const std::string_view* sound;
    };
    static constexpr ServoAction kDemoAct_orig = {"demo_orig", 9000, kDemoOrigSeries, 1, GROUP_ALL};
    static constexpr ServoAction kDemoAct_a3   = {"demo_a3",   9001, kDemoA3Series,   1, GROUP_ALL};
    static constexpr ServoAction kDemoAct_a6   = {"demo_a6",   9002, kDemoA6Series,   1, GROUP_ALL};
    static constexpr ServoAction kDemoAct_a7   = {"demo_a7",   9003, kDemoA7Series,   1, GROUP_ALL};
    static constexpr ServoAction kDemoAct_a9   = {"demo_a9",   9004, kDemoA9Series,   1, GROUP_ALL};
    static constexpr DemoEntry kDemoEntries[kDemoCount] = {
        {"orig", &kDemoAct_orig, &Lang::Sounds::OGG_DEMO_ORIG},
        {"A3",   &kDemoAct_a3,   &Lang::Sounds::OGG_DEMO_A3},
        {"A6",   &kDemoAct_a6,   &Lang::Sounds::OGG_DEMO_A6},
        {"A7",   &kDemoAct_a7,   &Lang::Sounds::OGG_DEMO_A7},
        {"A9",   &kDemoAct_a9,   &Lang::Sounds::OGG_DEMO_A9},
    };

    void RunDemo(int idx) {
        if (idx < 0 || idx >= kDemoCount) return;
        neck_exec_.Stop(); tail_exec_.Stop(); head_exec_.Stop();
        vTaskDelay(pdMS_TO_TICKS(100));
        neck_exec_.ClearPending(); tail_exec_.ClearPending(); head_exec_.ClearPending();
        auto& entry = kDemoEntries[idx];
        neck_exec_.Run(entry.action->name, 100, 1, false, 1.0f);
        tail_exec_.Run(entry.action->name, 100, 1, false, 1.0f);
        head_exec_.Run(entry.action->name, 100, 1, false, 1.0f);
        Application::GetInstance().PlaySound(*entry.sound);
        ESP_LOGI(TAG, "Demo: %s started", entry.name);
    }

    void OnTouch(int code) {
        if (code <= 0) return;

        // Combo lock: clear queued actions on all 3 groups, wait for running ones to finish
        neck_exec_.ClearPending();
        tail_exec_.ClearPending();
        head_exec_.ClearPending();
        uint16_t waited = 0;
        while ((neck_exec_.IsBusy() || tail_exec_.IsBusy() || head_exec_.IsBusy()) && waited < 5000) {
            vTaskDelay(pdMS_TO_TICKS(20));
            waited += 20;
        }

        auto& app = Application::GetInstance();
        auto pick = [](const char* const* list, int n) -> const char* {
            return list[rand() % n];
        };
        // Sound cooldown: prevent overlapping, wait for current to finish
        TickType_t now = xTaskGetTickCount();
        bool can_play_sound = ((now - last_sound_tick_) * portTICK_PERIOD_MS) >= SOUND_COOLDOWN_MS;
        auto pickSound = [&](const std::string_view* list, int n) {
            if (can_play_sound) {
                auto& picked = list[rand() % n];
                ESP_LOGI(TAG, "🔊 Touch sound: %.*s", (int)picked.size(), picked.data());
                app.PlaySound(picked);
                last_sound_tick_ = now;
            }
        };

        // ===== Unified audio pool — all 6 sounds, intensity + duration drive action =====
        struct AudioEntry { std::string_view sound; float intensity; float duration_ms; };
        static const AudioEntry kAudioPool[] = {
            {Lang::Sounds::OGG_CAT_B3,   0.30f, 5140},   // soft, gentle, ~5.1s
            {Lang::Sounds::OGG_CAT_A6,   0.45f, 6250},   // mid-soft, ~6.3s
            {Lang::Sounds::OGG_CAT_A23,  0.55f, 7410},   // mid, ~7.4s
            {Lang::Sounds::OGG_CAT_A1,   0.65f, 6460},   // mid-strong, ~6.5s
            {Lang::Sounds::OGG_CAT_B13,  0.80f, 5630},   // strong, ~5.6s
        };
        constexpr int kAudioPoolN = sizeof(kAudioPool) / sizeof(kAudioPool[0]);

        // Pick random audio → get intensity + duration for coordinated action timing
        auto pickAudio = [&](float* intensity_out, float* duration_ms_out) -> const std::string_view& {
            int idx = rand() % kAudioPoolN;
            *intensity_out = kAudioPool[idx].intensity;
            *duration_ms_out = kAudioPool[idx].duration_ms;
            ESP_LOGI(TAG, "🎵 Pick audio #%d dur=%.0fms int=%.2f", idx, kAudioPool[idx].duration_ms, kAudioPool[idx].intensity);
            return kAudioPool[idx].sound;
        };

        // Look up action's total_steps for duration matching
        auto getActionSteps = [&](const char* name) -> int {
            const ServoAction* a = neck_exec_.FindAction(name);
            if (a && a->series_count > 0) return a->series[0].total_steps;
            return 25;  // default ~2.5s at 100ms
        };

        // ===== Random helpers =====
        auto frand = [](float lo, float hi) -> float {
            return lo + (float)rand() / (float)RAND_MAX * (hi - lo);
        };
        auto rcycle = []() -> uint8_t {
            int r = rand() % 7;
            return (r < 2) ? 2 : (r < 5 ? 3 : (r < 6 ? 4 : 5));  // mostly 2-4, sometimes 5
        };
        auto rspeed = []() -> uint16_t {
            int r = rand() % 10;
            // Gentle bias: mostly slow (150-180ms), rarely normal (100ms)
            return (r < 5) ? 150 : (r < 8 ? 180 : (r == 8 ? 120 : 0));  // 0=100ms
        };
        // Unused variables removed for clean compile
        //   0-6: ALL only (5-servo synchronized)
        //   7-8: per-group combo
        //   9:   ALL + per-group back-to-back
        // Dispatch action synchronized to audio duration + intensity
        //   intensity → amplitude within [amp_lo, amp_hi]
        //   duration_ms → cycles auto-calculated so action ≈ audio length
        auto dispatch = [&](const char* const* acts, int act_n,
                            const char* const* necks, int nk_n,
                            const char* const* tails, int tl_n,
                            const char* const* heads, int hd_n,
                            float amp_lo, float amp_hi,
                            float audio_intensity, float audio_duration_ms) {
            // Soft cap amplitude at 1.8 for elegant movement (never exceed 2.0)
            float raw_amp = amp_lo + audio_intensity * (amp_hi - amp_lo);
            float amp = raw_amp > 1.8f ? 1.8f : raw_amp;
            SetAllAmplitude(amp);
            int mode = rand() % 10;
            uint16_t spd = rspeed();
            uint16_t step_ms = spd > 0 ? spd : STEP_MS;
            // Pick action first, then match cycles to audio duration (max 3 for grace)
            const char* primary = pick(acts, act_n);
            int steps = getActionSteps(primary);
            float one_cycle_ms = (float)steps * step_ms;
            int cycles = (int)(audio_duration_ms / one_cycle_ms + 0.5f);
            if (cycles < 1) cycles = 1;
            if (cycles > 3) cycles = 3;
            ESP_LOGI(TAG, "🎬 Action '%s' steps=%d spd=%d → %d cycles (audio %.0fms / cycle %.0fms)",
                     primary, steps, step_ms, cycles, audio_duration_ms, one_cycle_ms);
            if (mode < 7) {
                RunAllAction(primary, spd, cycles);
            } else if (mode < 9) {
                if (nk_n > 0) neck_exec_.Run(pick(necks, nk_n), spd, cycles);
                if (tl_n > 0) tail_exec_.Run(pick(tails, tl_n), spd, cycles);
                if (hd_n > 0) head_exec_.Run(pick(heads, hd_n), spd, cycles);
            } else {
                // Double: split cycles between ALL and per-group
                int half = (cycles + 1) / 2;
                RunAllAction(primary, spd, half);
                if (nk_n > 0) neck_exec_.Run(pick(necks, nk_n), spd, half);
                if (tl_n > 0) tail_exec_.Run(pick(tails, tl_n), spd, half);
                if (hd_n > 0) head_exec_.Run(pick(heads, hd_n), spd, half);
            }
        };


        // Update web status with touch code
        last_touch_code_ = code;
        // In demo mode, skip touch-triggered actions
        if (demo_mode_) return;
        switch (code) {
            case 0x01: { // 摸头 → 快乐  amp 1.4~1.8
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                static const char* acts[] = {"all_happy","all_greeting","all_playful","all_dance","all_love","all_head_tilt","all_stretch","all_peek"};
                static const char* necks[] = {"neck_nod","neck_wave","neck_tilt_left","neck_tilt_right","neck_glance_lu","neck_sway","neck_circle","neck_figure8","neck_curious"};
                static const char* tails[] = {"tail_wag","tail_up","tail_perk","tail_bounce","tail_quiver","tail_sway","tail_spiral","tail_scurve","tail_curl"};
                static const char* heads[] = {"head_nod","head_bob","head_sway","head_tilt","head_tilt_l","head_tilt_r","head_micro_l","head_micro_r","head_curious","head_loop"};
                dispatch(acts,9, necks,9, tails,9, heads,10, 1.4f, 1.8f, intensity, duration_ms);
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
            case 0x02: { // 摸背 → 喜爱  amp 1.0~1.5
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                static const char* acts[] = {"all_snuggle","all_love","all_slow_blink","all_head_tilt","all_peek","all_greeting"};
                static const char* necks[] = {"neck_nod_slow","neck_sway","neck_relax","neck_figure8","neck_nod"};
                static const char* tails[] = {"tail_sway","tail_wag_slow","tail_curl","tail_scurve","tail_rest"};
                static const char* heads[] = {"head_calm","head_relax","head_sway","head_tilt","head_micro_l"};
                dispatch(acts,6, necks,5, tails,5, heads,5, 1.0f, 1.5f, intensity, duration_ms);
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
            case 0x04: { // 摸左爪 → 好奇  amp 1.3~1.8
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                static const char* acts[] = {"all_curious","all_peek","all_head_tilt","all_stretch","all_alert"};
                static const char* necks[] = {"neck_curious","neck_glance_lu","neck_tilt_left","neck_figure8","neck_stretch","neck_wave"};
                static const char* tails[] = {"tail_question","tail_perk","tail_hook","tail_loop","tail_quiver"};
                static const char* heads[] = {"head_curious","head_glance_l","head_dbl_take","head_scan_l","head_tilt_l","head_loop"};
                dispatch(acts,5, necks,6, tails,5, heads,6, 1.3f, 1.8f, intensity, duration_ms);
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
            case 0x08: { // 摸右爪 → 惊讶  amp 1.5~1.8
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                static const char* acts[] = {"all_alert","all_dance","all_stretch","all_greeting","all_head_tilt","all_wake"};
                static const char* necks[] = {"neck_stretch","neck_backward","neck_circle","neck_curious","neck_glance_lu"};
                static const char* tails[] = {"tail_up","tail_perk","tail_bounce","tail_spiral","tail_scurve"};
                static const char* heads[] = {"head_dbl_take","head_alert","head_stretch","head_loop","head_tilt"};
                dispatch(acts,6, necks,5, tails,5, heads,5, 1.5f, 1.8f, intensity, duration_ms);
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
            case 0x03: { // 摸头+背 → 超开心  amp 1.6~1.8
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                float amp = 1.6f + intensity * 0.2f;
                if (amp > 1.8f) amp = 1.8f;
                SetAllAmplitude(amp);
                StopAllActions();
                static const char* acts[] = {"all_happy","all_dance","all_playful","all_greeting","all_love","all_stretch","all_head_tilt","all_curious"};
                const char* picked = pick(acts,8);
                uint16_t spd = rspeed();
                uint16_t step_ms = spd > 0 ? spd : STEP_MS;
                int steps = getActionSteps(picked);
                int cycles = (int)(duration_ms / ((float)steps * step_ms) + 0.5f);
                
            cycles = std::max(2, std::min(6, cycles));
                RunAllAction(picked, spd, cycles);
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
            case 0x0C: { // 摸双爪 → 恐惧  amp 1.0~1.3
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                static const char* acts[] = {"all_bow","all_sleep","all_slow_blink","all_stretch","all_head_tilt"};
                static const char* necks[] = {"neck_bow","neck_backward","neck_nod_slow","neck_lean_fwd"};
                static const char* tails[] = {"tail_curl","tail_droop","tail_rest","tail_hook"};
                static const char* heads[] = {"head_tilt_l","head_tilt_r","head_calm","head_scan_l"};
                dispatch(acts,5, necks,4, tails,4, heads,4, 1.0f, 1.3f, intensity, duration_ms);
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
            case 0x05: { // 摸头+左爪 → 悲伤  amp 0.5~0.8
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                static const char* acts[] = {"all_bow","all_sleep","all_slow_blink","all_snuggle"};
                static const char* necks[] = {"neck_bow","neck_nod_slow","neck_forward","neck_lean_fwd"};
                static const char* tails[] = {"tail_droop","tail_curl","tail_sway","tail_rest"};
                static const char* heads[] = {"head_tilt_l","head_left","head_calm","head_scan_l"};
                dispatch(acts,4, necks,4, tails,4, heads,4, 0.5f, 0.8f, intensity, duration_ms);
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
            case 0x09: { // 摸头+右爪 → 愤怒  amp 1.4~1.8
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                static const char* acts[] = {"all_angry","all_stretch","all_alert","all_dance"};
                static const char* necks[] = {"neck_stretch","neck_circle","neck_sway","neck_left","neck_right"};
                static const char* tails[] = {"tail_spiral","tail_sway","tail_curl","tail_up","tail_loop"};
                static const char* heads[] = {"head_sweep","head_dbl_take","head_alert","head_stretch"};
                dispatch(acts,4, necks,5, tails,5, heads,4, 1.4f, 1.8f, intensity, duration_ms);
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
            case 0x06: { // 摸背+左爪 → 厌恶  amp 1.1~1.5
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                static const char* acts[] = {"all_bow","all_reset","all_stretch","all_head_tilt"};
                static const char* necks[] = {"neck_backward","neck_bow","neck_nod","neck_sway"};
                static const char* tails[] = {"tail_sway","tail_swish","tail_droop","tail_curl"};
                static const char* heads[] = {"head_sweep","head_confused","head_tilt","head_scan_r"};
                dispatch(acts,4, necks,4, tails,4, heads,4, 1.1f, 1.5f, intensity, duration_ms);
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
            case 0x0A: { // 摸背+右爪 → 中性  amp 0.8~1.2
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                static const char* acts[] = {"all_head_tilt","all_peek","all_slow_blink","all_stretch","all_reset"};
                static const char* necks[] = {"neck_nod_slow","neck_sway","neck_relax","neck_glance_lu"};
                static const char* tails[] = {"tail_sway","tail_wag_slow","tail_rest","tail_curl"};
                static const char* heads[] = {"head_calm","head_left","head_right","head_scan_l","head_scan_r","head_micro_l","head_micro_r"};
                dispatch(acts,5, necks,4, tails,4, heads,7, 0.8f, 1.2f, intensity, duration_ms);
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
            default: { // 多点/其他 → 示好  amp 0.8~1.2
                float intensity, duration_ms; auto& snd = pickAudio(&intensity, &duration_ms);
                float amp = 0.8f + intensity * 0.4f;
                SetAllAmplitude(amp);
                if (rand() % 10 < 7) {
                    static const char* acts[] = {"all_slow_blink","all_head_tilt","all_peek","all_snuggle"};
                    RunAllAction(pick(acts,4), rspeed(), 1);
                } else {
                    static const char* necks[] = {"neck_sway","neck_nod_slow","neck_relax"};
                    static const char* tails[] = {"tail_sway","tail_wag_slow","tail_rest"};
                    static const char* heads[] = {"head_calm","head_sway","head_micro_l","head_micro_r"};
                    neck_exec_.Run(pick(necks,3),rspeed(),1); tail_exec_.Run(pick(tails,3),rspeed(),1); head_exec_.Run(pick(heads,4),rspeed(),1);
                }
                if (can_play_sound) { app.PlaySound(snd); last_sound_tick_ = now; }
                break;
            }
        }
    }

    // Motion detection task (touch + MPU-6050)
    void TouchMotionTask() {
        vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for system init
        ESP_LOGI(TAG, "Touch & motion task started (mpu=%s)", mpu_ok_ ? "OK" : "NONE");

        // Calibrate gyro bias now that system is stable
        if (mpu_ok_) {
            mpu6050_calibrate();
            ESP_LOGI(TAG, "Gyro calibrated, motion detection active");
        }

        uint32_t last_motion_ms = 0;
        int last_touch = -1, stable_cnt = 0, active_touch = -1;
        while (true) {
            // 1. Read touch buttons (every 20ms)
            int touch = TouchRead();

            // Debug: log every 50 cycles (~1s)
            static int dbg_tick = 0;
            dbg_tick++;
            if (dbg_tick % 50 == 0) {
                ESP_LOGI(TAG, "Touch dbg: read=%d last=%d stable=%d active=%d", touch, last_touch, stable_cnt, active_touch);
            }

            if (touch < 0) {
                // ADC read failed (mutex timeout), skip this cycle
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            if (touch == last_touch) {
                stable_cnt++;
                if (active_touch < 0 && touch > 0 && stable_cnt >= 5) {
                    // New touch detected
                    active_touch = touch;
                    ESP_LOGI(TAG, "👆 Touch: code=0x%02X", touch);
                    OnTouch(touch);
                } else if (active_touch >= 0 && touch == 0 && stable_cnt >= 8) {
                    // Released
                    active_touch = -1;
                    ESP_LOGI(TAG, "Touch released");
                }
            } else {
                stable_cnt = 1;
                last_touch = touch;
            }

            // 2. MPU-6050: motion detect every 3rd cycle, raw log every 500ms
            static int tick = 0;
            tick++;
            if (mpu_ok_ && tick % 150 == 0) {
                mpu6050_print_raw();  // every ~3s
            }
            if (mpu_ok_ && tick % 3 == 0) {
                mpu6050_motion_t motion = mpu6050_detect_motion();
                if (motion != MOTION_NONE) {
                    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (now - last_motion_ms > MOTION_COOLDOWN_MS) {
                        last_motion_ms = now;
                        ESP_LOGI(TAG, "🌀 Motion: %s | Orient: %s", mpu6050_motion_name(motion),
                                 mpu6050_orient_name(mpu6050_get_orientation()));
                        // Motion → action (when enabled), randomized for variety + sound
                        if (gyro_actions_enabled_ && !demo_mode_) {
                            auto pick = [](const char* const* list, int n) -> const char* {
                                return list[rand() % n];
                            };
                            // Unified audio pool (same as OnTouch)
                            struct ImuAudioEntry { std::string_view sound; float intensity; float duration_ms; };
                            static const ImuAudioEntry kImuAudio[] = {
                                {Lang::Sounds::OGG_CAT_B3,   0.30f, 5140},
                                {Lang::Sounds::OGG_CAT_A6,   0.45f, 6250},
                                {Lang::Sounds::OGG_CAT_A23,  0.55f, 7410},
                                {Lang::Sounds::OGG_CAT_A1,   0.65f, 6460},
                                {Lang::Sounds::OGG_CAT_B13,  0.80f, 5630},
                            };
                            constexpr int kImuAudioN = sizeof(kImuAudio)/sizeof(kImuAudio[0]);
                            auto imuPickAudio = [&](float* int_out, float* dur_out) -> const std::string_view& {
                                int idx = rand() % kImuAudioN;
                                *int_out = kImuAudio[idx].intensity;
                                *dur_out = kImuAudio[idx].duration_ms;
                                return kImuAudio[idx].sound;
                            };
                            // Lookup action steps for duration matching
                            auto imuGetSteps = [&](const char* name) -> int {
                                const ServoAction* a = neck_exec_.FindAction(name);
                                if (a && a->series_count > 0) return a->series[0].total_steps;
                                return 25;
                            };
                            auto imu_rspeed = []() -> uint16_t {
                                int r = rand() % 10;
                                return (r < 5) ? 150 : (r < 8 ? 180 : (r == 8 ? 120 : 0));
                            };
                            // Coordinated dispatch: audio duration → cycles, audio intensity → amplitude
                            auto imu_dispatch = [&](const char* const* acts, int act_n,
                                                     const char* const* necks, int nk_n,
                                                     const char* const* tails, int tl_n,
                                                     const char* const* heads, int hd_n,
                                                     float amp_lo, float amp_hi) {
                                float intensity, dur_ms;
                                auto& snd = imuPickAudio(&intensity, &dur_ms);
                                Application::GetInstance().PlaySound(snd);
                                float amp = amp_lo + intensity * (amp_hi - amp_lo);
                                SetAllAmplitude(amp);
                                uint16_t spd = imu_rspeed();
                                uint16_t step_ms = spd > 0 ? spd : STEP_MS;
                                const char* primary = pick(acts, act_n);
                                int steps = imuGetSteps(primary);
                                int cycles = (int)(dur_ms / ((float)steps * step_ms) + 0.5f);
                                
            cycles = std::max(2, std::min(6, cycles));
                                if (rand() % 10 < 7) {
                                    RunAllAction(primary, spd, cycles);
                                } else {
                                    if (nk_n > 0) neck_exec_.Run(pick(necks, nk_n), spd, cycles);
                                    if (tl_n > 0) tail_exec_.Run(pick(tails, tl_n), spd, cycles);
                                    if (hd_n > 0) head_exec_.Run(pick(heads, hd_n), spd, cycles);
                                }
                            };

                            switch (motion) {
                                case MOTION_PICKED_UP: {
                                    static const char* acts[] = {"all_curious","all_stretch","all_head_tilt","all_alert","all_peek"};
                                    static const char* necks[] = {"neck_nod","neck_left","neck_right","neck_wave","neck_curious","neck_figure8"};
                                    static const char* tails[] = {"tail_up","tail_wag","tail_curl","tail_loop","tail_question"};
                                    static const char* heads[] = {"head_tilt","head_left","head_right","head_nod","head_loop","head_curious"};
                                    imu_dispatch(acts,5, necks,6, tails,5, heads,6, 1.4f, 1.8f);
                                    break;
                                }
                                case MOTION_SHAKEN: {
                                    static const char* acts[] = {"all_dance","all_alert","all_stretch","all_curious","all_angry","all_greeting"};
                                    static const char* necks[] = {"neck_wave","neck_circle","neck_stretch","neck_curious","neck_left","neck_right"};
                                    static const char* tails[] = {"tail_loop","tail_curl","tail_spiral","tail_sway","tail_perk"};
                                    static const char* heads[] = {"head_loop","head_tilt","head_alert","head_dbl_take","head_stretch"};
                                    imu_dispatch(acts,6, necks,6, tails,5, heads,5, 1.5f, 1.8f);
                                    break;
                                }
                                case MOTION_FLIPPED: {
                                    static const char* acts[] = {"all_dance","all_stretch","all_alert","all_curious","all_head_tilt"};
                                    static const char* necks[] = {"neck_wave","neck_circle","neck_nod","neck_figure8","neck_stretch"};
                                    static const char* tails[] = {"tail_curl","tail_loop","tail_spiral","tail_sway","tail_quiver"};
                                    static const char* heads[] = {"head_loop","head_tilt","head_nod","head_stretch","head_curious"};
                                    imu_dispatch(acts,5, necks,5, tails,5, heads,5, 1.5f, 1.8f);
                                    break;
                                }
                                case MOTION_PETTING: {
                                    float intensity, dur_ms;
                                    auto& snd = imuPickAudio(&intensity, &dur_ms);
                                    Application::GetInstance().PlaySound(snd);
                                    float amp = 0.9f + intensity * 0.3f;
                                    SetAllAmplitude(amp);
                                    uint16_t spd = imu_rspeed();
                                    static const char* acts[] = {"all_snuggle","all_love","all_slow_blink","all_head_tilt","all_peek"};
                                    const char* picked = pick(acts,5);
                                    uint16_t step_ms = spd > 0 ? spd : STEP_MS;
                                    int steps = imuGetSteps(picked);
                                    int cycles = (int)(dur_ms / ((float)steps * step_ms) + 0.5f);
                                    
            cycles = std::max(2, std::min(6, cycles));
                                    RunAllAction(picked, spd, cycles);
                                    break;
                                }
                                case MOTION_DROPPED: {
                                    static const char* acts[] = {"all_bow","all_sleep","all_slow_blink","all_stretch"};
                                    static const char* necks[] = {"neck_nod_slow","neck_bow","neck_lean_fwd","neck_relax"};
                                    static const char* tails[] = {"tail_droop","tail_curl","tail_wag_slow","tail_rest"};
                                    static const char* heads[] = {"head_tilt_l","head_left","head_calm"};
                                    imu_dispatch(acts,4, necks,4, tails,4, heads,3, 0.5f, 0.7f);
                                    break;
                                }
                                default: break;
                            }
                        }
                    }
                }
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    void InitializePowerManagement() {
        // Configure POWER_CTRL as output, set HIGH to latch power on
        gpio_config_t pwr_ctrl_cfg = {
            .pin_bit_mask = 1ULL << POWER_CTRL_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&pwr_ctrl_cfg);
        gpio_set_level(POWER_CTRL_GPIO, 1);  // Latch power on immediately
        ESP_LOGI(TAG, "POWER_CTRL (IO%d) set HIGH, power latched", POWER_CTRL_GPIO);

        // POWER_OUT (IO6) — GPIO input with pull-up, then hold to prevent MSPI override
        gpio_config_t pwr_out_cfg = {
            .pin_bit_mask = 1ULL << POWER_OUT_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&pwr_out_cfg);
        gpio_hold_en(POWER_OUT_GPIO);

        xTaskCreate(
            [](void* arg) {
                auto* board = static_cast<CatTailAndNeckBoard*>(arg);
                uint32_t press_count = 0;
                uint32_t release_count = 0;
                const uint32_t poll_interval_ms = 10;
                const uint32_t long_press_ticks = POWER_LONG_PRESS_MS / poll_interval_ms;
                const uint32_t debounce_ticks = 50 / poll_interval_ms;  // 50ms debounce

                // Wait 5s for IO6 to stabilize after power-up
                vTaskDelay(pdMS_TO_TICKS(5000));

                uint32_t tick = 0;
                while (true) {
                    // ADC read IO6, 1V threshold
                    int adc_raw = 0;
                    if (xSemaphoreTake(board->adc_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                        adc_oneshot_read(board->adc_handle_, ADC_CHANNEL_5, &adc_raw);
                        xSemaphoreGive(board->adc_mutex_);
                    }
                    int adc_mv = board->AdcToMv(adc_raw);
                    bool pressed = (adc_mv < 1000);  // active-low: pressed when voltage < 1V
                    tick++;

                    // Every 10s: read battery ADC + log IO6
                    if (tick % 1000 == 0) {
                        ESP_LOGI(TAG, "POWER_OUT (IO6) adc=%dmV pressed=%d", adc_mv, pressed);

                        int bat_raw = 0;
                        if (xSemaphoreTake(board->adc_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                            adc_oneshot_read(board->adc_handle_, BATTERY_ADC_CHANNEL, &bat_raw);
                            xSemaphoreGive(board->adc_mutex_);
                        }
                        int vpin_mv = board->AdcToMv(bat_raw);
                        int vbat_mv = static_cast<int>(vpin_mv * BATTERY_DIVIDER_RATIO);
                        if (board->battery_vbat_filtered_mv_ == 0) {
                            board->battery_vbat_filtered_mv_ = vbat_mv;
                        } else {
                            board->battery_vbat_filtered_mv_ +=
                                (vbat_mv - board->battery_vbat_filtered_mv_) / 5;
                        }
                        int vbat_f = board->battery_vbat_filtered_mv_;
                        int bat_level = (vbat_f - BATTERY_EMPTY_VOLTAGE_MV) * 100 /
                                        (BATTERY_FULL_VOLTAGE_MV - BATTERY_EMPTY_VOLTAGE_MV);
                        board->battery_level_ = std::clamp(bat_level, 0, 100);
                        ESP_LOGI(TAG, "Battery (IO3) raw=%dmV filtered=%dmV level=%d%%", vbat_mv,
                                 vbat_f, board->battery_level_);
                    }

                    if (pressed) {
                        press_count++;
                        release_count = 0;
                        if (press_count >= long_press_ticks) {
                            ESP_LOGW(TAG, "Power button long press detected (%dms), shutting down",
                                     POWER_LONG_PRESS_MS);
                            gpio_set_level(POWER_CTRL_GPIO, 0);
                        }
                    } else {
                        release_count++;
                        if (release_count >= debounce_ticks) {
                            press_count = 0;
                            release_count = debounce_ticks;
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
                }
            },
            "power_monitor", 2048, this, 1, &power_monitor_task_);
    }

    void InitializeCodecI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));

        if (i2c_master_probe(codec_i2c_bus_, AUDIO_CODEC_ES8311_ADDR, pdMS_TO_TICKS(1000)) !=
            ESP_OK) {
            ESP_LOGE(TAG, "ES8311 not found on I2C bus — audio will not work");
        }
        I2cDetect();
    }

    void I2cDetect() {
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                uint8_t address = i + j;
                esp_err_t ret = i2c_master_probe(codec_i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(160, 300);
        power_save_timer_->SetEnabled(true);
    }

    void InitializeBatteryAdc() {
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id = ADC_UNIT_1,
            .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        esp_err_t ret = adc_oneshot_new_unit(&init_cfg, &adc_handle_);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ADC init failed, battery/power monitor disabled");
            return;
        }
        adc_mutex_ = xSemaphoreCreateMutex();

        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };

        // Battery voltage channel (IO3 = ADC1_CH2)
        adc_oneshot_config_channel(adc_handle_, BATTERY_ADC_CHANNEL, &chan_cfg);

        // IO6 power button ADC channel (IO6 → ADC1_CH5)
        adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_5, &chan_cfg);

        // IO5 touch button ADC channel (IO5 → ADC1_CH4, 4-bit R-2R DAC)
        // Disable internal pulls to avoid interfering with R-2R DAC voltage levels
        gpio_config_t io5_cfg = {
            .pin_bit_mask = 1ULL << GPIO_NUM_5,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io5_cfg);
        adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_4, &chan_cfg);

        // ADC calibration via curve fitting (eFuse VREF)
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_1,
            .chan = ADC_CHANNEL_5,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_handle_);
        if (cali_ret == ESP_OK) {
            ESP_LOGI(TAG, "ADC calibration enabled (curve fitting)");
        } else {
            ESP_LOGW(TAG, "ADC calibration skipped (ret=0x%x), using linear", cali_ret);
        }
    }

    int AdcToMv(int raw) const {
        if (adc_cali_handle_) {
            int mv = 0;
            if (adc_cali_raw_to_voltage(adc_cali_handle_, raw, &mv) == ESP_OK)
                return mv;
        }
        return raw * 3300 / 4096;  // fallback: 3.3V, 12-bit
    }

    static void SetServoAngle(int servo_index, int angle) {
        angle = std::clamp(angle, 0, 180);
        if (kServoInvert[servo_index])
            angle = 180 - angle;
        uint32_t duty = AngleToDuty(angle);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, kServoChannels[servo_index], duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, kServoChannels[servo_index]);
    }

    void InitializeServo() {
        // Power on servos via IO4
        gpio_config_t pwr_cfg = {
            .pin_bit_mask = 1ULL << SERVO_POWER_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&pwr_cfg);
        gpio_set_level(SERVO_POWER_GPIO, 1);
        ESP_LOGI(TAG, "Servo power ON (IO%d HIGH)", SERVO_POWER_GPIO);

        ledc_timer_config_t timer_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = SERVO_DUTY_RES,
            .timer_num = SERVO_TIMER,
            .freq_hz = SERVO_FREQ_HZ,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        esp_err_t ret = ledc_timer_config(&timer_cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "LEDC timer init failed, servo disabled");
            return;
        }

        for (int i = 0; i < kServoCount; i++) {
            ledc_channel_config_t ch_cfg = {
                .gpio_num = static_cast<int>(kServoPins[i]),
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = kServoChannels[i],
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = SERVO_TIMER,
                .duty = AngleToDuty(servo_angle_[i]),  // Start at 90°
                .hpoint = 0,
                .flags = {.output_invert = 0},
            };
            ret = ledc_channel_config(&ch_cfg);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Servo IO%d init failed, skipped", kServoPins[i]);
                continue;
            }
            ESP_LOGI(TAG, "Servo %d (%s) initialized at 90°", i, kServoNames[i]);
        }

        // Start all 3 action executors (each drives only its own group)
        neck_exec_.Start(SetServoAngle);
        tail_exec_.Start(SetServoAngle);
        head_exec_.Start(SetServoAngle);
    }

    // Route action name to the correct executor(s) based on prefix
    ActionExecutor* GetExecutorFor(const char* name) {
        const ServoAction* action = neck_exec_.FindAction(name);
        if (!action)
            return nullptr;
        if (action->group_mask == GROUP_NECK)
            return &neck_exec_;
        if (action->group_mask == GROUP_TAIL)
            return &tail_exec_;
        if (action->group_mask == GROUP_HEAD)
            return &head_exec_;
        return nullptr;  // GROUP_ALL handled separately (broadcast)
    }

    // Run a single ALL-group action on all 3 executors
    void RunAllAction(const char* name, uint16_t speed, uint8_t cycles) {
        neck_exec_.Run(name, speed, cycles);
        tail_exec_.Run(name, speed, cycles);
        head_exec_.Run(name, speed, cycles);
    }

    // Set amplitude for all 3 executors in one call (1.0=normal, >1=wider)
    void SetAllAmplitude(float amp) {
        neck_exec_.SetAmplitude(amp);
        tail_exec_.SetAmplitude(amp);
        head_exec_.SetAmplitude(amp);
    }

    void RunOnAllGroups(const char* name, uint16_t speed, uint8_t cycles, bool loop) {
        // For combined (all_*) actions: submit to all 3 executors simultaneously
        neck_exec_.Run(name, speed, cycles, loop);
        tail_exec_.Run(name, speed, cycles, loop);
        head_exec_.Run(name, speed, cycles, loop);
    }

    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        // === Action tools ===

        mcp.AddTool("self.action.run",
                    "Run ONE action by name or id. Call multiple times to combine groups! "
                    "speed: ms/step (0=100=1s/action). cycles: repeat count. loop: true=continuous.\n"
                    "Neck(1001-1023) 脖子: left/right(左右弯) forward(前倾) backward(后仰) nod(点头) "
                    "shake(左右摇) tilt_left/right(左/右歪头) nod_slow(慢点头) wave(波浪) "
                    "stretch(伸懒腰) relax(放松) sway(摇摆) snap_l/r(快速左/右甩) "
                    "lean_fwd(前倾深) bow(深鞠躬) fast_shake(快摇) curious(好奇歪头) "
                    "circle(画圈) glance_lu/rd(左上/右下瞥) figure8(8字).\n"
                    "Tail(2001-2024) 尾巴: wag(摇尾) wag_fast(快摇) wag_slow(慢摇) "
                    "up(上翘) down(下垂) left/right(左/右弯) tremble(颤抖) curl(卷曲) "
                    "loop(循环) swish(优雅甩) flick(轻弹) perk(开心翘) sweep(横扫) "
                    "question(问号) sway(摇摆) bounce(弹跳) droop(垂) hook(钩) "
                    "spiral(盘旋) scurve(S弯) rest(回中) quiver(微颤).\n"
                    "Head(3001-3025) 头: left/right(左/右转) nod(点头) shake(摇头) "
                    "tilt_l/r(左/右歪) tilt(交替歪) glance_l/r(快速瞥) dbl_take(双次确认) "
                    "curious(好奇) loop(循环) sway(摇摆) alert(警觉) bob(轻点) "
                    "confused(困惑) sweep(慢扫) sweep_fast(快扫) calm(平静) relax(放松) "
                    "scan_l/r(慢扫视) micro_l/r(微调) stretch(极限拉伸).\n"
                    "All(4001-4020) 组合: reset(回中) happy(开心) greeting(问候) sleep(睡) "
                    "wake(醒) dance(跳舞) curious(好奇) stretch(伸懒腰) bow(鞠躬) "
                    "snuggle(依偎) scared(害怕) angry(生气) playful(玩耍) love(爱) "
                    "peek(偷看) slow_blink(慢眨眼) head_tilt(歪头杀) pounce(前扑) "
                    "shake_off(抖身体) alert(警觉).",
                    PropertyList({Property("name", kPropertyTypeString, ""),
                                  Property("id", kPropertyTypeInteger, 0),
                                  Property("speed", kPropertyTypeInteger, 0),
                                  Property("cycles", kPropertyTypeInteger, 0),
                                  Property("loop", kPropertyTypeBoolean, false),
                                  Property("amplitude", kPropertyTypeInteger, 0)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        const auto& name = props["name"].value<std::string>();
                        int id = props["id"].value<int>();
                        uint16_t speed = props["speed"].value<int>();
                        uint8_t cycles = props["cycles"].value<int>();
                        bool loop = props["loop"].value<bool>();
                        int amp100 = props["amplitude"].value<int>(); // % e.g. 150 = 1.5x
                        float amplitude = amp100 > 0 ? amp100 / 100.0f : -1.0f;

                        bool ok = (id > 0) ? RunActionById(id, speed, cycles, loop, amplitude > 0.0f ? amplitude : -1.0f)
                                           : RunAction(name.c_str(), speed, cycles, loop, amplitude > 0.0f ? amplitude : -1.0f);

                        char buf[64];
                        snprintf(buf, sizeof(buf), "%s: %s", ok ? "OK" : "FAIL",
                                 id > 0 ? std::to_string(id).c_str() : name.c_str());
                        return std::string(buf);
                    });

        mcp.AddTool("self.action.combo",
                    "Run neck+tail+head actions TOGETHER! Smart combo for natural commands.\n"
                    "Examples: 向左看→neck=neck_left+head=head_left "
                    "开心→neck=neck_nod+tail=tail_wag "
                    "生气→neck=neck_fast_shake+tail=tail_wag_fast+head=head_shake "
                    "撒娇→neck=neck_tilt_left+tail=tail_perk+head=head_tilt_left "
                    "好奇→neck=neck_curious+tail=tail_question+head=head_curious "
                    "跳舞→neck=neck_wave+tail=tail_spiral+head=head_sway "
                    "鞠躬→neck=neck_bow+tail=tail_sweep "
                    "舒服→neck=neck_nod_slow+tail=tail_sway "
                    "警觉→neck=neck_stretch+head=head_alert "
                    "放松→neck=neck_relax+tail=tail_rest+head=head_calm "
                    "被摸→neck=neck_nod_slow+tail=tail_sway 等等任意组合!"
                    "Leave a group empty to skip.",
                    PropertyList({Property("neck", kPropertyTypeString, ""),
                                  Property("tail", kPropertyTypeString, ""),
                                  Property("head", kPropertyTypeString, ""),
                                  Property("speed", kPropertyTypeInteger, 0),
                                  Property("cycles", kPropertyTypeInteger, 0),
                                  Property("loop", kPropertyTypeBoolean, false),
                                  Property("amplitude", kPropertyTypeInteger, 0)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        const auto& neck = props["neck"].value<std::string>();
                        const auto& tail = props["tail"].value<std::string>();
                        const auto& head = props["head"].value<std::string>();
                        uint16_t speed = props["speed"].value<int>();
                        uint8_t cycles = props["cycles"].value<int>();
                        bool loop = props["loop"].value<bool>();
                        float amplitude = props["amplitude"].value<int>() / 100.0f;
                        if (amplitude > 0.0f) SetAllAmplitude(amplitude);
                        int count = 0;

                        if (!neck.empty()) {
                            neck_exec_.Run(neck.c_str(), speed, cycles, loop);
                            count++;
                        }
                        if (!tail.empty()) {
                            tail_exec_.Run(tail.c_str(), speed, cycles, loop);
                            count++;
                        }
                        if (!head.empty()) {
                            head_exec_.Run(head.c_str(), speed, cycles, loop);
                            count++;
                        }

                        if (count == 0) return std::string("FAIL: no group specified");
                        char buf[32];
                        snprintf(buf, sizeof(buf), "OK: %d groups", count);
                        return std::string(buf);
                    });

        mcp.AddTool("self.action.list", "List all action names.", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        std::string names;
                        for (int i = 0; i < kActionCount; i++) {
                            if (i > 0)
                                names += ", ";
                            names += kActionList[i].name;
                        }
                        return names;
                    });

        mcp.AddTool("self.action.stop", "Stop all running actions across all 3 groups.",
                    PropertyList(), [this](const PropertyList&) -> ReturnValue {
                        StopAllActions();
                        return std::string("OK: all stopped");
                    });

        mcp.AddTool("self.action.emote",
                    "Toggle emotion-driven spontaneous movement during chat. on=true/false.",
                    PropertyList({Property("on", kPropertyTypeBoolean, true)}),
                    [this](const PropertyList& props) -> ReturnValue {
                        emote_enabled_ = props["on"].value<bool>();
                        ESP_LOGI(TAG, "Emote mode %s", emote_enabled_ ? "ON" : "OFF");
                        return true;
                    });

        // Battery status
        mcp.AddTool("self.battery.status", "Get battery voltage(mV) and level(%).", PropertyList(),
                    [this](const PropertyList&) -> ReturnValue {
                        char buf[64];
                        snprintf(buf, sizeof(buf), "%dmV %d%%", battery_vbat_filtered_mv_,
                                 battery_level_);
                        ESP_LOGI(TAG, "Battery status: %s", buf);
                        return std::string(buf);
                    });

        // Emotion-driven spontaneous movement — randomly triggers across all 3 groups
        xTaskCreate(
            [](void* arg) {
                auto* board = static_cast<CatTailAndNeckBoard*>(arg);
                vTaskDelay(pdMS_TO_TICKS(10000));  // Wait for system init
                while (true) {
                    if (board->emote_enabled_) {
                        auto state = Application::GetInstance().GetDeviceState();
                        if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
                            if ((rand() % 10) == 0) {  // 10% chance every ~4s
                                int g = rand() % 6;  // 0-2=single, 3-4=dual, 5=triple
                                if (g == 0)
                                    board->neck_exec_.PlayRandomEmote();
                                else if (g == 1)
                                    board->tail_exec_.PlayRandomEmote();
                                else if (g == 2)
                                    board->head_exec_.PlayRandomEmote();
                                else if (g <= 4) {
                                    // Dual combo
                                    board->neck_exec_.PlayRandomEmote();
                                    board->tail_exec_.PlayRandomEmote();
                                } else {
                                    // Triple combo
                                    board->neck_exec_.PlayRandomEmote();
                                    board->tail_exec_.PlayRandomEmote();
                                    board->head_exec_.PlayRandomEmote();
                                }
                                // Occasionally play a tiny meow during chat (30% chance)
                                if ((rand() % 100) < 30) {
                                    static const std::string_view kEmoteSounds[] = {
                                        Lang::Sounds::OGG_CAT_A1, Lang::Sounds::OGG_CAT_A6,
                                        Lang::Sounds::OGG_CAT_A1, Lang::Sounds::OGG_CAT_A23,
                                        Lang::Sounds::OGG_CAT_B3, Lang::Sounds::OGG_CAT_B13,
                                    };
                                    Application::GetInstance().PlaySound(kEmoteSounds[rand() % 6]);
                                }
                            }
                        }
                        vTaskDelay(pdMS_TO_TICKS(4000 + (rand() % 4000)));
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
            },
            "emote_task", 2048, this, 1, nullptr);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

public:
public:




    // ===== WiFi AP + Web Server (runs alongside framework STA) =====
    void InitAP() {
        // Create AP netif (esp_netif already initialized by framework)
        esp_netif_create_default_wifi_ap();
        // Try to set APSTA mode without disrupting existing WiFi
        esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "APSTA mode failed (%s) — AP may not be available", esp_err_to_name(ret));
            return;
        }
        wifi_config_t wcfg = {};
        strcpy((char*)wcfg.ap.ssid, "CatDemo");
        wcfg.ap.ssid_len = strlen("CatDemo");
        strcpy((char*)wcfg.ap.password, "12345678");
        wcfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        wcfg.ap.max_connection = 4;
        ret = esp_wifi_set_config(WIFI_IF_AP, &wcfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "AP config failed: %s", esp_err_to_name(ret));
            return;
        }
        ESP_LOGI(TAG, "WiFi AP ready: CatDemo / 12345678 → http://192.168.4.1");
    }

    static esp_err_t HandleRoot(httpd_req_t* req) {
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_send(req, kDemoHtml, strlen(kDemoHtml));
        return ESP_OK;
    }

    static esp_err_t HandleStatus(httpd_req_t* req) {
        auto* board = static_cast<CatTailAndNeckBoard*>(req->user_ctx);
        // Do an immediate battery read if not yet initialized
        if (board->battery_vbat_filtered_mv_ == 0 && board->adc_handle_) {
            int raw = 0;
            if (xSemaphoreTake(board->adc_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                adc_oneshot_read(board->adc_handle_, BATTERY_ADC_CHANNEL, &raw);
                xSemaphoreGive(board->adc_mutex_);
            }
            int mv = raw * 3300 / 4096;
            if (board->adc_cali_handle_) {
                int cali_mv = 0;
                if (adc_cali_raw_to_voltage(board->adc_cali_handle_, raw, &cali_mv) == ESP_OK)
                    mv = cali_mv;
            }
            int vbat_mv = (int)(mv * BATTERY_DIVIDER_RATIO);
            board->battery_vbat_filtered_mv_ = vbat_mv;
            board->battery_level_ = std::clamp((vbat_mv - BATTERY_EMPTY_VOLTAGE_MV) * 100 / (BATTERY_FULL_VOLTAGE_MV - BATTERY_EMPTY_VOLTAGE_MV), 0, 100);
        }
        char buf[512];
        int level = board->battery_level_;
        int vbat = board->battery_vbat_filtered_mv_;
        bool running = board->neck_exec_.IsBusy() || board->tail_exec_.IsBusy() || board->head_exec_.IsBusy();
        const char* orient = "N/A";
        if (board->mpu_ok_) orient = mpu6050_orient_name(mpu6050_get_orientation());
        snprintf(buf, sizeof(buf),
            "{\"bat_pct\":%d,\"bat_mv\":%d,\"touch_code\":%d,\"touch_mv\":%d,\"gyro_ok\":\"%s\",\"orient\":\"%s\",\"action_running\":%s,\"demo_mode\":%s}",
            level, vbat, board->last_touch_code_, board->last_touch_mv_,
            board->mpu_ok_ ? "OK" : "NONE", orient,
            running ? "true" : "false",
            board->demo_mode_ ? "true" : "false");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, buf);
        return ESP_OK;
    }

    static esp_err_t HandleDemo(httpd_req_t* req) {
        auto* board = static_cast<CatTailAndNeckBoard*>(req->user_ctx);
        char buf[64] = {};
        httpd_req_recv(req, buf, sizeof(buf)-1);
        int id = -1;
        const char* p = strstr(buf, "\"id\":");
        if (p) id = atoi(p + 5);
        if (id >= 0 && id < 5) {
            board->RunDemo(id);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
        } else {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"status\":\"error\",\"msg\":\"invalid id\"}");
        }
        return ESP_OK;
    }

    static esp_err_t HandleServo(httpd_req_t* req) {
        auto* board = static_cast<CatTailAndNeckBoard*>(req->user_ctx);
        char buf[128] = {};
        httpd_req_recv(req, buf, sizeof(buf)-1);
        int angles[5] = {90,90,90,90,90};
        const char* p = strstr(buf, "\"angles\":");
        if (p) {
            p += 9;
            for (int i = 0; i < 5; i++) {
                while (*p == ' ' || *p == '[' || *p == ',') p++;
                angles[i] = atoi(p);
                if (angles[i] < 0) angles[i] = 0;
                if (angles[i] > 180) angles[i] = 180;
                while (*p && *p != ',' && *p != ']') p++;
            }
            for (int i = 0; i < 5; i++) SetServoAngle(i, angles[i]);
        }
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
        return ESP_OK;
    }

    static esp_err_t HandleStop(httpd_req_t* req) {
        auto* board = static_cast<CatTailAndNeckBoard*>(req->user_ctx);
        board->neck_exec_.Stop(); board->tail_exec_.Stop(); board->head_exec_.Stop();
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
        return ESP_OK;
    }

    static esp_err_t HandleMode(httpd_req_t* req) {
        auto* board = static_cast<CatTailAndNeckBoard*>(req->user_ctx);
        char buf[64] = {};
        httpd_req_recv(req, buf, sizeof(buf)-1);
        const char* p = strstr(buf, "\"demo_mode\":");
        if (p) {
            p += 13;
            board->demo_mode_ = (strncmp(p, "true", 4) == 0);
            if (board->demo_mode_) {
                board->neck_exec_.Stop(); board->tail_exec_.Stop(); board->head_exec_.Stop();
            }
            ESP_LOGI(TAG, "Demo mode: %s", board->demo_mode_ ? "ON (phone)" : "OFF (touch/IMU)");
        }
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
        return ESP_OK;
    }

    void StartWebServer() {
        httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
        cfg.max_uri_handlers = 10;
        httpd_start(&web_server_, &cfg);

        auto reg = [this](const char* uri, esp_err_t (*handler)(httpd_req_t*), httpd_method_t method = HTTP_GET) {
            httpd_uri_t u = {.uri = uri, .method = method, .handler = handler, .user_ctx = this};
            httpd_register_uri_handler(web_server_, &u);
        };
        reg("/", HandleRoot);
        reg("/api/status", HandleStatus);
        reg("/api/demo", HandleDemo, HTTP_POST);
        reg("/api/servo", HandleServo, HTTP_POST);
        reg("/api/stop", HandleStop, HTTP_POST);
        reg("/api/mode", HandleMode, HTTP_POST);
        ESP_LOGI(TAG, "Web server started on http://192.168.4.1");
    }

    void UpdateTouchStatus(int code, int mv) {
        last_touch_code_ = code;
        last_touch_mv_ = mv;
    }

    CatTailAndNeckBoard() : WifiBoard(), boot_button_(BOOT_BUTTON_GPIO) {
        esp_log_level_set("Display", ESP_LOG_NONE);
        InitializePowerManagement();  // Must be first: latch power, ADC init
        InitializeCodecI2c();
        InitializeButtons();

        // MPU-6050 (I2C_NUM_1: IO43=SDA, IO44=SCL)
        esp_log_level_set("mpu6050", ESP_LOG_INFO);  // Ensure gyro logs are visible
        mpu_ok_ = mpu6050_init();
        if (mpu_ok_) {
            ESP_LOGI(TAG, "MPU-6050 OK (IO43=SDA, IO44=SCL)");
        } else {
            ESP_LOGW(TAG, "MPU-6050 not found, motion disabled");
        }

        InitializeBatteryAdc();  // Also configures IO5 touch CH4
        InitializeServo();
        InitializePowerSaveTimer();
        InitializeTools();

        // Cat sound playback queue + task (non-blocking for touch task)
        cat_sound_queue_ = xQueueCreate(3, sizeof(cat_sound_type_t));
        xTaskCreate([](void* arg) { static_cast<CatTailAndNeckBoard*>(arg)->CatSoundTask(); },
                    "cat_sound", 4096, this, 3, nullptr);

        // Start touch + motion detection task
        xTaskCreate([](void* arg) { static_cast<CatTailAndNeckBoard*>(arg)->TouchMotionTask(); },
                    "touch_motion", 4096, this, 2, &touch_task_);

        // Boot sound: cute short meow via OGG
        xTaskCreate([](void* arg) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            auto state = Application::GetInstance().GetDeviceState();
            if (state == kDeviceStateStarting || state == kDeviceStateIdle ||
                state == kDeviceStateConnecting) {
                Application::GetInstance().PlaySound(Lang::Sounds::OGG_ANIME_MEW1);
                ESP_LOGI(TAG, "🐱 开机喵~");
            }
            vTaskDelete(nullptr);
        }, "boot_sound", 2048, nullptr, 1, nullptr);
    }

    // ===== Action API (call from anywhere) =====
    bool RunActionById(uint16_t id, uint16_t speed = 0, uint8_t cycles = 1, bool loop = false, float amplitude = -1.0f) {
        const ServoAction* action = neck_exec_.FindActionById(id);
        if (!action) {
            ESP_LOGW(TAG, "Action id=%d not found", id);
            return false;
        }
        if (amplitude >= 0.0f) SetAllAmplitude(amplitude);
        if (action->group_mask == GROUP_ALL)
            RunOnAllGroups(action->name, speed, cycles, loop);
        else {
            ActionExecutor* exec = GetExecutorFor(action->name);
            if (!exec)
                return false;
            exec->Run(action->name, speed, cycles, loop);
        }
        return true;
    }

    bool RunAction(const char* name, uint16_t speed = 0, uint8_t cycles = 1, bool loop = false, float amplitude = -1.0f) {
        const ServoAction* action = neck_exec_.FindAction(name);
        if (!action) {
            ESP_LOGW(TAG, "Action '%s' not found", name);
            return false;
        }
        if (amplitude >= 0.0f) SetAllAmplitude(amplitude);
        if (action->group_mask == GROUP_ALL)
            RunOnAllGroups(name, speed, cycles, loop);
        else {
            ActionExecutor* exec = GetExecutorFor(name);
            if (!exec)
                return false;
            exec->Run(name, speed, cycles, loop);
        }
        return true;
    }

    void StopAllActions() {
        neck_exec_.Stop();
        tail_exec_.Stop();
        head_exec_.Stop();
    }

    // ===== Virtual overrides =====

    virtual void StartNetwork() override {
        WifiBoard::StartNetwork();
        // Start AP + web server after WiFi stack is fully initialized
        xTaskCreate([](void* arg) {
            vTaskDelay(pdMS_TO_TICKS(3000));
            auto* b = static_cast<CatTailAndNeckBoard*>(arg);
            b->InitAP();
            b->StartWebServer();
            vTaskDelete(nullptr);
        }, "web_init", 3584, this, 1, nullptr);
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_GPIO_PA, AUDIO_CODEC_ES8311_ADDR, false);
        return &audio_codec;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        level = battery_level_;
        charging = false;
        discharging = true;
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};


// ===== Web control page =====


DECLARE_BOARD(CatTailAndNeckBoard);
