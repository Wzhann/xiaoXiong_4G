#ifndef _ACTION_WEB_H_
#define _ACTION_WEB_H_

#include <esp_http_server.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "action_executor.h"
#include "action_list.h"
#include "config.h"
#include <driver/gpio.h>

static const char* TAG_AW = "ActionWeb";

// Embedded web page
static const char kActionHtml[] = R"raw(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>舵机控制台</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;padding:10px;max-width:500px;margin:auto}
h1{text-align:center;margin:8px 0;color:#e94560;font-size:18px}
.card{background:#16213e;border-radius:8px;padding:10px;margin-bottom:8px}
h2{font-size:13px;margin-bottom:6px;color:#4ecca3}
.status{padding:4px;text-align:center;font-size:11px;color:#888;min-height:16px}
.row{display:flex;gap:6px}.row>*{flex:1}
.btn{padding:6px 10px;border:none;border-radius:4px;font-size:12px;cursor:pointer;margin:2px;color:#fff}
.btn-run{background:#e94560}.btn-rec{background:#ff6b35}.btn-save{background:#4ecca3;color:#000}
.btn-stop{background:#555}.btn-del{background:#555;font-size:10px;padding:3px 6px}
input,select{width:100%;padding:6px;border:1px solid #333;border-radius:4px;background:#0f3460;color:#eee;font-size:12px;margin-bottom:3px}
input[type=range]{-webkit-appearance:none;width:100%;height:36px;background:linear-gradient(90deg,#0f3460,#e94560);border-radius:4px;outline:none;margin:6px 0}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:32px;height:32px;background:#e94560;border-radius:50%;cursor:pointer}
.slider-label{display:flex;justify-content:space-between;font-size:11px;color:#aaa}
.angle{font-size:22px;font-weight:bold;text-align:center;color:#4ecca3}
table{width:100%;border-collapse:collapse;font-size:11px}
td,th{padding:3px 2px;border-bottom:1px solid #333}
th{color:#aaa;font-size:10px}
</style>
</head>
<body>
<h1>舵机控制台</h1>
<div class="status" id="status">就绪</div>

<div class="card">
<h2>摇杆</h2>
<div class="slider-label"><span>左耳(IO15)</span><span class="angle" id="angleA">90°</span></div>
<input type="range" id="sliderA" min="0" max="180" value="90" oninput="onSlider()">
<div class="slider-label"><span>右耳(IO16)</span><span class="angle" id="angleB">90°</span></div>
<input type="range" id="sliderB" min="0" max="180" value="90" oninput="onSlider()">
<div class="row">
<button class="btn btn-rec" id="btnRec" onclick="toggleRecord()">录制</button>
<button class="btn btn-save" id="btnSave" onclick="saveRecording()" style="display:none">保存</button>
<button class="btn btn-stop" onclick="resetServos()">归中</button>
</div>
</div>

<div class="card">
<h2>动作库</h2>
<input id="addName" placeholder="名称(英文)">
<div class="row">
<select id="addServo"><option value="0">左耳(IO15)</option><option value="1">右耳(IO16)</option><option value="2" selected>双耳</option></select>
</div>
<div class="row"><input id="addStart" placeholder="起" type="number" value="0"><input id="addEnd" placeholder="止" type="number" value="180"></div>
<div class="row"><input id="addMs" placeholder="速度ms" type="number" value="500"><input id="addCycles" placeholder="循环" type="number" value="1"></div>
<label style="font-size:12px;color:#aaa"><input type="checkbox" id="addBf" checked> 来回</label>
<button class="btn btn-save" onclick="addAction()" style="width:100%;margin-top:4px">保存动作</button>
</div>

<div class="card">
<h2>已存动作</h2>
<table><thead><tr><th>名称</th><th>舵机</th><th>起→止</th><th>速度</th><th>操作</th></tr></thead>
<tbody id="actionTable"></tbody></table>
</div>

<script>
const BASE='';
let recording=false,recordStart=0,recordAngleA=90,recordAngleB=90,recordInterval=null;
let lastSend=0;

function setStatus(s,c){document.getElementById('status').textContent=s;if(c)document.getElementById('status').style.color=c;}
async function api(url,method,body){
 try{let o={method};if(body){o.headers={'Content-Type':'application/json'};o.body=JSON.stringify(body);}
 let r=await fetch(BASE+url,o);return await r.text();}catch(e){setStatus('err:'+e,'#e94560');return null;}
}
function onSlider(){
 let a=parseInt(document.getElementById('sliderA').value),b=parseInt(document.getElementById('sliderB').value);
 document.getElementById('angleA').textContent=a+'°';document.getElementById('angleB').textContent=b+'°';
 let now=Date.now();if(now-lastSend>50){lastSend=now;api('/api/servo','POST',{a,b});}
}
async function resetServos(){
 document.getElementById('sliderA').value=90;document.getElementById('sliderB').value=90;
 document.getElementById('angleA').textContent='90°';document.getElementById('angleB').textContent='90°';
 await api('/api/servo','POST',{a:90,b:90});
}

// Record start/end angles
function toggleRecord(){
 if(!recording){
  recording=true;
  recordAngleA=parseInt(document.getElementById('sliderA').value);
  recordAngleB=parseInt(document.getElementById('sliderB').value);
  recordStart=Date.now();
  document.getElementById('btnRec').textContent='停止录制';
  document.getElementById('btnRec').style.background='#e94560';
  document.getElementById('btnSave').style.display='none';
  setStatus('录制中... 移动摇杆，再点停止');
 }else{
  recording=false;
  let endA=parseInt(document.getElementById('sliderA').value);
  let endB=parseInt(document.getElementById('sliderB').value);
  let ms=Date.now()-recordStart;
  document.getElementById('btnRec').textContent='录制';
  document.getElementById('btnRec').style.background='#ff6b35';
  document.getElementById('btnSave').style.display='inline-block';
  // Pre-fill add form with recorded values
  document.getElementById('addStart').value=recordAngleA;
  document.getElementById('addEnd').value=endA;
  document.getElementById('addMs').value=ms>0?ms:500;
  setStatus(`录制: ${recordAngleA}→${endA}, ${ms}ms`);
 }
}

async function saveRecording(){
 let name=prompt('动作名称:');
 if(!name)return;
 let r=await api('/api/actions','POST',{
  name,servo:2,
  start:parseInt(document.getElementById('addStart').value),
  end:parseInt(document.getElementById('addEnd').value),
  ms:parseInt(document.getElementById('addMs').value),
  cycles:parseInt(document.getElementById('addCycles').value)||1,
  flags:document.getElementById('addBf').checked?1:0
 });
 document.getElementById('btnSave').style.display='none';
 loadActions();
}

async function addAction(){
 let name=document.getElementById('addName').value;
 if(!name){setStatus('输入名称','#e94560');return;}
 let r=await api('/api/actions','POST',{
  name,servo:parseInt(document.getElementById('addServo').value),
  start:parseInt(document.getElementById('addStart').value),
  end:parseInt(document.getElementById('addEnd').value),
  ms:parseInt(document.getElementById('addMs').value),
  cycles:parseInt(document.getElementById('addCycles').value)||1,
  flags:document.getElementById('addBf').checked?1:0
 });
 loadActions();
}

async function loadActions(){
 let t=await api('/api/actions','GET');if(!t)return;
 try{let actions=JSON.parse(t),tb=document.getElementById('actionTable');
 tb.innerHTML='';
 actions.forEach(a=>{let sv=a.servo==0?'左':a.servo==1?'右':'双';
 tb.innerHTML+=`<tr><td>${a.name}</td><td>${sv}</td><td>${a.start}→${a.end}</td><td>${a.ms}ms</td>
 <td><button class="btn btn-run" onclick="runAction('${a.name}')">跑</button>
 <button class="btn btn-del" onclick="delAction('${a.name}')">删</button></td></tr>`;
 });
 setStatus(`${actions.length} 个动作`);
 }catch(e){setStatus('parse err','#e94560');}
}
async function delAction(name){if(confirm('删除 '+name+'?')){await api('/api/actions/'+name,'DELETE');loadActions();}}
async function runAction(n){let r=await api('/api/run','POST',{name:n});setStatus(r);}
async function stopAction(){await api('/api/stop','POST');}
loadActions();
</script>
</body>
</html>
)raw";

// --- NVS helpers for custom actions ---
#define NVS_NAMESPACE "actions"
#define NVS_KEY_LIST  "list"

static std::string NvsGetStr(const char* key) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return "";
    size_t len = 0;
    nvs_get_str(h, key, nullptr, &len);
    if (len == 0) { nvs_close(h); return ""; }
    char* buf = (char*)malloc(len);
    nvs_get_str(h, key, buf, &len);
    nvs_close(h);
    std::string s(buf);
    free(buf);
    return s;
}

static void NvsSetStr(const char* key, const char* val) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, key, val);
    nvs_commit(h);
    nvs_close(h);
}

// Build JSON array of all actions (predefined + NVS custom)
static std::string BuildActionJson() {
    std::string json = "[";
    // Predefined actions
    for (int i = 0; i < kActionCount; i++) {
        if (i > 0) json += ",";
        char buf[256];
        snprintf(buf, sizeof(buf),
                 R"({"name":"%s","servo":%d,"start":%d,"end":%d,"ms":%d,"cycles":%d,"flags":%d,"custom":false})",
                 kActionList[i].name, kActionList[i].servo,
                 kActionList[i].start_angle, kActionList[i].end_angle,
                 kActionList[i].default_ms, kActionList[i].default_cycles,
                 kActionList[i].flags);
        json += buf;
    }
    // Custom actions from NVS
    std::string custom = NvsGetStr(NVS_KEY_LIST);
    if (!custom.empty() && custom != "[]") {
        // custom is a JSON array string, strip the brackets and append
        if (custom[0] == '[' && custom[custom.size()-1] == ']') {
            std::string inner = custom.substr(1, custom.size() - 2);
            if (!inner.empty()) {
                if (!json.empty() && json != "[") json += ",";
                json += inner;
            }
        }
    }
    json += "]";
    return json;
}

// Add a custom action (stores in NVS as appended JSON array)
static std::string AddCustomAction(const char* json_body) {
    std::string existing = NvsGetStr(NVS_KEY_LIST);
    std::string new_list = "[";
    if (!existing.empty() && existing != "[]") {
        new_list += existing.substr(1, existing.size() - 2);
        new_list += ",";
    }
    new_list += json_body;
    new_list += "]";
    NvsSetStr(NVS_KEY_LIST, new_list.c_str());
    return "OK: added";
}

// Delete a custom action by name
static std::string DelCustomAction(const char* name) {
    std::string existing = NvsGetStr(NVS_KEY_LIST);
    if (existing.empty()) return "FAIL: not found";

    // Simple approach: rebuild the list without the named action
    // JSON format: [{"name":"xxx",...},...]
    std::string new_list = "[";
    bool first = true;
    const char* p = existing.c_str();
    while (*p) {
        if (*p == '{') {
            const char* end = strchr(p, '}');
            if (!end) break;
            std::string entry(p, end - p + 1);
            // Check if this entry contains "name":"<name>"
            char search[128];
            snprintf(search, sizeof(search), "\"name\":\"%s\"", name);
            if (strstr(entry.c_str(), search) == nullptr) {
                if (!first) new_list += ",";
                new_list += entry;
                first = false;
            }
            p = end + 1;
        } else {
            p++;
        }
    }
    new_list += "]";
    NvsSetStr(NVS_KEY_LIST, new_list.c_str());
    return "OK: deleted";
}

// --- HTTP request handlers ---
static ActionExecutor* g_action_executor = nullptr;

static esp_err_t HandleGetHtml(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, kActionHtml, strlen(kActionHtml));
    return ESP_OK;
}

static esp_err_t HandleGetActions(httpd_req_t* req) {
    std::string json = BuildActionJson();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json.c_str(), json.size());
    return ESP_OK;
}

static esp_err_t HandlePostAction(httpd_req_t* req) {
    char buf[512] = {};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
    buf[ret] = 0;

    // Add custom=true marker
    // Parse the JSON, add "custom":true
    std::string json(buf);
    // Insert "custom":true before closing }
    size_t pos = json.rfind('}');
    if (pos != std::string::npos) {
        json.insert(pos, ",\"custom\":true");
    }
    std::string result = AddCustomAction(json.c_str());
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, result.c_str(), result.size());
    return ESP_OK;
}

static esp_err_t HandleDeleteAction(httpd_req_t* req) {
    char name[64] = {};
    char qbuf[256];
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        char val[64];
        if (httpd_query_key_value(qbuf, "name", val, sizeof(val)) == ESP_OK) {
            strncpy(name, val, sizeof(name) - 1);
        }
    }
    if (name[0] == 0) {
        // Try parsing from URL path: /api/actions/NAME
        // Simple: just use the path
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    std::string result = DelCustomAction(name);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, result.c_str(), result.size());
    return ESP_OK;
}

static esp_err_t HandleRun(httpd_req_t* req) {
    char buf[256] = {};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
    buf[ret] = 0;

    // Parse {"name":"xxx","speed":500,"cycles":3}
    char name[64] = {};
    int speed = 0, cycles = 0;
    // Simple JSON parsing
    const char* np = strstr(buf, "\"name\":\"");
    if (np) {
        np += 8;
        int i = 0;
        while (*np && *np != '"' && i < 63) name[i++] = *np++;
        name[i] = 0;
    }
    const char* sp = strstr(buf, "\"speed\":");
    if (sp) speed = atoi(sp + 8);
    const char* cp = strstr(buf, "\"cycles\":");
    if (cp) cycles = atoi(cp + 9);

    if (name[0] == 0) {
        httpd_resp_sendstr(req, "FAIL: name required");
        return ESP_FAIL;
    }

    if (g_action_executor && g_action_executor->Run(name, speed, cycles)) {
        char result[128];
        snprintf(result, sizeof(result), "OK: %s running (speed=%dms, cycles=%d)", name, speed, cycles);
        httpd_resp_sendstr(req, result);
    } else {
        httpd_resp_sendstr(req, "FAIL: action not found");
    }
    return ESP_OK;
}

static esp_err_t HandleStop(httpd_req_t* req) {
    if (g_action_executor) g_action_executor->Stop();
    httpd_resp_sendstr(req, "OK: stopped");
    return ESP_OK;
}

// Real-time servo control from web sliders
static esp_err_t HandleServo(httpd_req_t* req) {
    char buf[128] = {};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
    buf[ret] = 0;
    int a = -1, b = -1;
    const char* ap = strstr(buf, "\"a\":");
    if (ap && strncmp(ap + 4, "null", 4) != 0) { a = atoi(ap + 4); if (a < 0) a = 0; if (a > 180) a = 180; }
    const char* bp = strstr(buf, "\"b\":");
    if (bp && strncmp(bp + 4, "null", 4) != 0) { b = atoi(bp + 4); if (b < 0) b = 0; if (b > 180) b = 180; }
    gpio_set_level(SERVO_POWER_GPIO, 1);
    if (g_action_executor) {
        if (a >= 0) g_action_executor->SetBothServos(a, b >= 0 ? b : a);  // a only
        else if (b >= 0) g_action_executor->SetBothServos(90, b);          // b only (keep A at 90)
    }
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// --- Register routes on an existing HTTP server ---
static void RegisterActionRoutes(httpd_handle_t server, ActionExecutor* executor) {
    g_action_executor = executor;

    // GET /action → HTML page
    httpd_uri_t uri_html = {.uri = "/action", .method = HTTP_GET, .handler = HandleGetHtml, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_html);

    // GET /api/actions → list all actions
    httpd_uri_t uri_list = {.uri = "/api/actions", .method = HTTP_GET, .handler = HandleGetActions, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_list);

    // POST /api/actions → add custom action
    httpd_uri_t uri_add = {.uri = "/api/actions", .method = HTTP_POST, .handler = HandlePostAction, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_add);

    // DELETE /api/actions?name=xxx → delete custom action
    httpd_uri_t uri_del = {.uri = "/api/actions", .method = HTTP_DELETE, .handler = HandleDeleteAction, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_del);

    // POST /api/run → run an action
    httpd_uri_t uri_run = {.uri = "/api/run", .method = HTTP_POST, .handler = HandleRun, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_run);

    // POST /api/stop → stop actions
    httpd_uri_t uri_stop = {.uri = "/api/stop", .method = HTTP_POST, .handler = HandleStop, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_stop);

    // POST /api/servo → real-time servo control {a:0-180, b:0-180}
    httpd_uri_t uri_servo = {.uri = "/api/servo", .method = HTTP_POST, .handler = HandleServo, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_servo);

    ESP_LOGI(TAG_AW, "Action routes registered");
}


#endif  // _ACTION_WEB_H_
