// ============================================
// webinterface/webserver.h
// HTTP Web Server - Dashboard at 192.168.1.10
// ============================================

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <WebServer.h>
#include "../config/config.h"
#include "../config/pins_config.h"
#include "../config/thread_safety.h"

struct WebServerState {
    bool running = false;
    unsigned int requestsHandled = 0;
};

static WebServerState webServerState;
static WebServer webServer(80);

const char INDEX_HTML[] PROGMEM = R"(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AOG Unified ESP32-S3</title><style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: Arial, sans-serif; background: #1a1a1a; color: #fff; }
.container { max-width: 1200px; margin: 0 auto; padding: 20px; }
.header { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 30px; border-radius: 10px; margin-bottom: 20px; }
.header h1 { font-size: 2.5em; margin-bottom: 10px; }
.grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin-bottom: 20px; }
.card { background: #2a2a2a; border-left: 4px solid #667eea; padding: 20px; border-radius: 8px; }
.card h2 { font-size: 1.2em; margin-bottom: 15px; color: #667eea; }
.status-row { display: flex; justify-content: space-between; margin-bottom: 10px; }
.online { color: #4caf50; }
.offline { color: #f44336; }
.sections { display: grid; grid-template-columns: repeat(8, 1fr); gap: 10px; margin-top: 15px; }
.section-btn { padding: 10px; background: #333; border: 2px solid #666; border-radius: 5px; cursor: pointer; text-align: center; transition: 0.3s; }
.section-btn.active { background: #4caf50; border-color: #4caf50; }
.section-btn:hover { border-color: #667eea; }
button { padding: 10px 20px; background: #667eea; color: white; border: none; border-radius: 5px; cursor: pointer; margin: 5px; }
button:hover { background: #764ba2; }
.info-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
.info-item { background: #333; padding: 10px; border-radius: 5px; }
.info-label { font-size: 0.9em; opacity: 0.7; }
.info-value { font-size: 1.3em; color: #667eea; font-weight: bold; }
</style></head><body>
<div class="container"><div class="header"><h1>🚜 AOG Unified ESP32-S3</h1><p>Kontroler do AgOpenGPS</p></div>
<div class="grid">
<div class="card"><h2>🌐 Połączenia</h2><div class="status-row"><span>Ethernet:</span><span id="eth-status" class="online">✓ Online</span></div><div class="status-row"><span>IP:</span><span id="eth-ip">192.168.1.10</span></div></div>
<div class="card"><h2>📍 GPS</h2><div class="info-grid"><div class="info-item"><div class="info-label">Fix</div><div class="info-value" id="gps-fix">RTK</div></div><div class="info-item"><div class="info-label">Sats</div><div class="info-value" id="gps-sats">12</div></div></div></div>
<div class="card"><h2>🎯 IMU</h2><div class="info-grid"><div class="info-item"><div class="info-label">Heading</div><div class="info-value" id="imu-heading">45.3°</div></div><div class="info-item"><div class="info-label">Roll</div><div class="info-value" id="imu-roll">-2.1°</div></div></div></div>
<div class="card"><h2>🎛️ Autosteer</h2><div class="status-row"><span>Status:</span><span id="autosteer-status" class="offline">INACTIVE</span></div><div class="info-grid"><div class="info-item"><div class="info-label">Error</div><div class="info-value" id="autosteer-error">0.0°</div></div><div class="info-item"><div class="info-label">PWM</div><div class="info-value" id="autosteer-pwm">0</div></div></div></div>
<div class="card" style="grid-column: 1 / -1;"><h2>📋 Sekcje (16x)</h2><div class="sections" id="sections-grid"></div><button onclick="allOn()">Wszystkie ON</button><button onclick="allOff()">Wszystkie OFF</button></div>
</div></div>
<script>
setInterval(updateStatus, 1000);
function updateStatus() {
  fetch('/status.json').then(r => r.json()).then(data => {
    document.getElementById('eth-status').textContent = data.eth ? '✓ Online' : '✗ Offline';
    document.getElementById('eth-status').className = data.eth ? 'online' : 'offline';
    document.getElementById('gps-sats').textContent = data.sats || '0';
    document.getElementById('imu-heading').textContent = (data.heading || 0).toFixed(1) + '°';
    document.getElementById('imu-roll').textContent = (data.roll || 0).toFixed(1) + '°';
    document.getElementById('autosteer-error').textContent = (data.error || 0).toFixed(1) + '°';
    document.getElementById('autosteer-pwm').textContent = data.pwm || '0';
    updateSections(data.sections || 0);
  });
}
function updateSections(state) {
  const grid = document.getElementById('sections-grid');
  if (grid.children.length === 0) {
    for (let i = 0; i < 16; i++) {
      const btn = document.createElement('button');
      btn.className = 'section-btn';
      btn.textContent = (i + 1);
      btn.id = 's' + i;
      btn.onclick = () => toggleSection(i);
      grid.appendChild(btn);
    }
  }
  for (let i = 0; i < 16; i++) {
    const btn = document.getElementById('s' + i);
    if (state & (1 << i)) btn.classList.add('active');
    else btn.classList.remove('active');
  }
}
function toggleSection(num) { fetch('/api/section?n=' + num); }
function allOn() { fetch('/api/all?state=1'); }
function allOff() { fetch('/api/all?state=0'); }
updateStatus();
</script></body></html>
)";

void handle_root() {
    webServer.send(200, "text/html", INDEX_HTML);
    webServerState.requestsHandled++;
}

void handle_status_json() {
    float heading = 0.0f;
    float roll = 0.0f;
    float error = 0.0f;
    byte pwmValue = 0;
    uint16_t sections = 0;
    bool eth = false;

    if (lock_shared_data()) {
        heading = imu1_data.heading;
        roll = imu1_data.roll;
        error = autosteer.heading_error;
        pwmValue = autosteer.pwmValue;
        sections = sectionState.currentState;
        eth = ethConnected;
        unlock_shared_data();
    }

    String json = "{";
    json += "\"eth\":" + String(eth ? 1 : 0) + ",";
    json += "\"sats\":12,";
    json += "\"heading\":" + String(heading) + ",";
    json += "\"roll\":" + String(roll) + ",";
    json += "\"error\":" + String(error) + ",";
    json += "\"pwm\":" + String(pwmValue) + ",";
    json += "\"sections\":" + String(sections);
    json += "}";
    webServer.send(200, "application/json", json);
}

void handle_section_toggle() {
    if (webServer.hasArg("n")) {
        int n = webServer.arg("n").toInt();
        if (n >= 0 && n < 16) {
            section_toggle(n);
            webServer.send(200, "application/json", "{\"ok\":1}");
            return;
        }
    }
    webServer.send(400, "application/json", "{\"err\":1}");
}

void handle_all_sections() {
    if (webServer.hasArg("state")) {
        int state = webServer.arg("state").toInt();
        section_control_set_state(state ? 0xFFFF : 0x0000);
        webServer.send(200, "application/json", "{\"ok\":1}");
        return;
    }
    webServer.send(400, "application/json", "{\"err\":1}");
}

bool webserver_init() {
    Serial.println("[WEB] Starting HTTP server...");
    webServer.on("/", handle_root);
    webServer.on("/status.json", handle_status_json);
    webServer.on("/api/section", handle_section_toggle);
    webServer.on("/api/all", handle_all_sections);
    webServer.begin();
    webServerState.running = true;
    Serial.println("[WEB] ✓ Server running at http://192.168.1.10");
    return true;
}

void webserver_handle_clients() {
    if (webServerState.running) {
        webServer.handleClient();
    }
}

#endif // WEBSERVER_H
