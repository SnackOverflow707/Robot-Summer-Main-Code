#include "armWifiManager.h"
#include "ArmController2.h"
#include "taskManager.h"
#include "armTasks/solarPanels.h"

ArmWifiManager::ArmWifiManager(
    const char* ssid,
    const char* password,
    ArmController2& arm
)
    : _ssid(ssid),
      _password(password),
      _server(80),
      _arm(arm),
      _enabled(false)
{}

void ArmWifiManager::begin()  { WiFi.mode(WIFI_OFF); setupRoutes(); }
void ArmWifiManager::update() { if (_enabled) _server.handleClient(); }

bool ArmWifiManager::enable()
{
    if (_enabled) return true;
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP("Team-10-Arm", "arm12345")) {
        WiFi.mode(WIFI_OFF);
        return false;
    }
    _server.begin();
    _enabled = true;
    return true;
}

void ArmWifiManager::disable()
{
    if (!_enabled) return;
    _arm.goHome();
    _server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    _enabled = false;
}

bool ArmWifiManager::isEnabled() const { return _enabled; }

void ArmWifiManager::setupRoutes()
{
    // Serve control page
    _server.on("/", HTTP_GET, [this]() { showControlPage(); });

    // GET /status → JSON of current angles
    _server.on("/status", HTTP_GET, [this]()
    {
        String json = "{";
        json += "\"base\":"     + String(_arm.getBase())     + ",";
        json += "\"shoulder\":" + String(_arm.getShoulder()) + ",";
        json += "\"elbow\":"    + String(_arm.getElbow())    + ",";
        json += "\"wrist\":"    + String(_arm.getWrist())    + ",";
        json += "\"claw\":"     + String(_arm.getClaw());
        json += "}";
        _server.send(200, "application/json", json);
    });

    // GET /set?joint=base&angle=90 → move joint to angle
    _server.on("/set", HTTP_GET, [this]()
    {
        if (!_server.hasArg("joint") || !_server.hasArg("angle")) {
            _server.send(400, "text/plain", "Missing joint or angle.");
            return;
        }
        const String joint = _server.arg("joint");
        const int    angle = _server.arg("angle").toInt();
        if      (joint == "base")     _arm.setBase(angle);
        else if (joint == "shoulder") _arm.setShoulder(angle);
        else if (joint == "elbow")    _arm.setElbow(angle);
        else if (joint == "wrist")    _arm.setWrist(angle);
        else if (joint == "claw")     _arm.setClaw(angle);
        else { _server.send(400, "text/plain", "Unknown joint: " + joint); return; }
        _server.send(200, "text/plain", "OK");
    });

    // GET /jog?joint=elbow&delta=5 → increment joint by delta degrees
    _server.on("/jog", HTTP_GET, [this]()
    {
        if (!_server.hasArg("joint") || !_server.hasArg("delta")) {
            _server.send(400, "text/plain", "Missing joint or delta.");
            return;
        }
        const String joint = _server.arg("joint");
        const int    delta = _server.arg("delta").toInt();

        // Read current, add delta, re-send
        int current = 0;
        if      (joint == "base")     current = _arm.getBase();
        else if (joint == "shoulder") current = _arm.getShoulder();
        else if (joint == "elbow")    current = _arm.getElbow();
        else if (joint == "wrist")    current = _arm.getWrist();
        else if (joint == "claw")     current = _arm.getClaw();
        else { _server.send(400, "text/plain", "Unknown joint: " + joint); return; }

        const int target = current + delta;
        if      (joint == "base")     _arm.setBase(target);
        else if (joint == "shoulder") _arm.setShoulder(target);
        else if (joint == "elbow")    _arm.setElbow(target);
        else if (joint == "wrist")    _arm.setWrist(target);
        else if (joint == "claw")     _arm.setClaw(target);

        _server.send(200, "text/plain", "OK");
    });

    // GET /home → home all joints
    _server.on("/home", HTTP_GET, [this]()
    {
        _arm.goHome();
        _server.send(200, "text/plain", "OK");
    });

    // GET /sequence?name=solarPanel → run a named movement sequence
    _server.on("/sequence", HTTP_GET, [this]()
    {
        if (!_server.hasArg("name")) {
            _server.send(400, "text/plain", "Missing name."); return;
        }
        const String name = _server.arg("name");
        if (name == "solarPanel") {
            TaskManager tm(_arm);
            solarPanelSequence(tm);
            _server.send(200, "text/plain", "OK");
        } else {
            _server.send(400, "text/plain", "Unknown sequence: " + name);
        }
    });

    // GET /print → print current angles to Serial (for waypoint capture)
    _server.on("/print", HTTP_GET, [this]()
    {
        Serial.println("=== WAYPOINT CAPTURE ===");
        Serial.printf("{ %d, %d, %d, %d, %d }\n",
            _arm.getBase(), _arm.getShoulder(),
            _arm.getElbow(), _arm.getWrist(), _arm.getClaw());
        _server.send(200, "text/plain", "Printed to Serial.");
    });
}

void ArmWifiManager::showControlPage()
{
    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>Robot Arm</title>
  <style>
    :root {
      --bg:#f4f7fb; --panel:#fff; --accent:#2f6fed;
      --text:#1f2937; --muted:#6b7280; --border:#dbe3ee;
      --red:#dc2626; --green:#16a34a;
    }
    *{box-sizing:border-box;margin:0;padding:0;}
    body{font-family:Arial,sans-serif;background:var(--bg);color:var(--text);padding:20px;}
    h1{text-align:center;margin-bottom:20px;font-size:24px;}
    h2{font-size:14px;letter-spacing:.08em;text-transform:uppercase;
       color:var(--muted);margin-bottom:12px;}
    .panel{background:var(--panel);border:1px solid var(--border);
           border-radius:12px;padding:18px 20px;
           box-shadow:0 2px 10px rgba(0,0,0,.05);margin-bottom:16px;}
    .row{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:8px;}
    input[type=text],input[type=number]{
      font-size:13px;padding:6px 8px;border-radius:6px;
      border:1px solid var(--border);width:80px;text-align:center;}
    button{font-size:13px;padding:6px 12px;border-radius:6px;border:none;
           background:var(--accent);color:#fff;cursor:pointer;}
    button:hover{opacity:.9;}
    button.sm{padding:4px 10px;font-size:12px;}
    button.red{background:var(--red);}
    button.green{background:var(--green);}
    .status{font-size:12px;color:var(--muted);margin-top:6px;}

    /* joint table */
    .jtable{width:100%;border-collapse:collapse;}
    .jtable th{font-size:11px;letter-spacing:.06em;text-transform:uppercase;
               color:var(--muted);padding:4px 8px;text-align:center;}
    .jtable td{padding:6px 8px;text-align:center;vertical-align:middle;}
    .jtable tr:nth-child(even){background:#f8fafc;}
    .current{font-weight:700;font-size:15px;min-width:48px;display:inline-block;}
    .jog-cell{display:flex;align-items:center;justify-content:center;gap:4px;}

    /* tabs */
    .tabs{display:flex;gap:6px;margin-bottom:12px;}
    .tab{padding:6px 14px;border-radius:6px;border:1px solid var(--border);
         background:#fff;cursor:pointer;font-size:13px;color:var(--muted);}
    .tab.active{background:var(--accent);color:#fff;border-color:var(--accent);}
    .tab-content{display:none;}
    .tab-content.active{display:block;}

    /* waypoint log */
    #wp-log{background:#1e293b;color:#7dd3fc;font-family:'Courier New',monospace;
            font-size:12px;padding:10px;border-radius:6px;min-height:80px;
            white-space:pre;overflow-x:auto;}
  </style>
</head>
<body>
<h1>Robot Arm Control</h1>

<!-- CONNECTION -->
<div class="panel">
  <div class="row">
    <input type="text" id="ipInput" value="192.168.4.1" style="width:130px"/>
    <button onclick="toggleConnect()" id="connBtn">Connect</button>
    <span class="status" id="connStatus">Disconnected</span>
  </div>
</div>

<!-- TABS -->
<div class="tabs">
  <div class="tab active" onclick="showTab('control')">Control</div>
  <div class="tab" onclick="showTab('jog')">Jog</div>
  <div class="tab" onclick="showTab('waypoints')">Waypoints</div>
</div>

<!-- TAB: CONTROL (set absolute angles) -->
<div class="tab-content active" id="tab-control">
  <div class="panel">
    <h2>Set Joint Angles</h2>
    <table class="jtable">
      <tr>
        <th>Joint</th><th>Current</th><th>Target</th><th></th>
      </tr>
)rawliteral";

    // Joint rows — control tab
    const char* joints[]   = {"Base","Shoulder","Elbow","Wrist","Claw"};
    const int   homeAngles[] = {HOME_BASE, HOME_SHOULDER, HOME_ELBOW, HOME_WRIST, HOME_CLAW};
    const int   maxAngles[]  = {MAX_ANGLE, MAX_ANGLE, MAX_ANGLE, MAX_ANGLE, MAX_ANGLE_CLAW};

    for (int i = 0; i < 5; i++) {
        html += "<tr><td><b>";
        html += joints[i];
        html += "</b></td><td><span class='current' id='cur-";
        html += joints[i];
        html += "'>—</span>°</td><td><input type='number' id='tgt-";
        html += joints[i];
        html += "' value='";
        html += String(homeAngles[i]);
        html += "' min='0' max='";
        html += String(maxAngles[i]);
        html += "'/></td><td><button class='sm' onclick=\"moveJoint('";
        html += joints[i];
        html += "')\">Move</button></td></tr>";
    }

    html += R"rawliteral(
    </table>
    <div class="row" style="margin-top:12px;">
      <button class="green" onclick="homeAll()">&#8962; Home All</button>
    </div>
  </div>

  <div class="panel">
    <h2>Sequences</h2>
    <p style="font-size:13px;color:var(--muted);margin-bottom:12px;">
      Runs a pre-programmed movement sequence. The page will be unresponsive until the sequence finishes.
    </p>
    <div class="row">
      <button onclick="runSequence('solarPanel')" id="seq-btn-solarPanel">Solar Panels</button>
      <span class="status" id="seq-status"></span>
    </div>
  </div>
</div>

<!-- TAB: JOG (increment/decrement by step) -->
<div class="tab-content" id="tab-jog">
  <div class="panel">
    <h2>Jog Joints</h2>
    <div class="row" style="margin-bottom:12px;">
      <span style="font-size:13px;">Step size:</span>
      <button class="sm" onclick="setStep(1)">1°</button>
      <button class="sm" onclick="setStep(5)">5°</button>
      <button class="sm" onclick="setStep(10)">10°</button>
      <input type="number" id="jogStep" value="5" min="1" max="45" style="width:60px"/>
    </div>
    <table class="jtable">
      <tr><th>Joint</th><th>Current</th><th>Jog</th></tr>
)rawliteral";

    for (int i = 0; i < 5; i++) {
        html += "<tr><td><b>";
        html += joints[i];
        html += "</b></td><td><span class='current' id='jog-cur-";
        html += joints[i];
        html += "'>—</span>°</td><td><div class='jog-cell'>";
        html += "<button class='sm red' onclick=\"jog('";
        html += joints[i];
        html += "',-1)\">−</button>";
        html += "<button class='sm green' onclick=\"jog('";
        html += joints[i];
        html += "',+1)\">+</button>";
        html += "</div></td></tr>";
    }

    html += R"rawliteral(
    </table>
  </div>
</div>

<!-- TAB: WAYPOINTS (capture & log) -->
<div class="tab-content" id="tab-waypoints">
  <div class="panel">
    <h2>Waypoint Capture</h2>
    <p style="font-size:13px;color:var(--muted);margin-bottom:12px;">
      Jog the arm to a position, name it, and capture.
      The struct is printed to Serial and logged below.
    </p>
    <div class="row">
      <input type="text" id="wpName" placeholder="e.g. PICK_APPROACH" style="width:200px"/>
      <button class="green" onclick="captureWaypoint()">📌 Capture</button>
      <button onclick="clearLog()">Clear</button>
    </div>
    <div style="margin-top:12px;">
      <div id="wp-log">// waypoints will appear here</div>
    </div>
  </div>
</div>

<script>
  const JOINTS = ["Base","Shoulder","Elbow","Wrist","Claw"];
)rawliteral";

    html += "const HOME_ANGLES = [" +
            String(HOME_BASE)     + "," +
            String(HOME_SHOULDER) + "," +
            String(HOME_ELBOW)    + "," +
            String(HOME_WRIST)    + "," +
            String(HOME_CLAW)     + "];\n";

    html += R"rawliteral(

  let connected = false;
  let baseUrl = "";
  let pollTimer = null;

  // ── Connection ──────────────────────────────────────────
  async function toggleConnect() {
    if (connected) {
      clearInterval(pollTimer);
      connected = false;
      document.getElementById("connBtn").textContent = "Connect";
      document.getElementById("connStatus").textContent = "Disconnected";
      return;
    }
    const ip = document.getElementById("ipInput").value.trim();
    baseUrl = "http://" + ip;
    document.getElementById("connStatus").textContent = "Connecting…";
    try {
      const r = await fetch(baseUrl + "/status", {cache:"no-store"});
      if (!r.ok) throw new Error();
      connected = true;
      document.getElementById("connBtn").textContent = "Disconnect";
      document.getElementById("connStatus").textContent = "Connected to " + ip;
      await refreshStatus();
      pollTimer = setInterval(refreshStatus, 500);  // auto-poll every 500ms
    } catch {
      document.getElementById("connStatus").textContent = "Could not reach ESP32.";
    }
  }

  window.addEventListener("load", toggleConnect);

  // ── Status polling ──────────────────────────────────────
  async function refreshStatus() {
    if (!connected) return;
    try {
      const r = await fetch(baseUrl + "/status", {cache:"no-store"});
      if (!r.ok) return;
      const d = await r.json();
      JOINTS.forEach(j => {
        const key = j.toLowerCase();
        const val = d[key] !== undefined ? d[key] : "—";
        const el1 = document.getElementById("cur-" + j);
        const el2 = document.getElementById("jog-cur-" + j);
        if (el1) el1.textContent = val;
        if (el2) el2.textContent = val;
      });
    } catch {}
  }

  // ── Commands ────────────────────────────────────────────
  async function cmd(path) {
    if (!connected) { alert("Connect first."); return false; }
    try {
      const r = await fetch(baseUrl + path);
      return r.ok;
    } catch { return false; }
  }

  async function moveJoint(joint) {
    const input = document.getElementById("tgt-" + joint);
    const angle = Math.round(Number(input.value));
    input.value = angle;
    await cmd("/set?joint=" + joint.toLowerCase() + "&angle=" + angle);
    await refreshStatus();
  }

  async function homeAll() {
    await cmd("/home");
    JOINTS.forEach((j,i) => {
      const el = document.getElementById("tgt-" + j);
      if (el) el.value = HOME_ANGLES[i];
    });
    await refreshStatus();
  }

  // ── Sequences ───────────────────────────────────────────
  async function runSequence(name) {
    const statusEl = document.getElementById("seq-status");
    const btn = document.getElementById("seq-btn-" + name);
    if (btn) btn.disabled = true;
    statusEl.textContent = "Running…";
    const ok = await cmd("/sequence?name=" + name);
    statusEl.textContent = ok ? "Done!" : "Failed.";
    if (btn) btn.disabled = false;
    setTimeout(() => { statusEl.textContent = ""; }, 3000);
    await refreshStatus();
  }

  // ── Jog ─────────────────────────────────────────────────
  function setStep(n) {
    document.getElementById("jogStep").value = n;
  }

  async function jog(joint, direction) {
    const step = parseInt(document.getElementById("jogStep").value) || 5;
    const delta = direction * step;
    await cmd("/jog?joint=" + joint.toLowerCase() + "&delta=" + delta);
    await refreshStatus();
  }

  // ── Waypoint capture ────────────────────────────────────
  async function captureWaypoint() {
    if (!connected) { alert("Connect first."); return; }
    const name = document.getElementById("wpName").value.trim() || "WAYPOINT";

    // Fetch current angles
    const r = await fetch(baseUrl + "/status", {cache:"no-store"});
    const d = await r.json();

    // Format as C++ struct
    const line =
      "const ArmPose " + name + " = { " +
      (d.base||0) + ", " + (d.shoulder||0) + ", " +
      (d.elbow||0) + ", " + (d.wrist||0) + ", " +
      (d.claw||0) + " };";

    // Log to page
    const log = document.getElementById("wp-log");
    if (log.textContent === "// waypoints will appear here") log.textContent = "";
    log.textContent += line + "\n";

    // Also trigger Serial print on ESP32
    await cmd("/print");
  }

  function clearLog() {
    document.getElementById("wp-log").textContent = "// waypoints will appear here";
  }

  // ── Tabs ────────────────────────────────────────────────
  function showTab(name) {
    document.querySelectorAll(".tab-content").forEach(el => el.classList.remove("active"));
    document.querySelectorAll(".tab").forEach(el => el.classList.remove("active"));
    document.getElementById("tab-" + name).classList.add("active");
    const tabs = document.querySelectorAll(".tab");
    const names = ["control","jog","waypoints"];
    tabs[names.indexOf(name)].classList.add("active");
  }
</script>
</body>
</html>
)rawliteral";

    _server.send(200, "text/html", html);
}