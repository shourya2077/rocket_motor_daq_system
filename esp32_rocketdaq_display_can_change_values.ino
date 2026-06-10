/*
 * ╔══════════════════════════════════════════════════════════════╗
 *   ROCKET MOTOR STATIC TEST — ESP32 (Wi-Fi DISPLAY + CONTROL)
 * ╠══════════════════════════════════════════════════════════════╣
 *
 *  ROLE : Receives DAQ data from Teensy 4.1, serves a live
 *         dashboard on your phone, lets you control recording,
 *         update calibration & frequency, and download CSV.
 *
 *  MODE : WiFi Access Point  (no router needed)
 *  LIB  : WebServer.h + WiFi.h ONLY (both built-in)
 *
 * ╠══════════════════════════════════════════════════════════════╣
 *  WIRING
 *    ESP32 GPIO 16 (RX2) ← Teensy PIN 1 (TX1)
 *    ESP32 GPIO 17 (TX2) → Teensy PIN 0 (RX1)
 *    ESP32 GND           → Teensy GND  ← CRITICAL
 *
 *  PHONE
 *    WiFi : RocketDAQ  /  12345678
 *    URL  : http://192.168.4.1
 *
 *  ENDPOINTS
 *    GET /           — dashboard HTML
 *    GET /data       — JSON snapshot (polled 100 ms)
 *    GET /cmd?c=X    — TARE | REC_ON | REC_OFF | RESETPEAK
 *    GET /cal?factor=X&period=Y  — update cal factor and/or freq
 *    GET /download   — CSV of last burn
 *    GET /clear      — wipe ESP32 burn buffer
 * ╚══════════════════════════════════════════════════════════════╝
 */

#include <WiFi.h>
#include <WebServer.h>

// ── Access Point ──────────────────────────────────────────────
const char* AP_SSID = "RocketDAQ";
const char* AP_PASS = "12345678";

// ── UART from Teensy ─────────────────────────────────────────
#define RXD2      16
#define TXD2      17
#define UART_BAUD 115200

WebServer server(80);

// ── Live telemetry ────────────────────────────────────────────
struct Telem {
  float    force_N      = 0;
  float    force_kg     = 0;
  float    temp_C       = 0;
  float    pres_MPa     = 0;
  float    impulse_Ns   = 0;
  float    peak_N       = 0;
  float    peak_kg      = 0;
  uint32_t recCount     = 0;
  bool     hasData      = false;
  bool     isRecording  = false;
  uint32_t lastRxMs     = 0;
};
Telem live;

// ── Burn recorder ─────────────────────────────────────────────
#define REC_BUF_SIZE 2000
struct BurnSample {
  uint32_t t_ms;
  float    force_N;
  float    force_kg;
  float    temp_C;
  float    pres_MPa;
  float    impulse_Ns;
};
BurnSample burnBuf[REC_BUF_SIZE];
uint32_t   burnCount    = 0;
bool       wasRecording = false;

// ── Cal / Freq state (mirrors what Teensy currently has) ──────
float    currentCal        = 107809.50451f;
uint32_t currentFreqPeriod = 50;             // ms  (50=20Hz default)

char statusMsg[64] = "Waiting for Teensy...";

// ═════════════════════════════════════════════════════════════
//  UART PARSER
// ═════════════════════════════════════════════════════════════
void parseLine(const String& line) {
  if (!line.startsWith("$")) return;

  auto extract = [&](const char* key) -> float {
    int idx = line.indexOf(key);
    if (idx < 0) return 0.0f;
    idx += strlen(key);
    int end = line.indexOf(',', idx);
    return (end < 0 ? line.substring(idx) : line.substring(idx, end)).toFloat();
  };
  auto extractUL = [&](const char* key) -> uint32_t {
    int idx = line.indexOf(key);
    if (idx < 0) return 0;
    idx += strlen(key);
    int end = line.indexOf(',', idx);
    return (uint32_t)(end < 0 ? line.substring(idx) : line.substring(idx, end)).toInt();
  };

  live.force_N     = extract("F:");
  live.force_kg    = extract("FK:");
  live.temp_C      = extract("T:");
  live.pres_MPa    = extract("P:");
  live.impulse_Ns  = extract("I:");
  live.peak_N      = extract("PK:");
  live.peak_kg     = extract("PKK:");
  live.recCount    = extractUL("RC:");
  live.hasData     = (extractUL("HD:") == 1);
  live.isRecording = (extractUL("REC:") == 1);
  live.lastRxMs    = millis();

  // ── Auto-capture burn buffer ──────────────────────────────
  if (live.isRecording) {
    if (!wasRecording) { burnCount = 0; wasRecording = true; }
    if (burnCount < REC_BUF_SIZE) {
      burnBuf[burnCount++] = {
        millis(), live.force_N, live.force_kg,
        live.temp_C, live.pres_MPa, live.impulse_Ns
      };
    }
  } else {
    if (wasRecording) {
      snprintf(statusMsg, sizeof(statusMsg),
               "Burn complete. %lu samples captured.", burnCount);
      wasRecording = false;
    }
  }
}

// ═════════════════════════════════════════════════════════════
//  DASHBOARD HTML
// ═════════════════════════════════════════════════════════════
static const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>RocketDAQ</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Barlow+Condensed:wght@400;600;700;900&display=swap');

  :root {
    --bg:       #050608;
    --panel:    #0c0e12;
    --border:   #1a1f2e;
    --accent:   #ff4b1f;
    --accent2:  #ff9f0a;
    --green:    #30d158;
    --blue:     #0a84ff;
    --text:     #e8eaf0;
    --muted:    #555e78;
    --font-hud: 'Share Tech Mono', monospace;
    --font-ui:  'Barlow Condensed', sans-serif;
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--font-ui);
    min-height: 100vh;
    padding: 0 0 32px;
    background-image: repeating-linear-gradient(
      0deg, transparent, transparent 2px,
      rgba(255,255,255,.012) 2px, rgba(255,255,255,.012) 4px);
  }

  .header {
    background: linear-gradient(135deg, #0c0e12 0%, #111520 100%);
    border-bottom: 2px solid var(--accent);
    padding: 14px 20px;
    display: flex; align-items: center; justify-content: space-between;
    position: sticky; top: 0; z-index: 100;
    box-shadow: 0 4px 24px rgba(255,75,31,.18);
  }
  .logo { font-family: var(--font-ui); font-weight: 900; font-size: 1.45rem;
    letter-spacing: .12em; text-transform: uppercase;
    color: var(--accent); text-shadow: 0 0 18px rgba(255,75,31,.5); }
  .logo span { color: var(--text); }
  .conn-badge { font-family: var(--font-hud); font-size: .72rem;
    padding: 4px 10px; border-radius: 3px; border: 1px solid var(--muted);
    color: var(--muted); transition: all .3s; }
  .conn-badge.live { border-color: var(--green); color: var(--green);
    box-shadow: 0 0 8px rgba(48,209,88,.3); }
  .conn-badge.rec  { border-color: var(--accent); color: var(--accent);
    box-shadow: 0 0 10px rgba(255,75,31,.5); animation: blink .8s step-end infinite; }
  @keyframes blink { 50% { opacity: .3; } }

  .main { padding: 16px; max-width: 600px; margin: 0 auto; }

  .thrust-section {
    background: var(--panel); border: 1px solid var(--border);
    border-top: 3px solid var(--accent); border-radius: 8px;
    padding: 18px; margin-bottom: 14px;
  }
  .thrust-label { font-family: var(--font-ui); font-weight: 700;
    font-size: .75rem; letter-spacing: .15em; text-transform: uppercase;
    color: var(--muted); margin-bottom: 6px; }
  .thrust-value { font-family: var(--font-hud); font-size: 3.8rem;
    color: var(--accent); line-height: 1;
    text-shadow: 0 0 24px rgba(255,75,31,.4); transition: color .15s; }
  .thrust-value .unit { font-size: 1.3rem; color: var(--muted); margin-left: 6px; }
  .thrust-kg { font-family: var(--font-hud); font-size: 1.1rem;
    color: var(--muted); margin-top: 2px; }
  .bar-track { margin-top: 12px; height: 10px;
    background: rgba(255,255,255,.05); border-radius: 5px;
    overflow: hidden; border: 1px solid var(--border); }
  .bar-fill { height: 100%; width: 0%;
    background: linear-gradient(90deg, var(--accent2), var(--accent));
    border-radius: 5px; transition: width .1s linear;
    box-shadow: 0 0 8px rgba(255,75,31,.5); }
  .bar-fill.zero { background: var(--muted); box-shadow: none; }

  .grid { display: grid; grid-template-columns: 1fr 1fr;
    gap: 10px; margin-bottom: 14px; }
  .card { background: var(--panel); border: 1px solid var(--border);
    border-radius: 8px; padding: 14px 16px; position: relative; overflow: hidden; }
  .card::before { content: ''; position: absolute; top: 0; left: 0; right: 0; height: 2px; }
  .card.temp::before { background: var(--accent2); }
  .card.pres::before { background: var(--blue); }
  .card.imp::before  { background: var(--green); }
  .card.peak::before { background: var(--accent); }
  .card .c-label { font-family: var(--font-ui); font-weight: 700;
    font-size: .68rem; letter-spacing: .13em; text-transform: uppercase;
    color: var(--muted); margin-bottom: 5px; }
  .card .c-val { font-family: var(--font-hud); font-size: 1.75rem;
    line-height: 1; transition: color .2s; }
  .card.temp .c-val { color: var(--accent2); }
  .card.pres .c-val { color: var(--blue); }
  .card.imp  .c-val { color: var(--green); }
  .card.peak .c-val { color: var(--accent); }
  .card .c-unit { font-size: .8rem; color: var(--muted); margin-left: 3px; }

  .wide-row { display: grid; grid-template-columns: 1fr 1fr;
    gap: 10px; margin-bottom: 14px; }

  .chart-section { background: var(--panel); border: 1px solid var(--border);
    border-top: 2px solid var(--accent2); border-radius: 8px;
    padding: 14px 16px; margin-bottom: 14px; }
  .chart-title { font-family: var(--font-ui); font-weight: 700;
    font-size: .72rem; letter-spacing: .13em; text-transform: uppercase;
    color: var(--muted); margin-bottom: 10px; }
  canvas#thrustChart { width: 100% !important; height: 120px; display: block; border-radius: 4px; }

  .controls { background: var(--panel); border: 1px solid var(--border);
    border-radius: 8px; padding: 16px; margin-bottom: 14px; }
  .ctrl-title { font-family: var(--font-ui); font-weight: 700;
    font-size: .72rem; letter-spacing: .15em; text-transform: uppercase;
    color: var(--muted); margin-bottom: 12px; }
  .btn-row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 8px; }
  .btn { font-family: var(--font-ui); font-weight: 700; font-size: .95rem;
    letter-spacing: .06em; text-transform: uppercase; padding: 12px 8px;
    border: none; border-radius: 5px; cursor: pointer; transition: all .15s;
    outline: none; -webkit-tap-highlight-color: transparent; }
  .btn:active { transform: scale(.96); }
  .btn-rec { background: var(--accent); color: #fff; box-shadow: 0 0 14px rgba(255,75,31,.35); }
  .btn-rec:hover { background: #ff6340; }
  .btn-rec.active { background: #222; border: 2px solid var(--accent);
    color: var(--accent); box-shadow: 0 0 16px rgba(255,75,31,.5); }
  .btn-tare { background: #1a1f2e; color: var(--text); border: 1px solid var(--border); }
  .btn-tare:hover { background: #22293d; border-color: var(--muted); }
  .btn-peak { background: #1a1f2e; color: var(--muted); border: 1px solid var(--border); }
  .btn-peak:hover { color: var(--accent2); border-color: var(--accent2); }
  .btn-save { background: var(--green); color: #000; font-weight: 900;
    box-shadow: 0 0 14px rgba(48,209,88,.25); }
  .btn-save:hover { background: #4dde74; }
  .btn-save:disabled { background: #1a1f2e; color: var(--muted); box-shadow: none; cursor: not-allowed; }

  /* ── Cal panel specific styles ─────────────────────────── */
  .cal-row { display:flex; justify-content:space-between; align-items:center;
    font-family: var(--font-hud); font-size:.72rem; color: var(--muted);
    background:#070910; border:1px solid var(--border);
    border-radius:6px; padding:8px 12px; margin-bottom:12px; }
  .cal-row span b { font-weight:normal; }
  .cal-row .cv { color: var(--accent2); }
  .cal-row .fv { color: var(--blue); }
  .inp-row { margin-bottom: 10px; }
  .inp-label { font-family: var(--font-ui); font-weight: 700;
    font-size: .68rem; letter-spacing:.13em; text-transform:uppercase;
    color: var(--muted); display:block; margin-bottom:5px; }
  .inp-group { display:flex; gap:8px; }
  .inp-group input, .inp-group select {
    flex:1; background:#070910; border:1px solid var(--border);
    border-radius:5px; color: var(--text); padding:10px 12px;
    font-family: var(--font-hud); font-size:.85rem; outline:none;
    -webkit-appearance:none; appearance:none; }
  .inp-group input:focus, .inp-group select:focus {
    border-color: var(--accent2); }
  .btn-full { width:100%; margin-top:4px; }

  .status-bar { background: #070910; border: 1px solid var(--border);
    border-radius: 6px; padding: 10px 14px; font-family: var(--font-hud);
    font-size: .72rem; color: var(--muted); min-height: 38px; word-break: break-all; }
  .status-bar .ok   { color: var(--green); }
  .status-bar .warn { color: var(--accent2); }
  .status-bar .err  { color: var(--accent); }
</style>
</head>
<body>

<div class="header">
  <div class="logo">ROCKET<span>DAQ</span></div>
  <div class="conn-badge" id="connBadge">NO SIGNAL</div>
</div>

<div class="main">

  <!-- Thrust -->
  <div class="thrust-section">
    <div class="thrust-label">Thrust (smoothed)</div>
    <div class="thrust-value" id="forceN">--.---<span class="unit">N</span></div>
    <div class="thrust-kg" id="forceKg">-- kg</div>
    <div class="bar-track"><div class="bar-fill zero" id="thrustBar"></div></div>
  </div>

  <!-- Metric grid -->
  <div class="grid">
    <div class="card temp">
      <div class="c-label">Temperature</div>
      <div class="c-val" id="tempC">--<span class="c-unit">°C</span></div>
    </div>
    <div class="card pres">
      <div class="c-label">Pressure</div>
      <div class="c-val" id="presM">--<span class="c-unit">MPa</span></div>
    </div>
    <div class="card imp">
      <div class="c-label">Impulse</div>
      <div class="c-val" id="impulse">--<span class="c-unit">N·s</span></div>
    </div>
    <div class="card peak">
      <div class="c-label">Peak Thrust</div>
      <div class="c-val" id="peakN">--<span class="c-unit">N</span></div>
    </div>
  </div>

  <div class="wide-row">
    <div class="card">
      <div class="c-label" style="color:var(--muted)">Peak (kg)</div>
      <div class="c-val" style="color:var(--accent)" id="peakKg">--<span class="c-unit">kg</span></div>
    </div>
    <div class="card">
      <div class="c-label" style="color:var(--muted)">Samples (Teensy)</div>
      <div class="c-val" style="color:var(--text);font-size:1.4rem" id="recCount">0</div>
    </div>
  </div>

  <!-- Mini chart -->
  <div class="chart-section">
    <div class="chart-title">Thrust History (last 100 pts)</div>
    <canvas id="thrustChart"></canvas>
  </div>

  <!-- Recording controls -->
  <div class="controls">
    <div class="ctrl-title">Controls</div>
    <div class="btn-row">
      <button class="btn btn-rec" id="btnRec" onclick="cmdRec()">&#9654; START REC</button>
      <button class="btn btn-tare" onclick="sendCmd('TARE')">&#9651; TARE</button>
    </div>
    <div class="btn-row">
      <button class="btn btn-peak" onclick="sendCmd('RESETPEAK')">&#8635; RESET PEAK</button>
      <button class="btn btn-save" id="btnSave" onclick="downloadCSV()" disabled>&#8615; SAVE CSV</button>
    </div>
  </div>

  <!-- ── Calibration & Frequency panel ────────────────────── -->
  <div class="controls">
    <div class="ctrl-title">Calibration &amp; Frequency</div>

    <!-- Live readout of current Teensy values -->
    <div class="cal-row">
      <span>CAL: <span class="cv" id="curCal">107809.50451</span></span>
      <span>RATE: <span class="fv" id="curFreq">20 Hz</span></span>
    </div>

    <!-- Cal factor input -->
    <div class="inp-row">
      <label class="inp-label">Load Cell Cal Factor</label>
      <div class="inp-group">
        <input id="inCal" type="number" step="0.001" value="107809.50451">
        <button class="btn btn-tare" style="flex:0 0 60px" onclick="sendCal()">SET</button>
      </div>
    </div>

    <!-- Frequency selector -->
    <div class="inp-row">
      <label class="inp-label">Teensy → ESP32 Rate</label>
      <div class="inp-group">
        <select id="inFreq">
          <option value="100">10 Hz  (100 ms)</option>
          <option value="50" selected>20 Hz  (50 ms) — default</option>
          <option value="20">50 Hz  (20 ms)</option>
          <option value="10">100 Hz (10 ms)</option>
        </select>
        <button class="btn btn-tare" style="flex:0 0 60px" onclick="sendFreq()">SET</button>
      </div>
    </div>

    <!-- Apply both at once -->
    <button class="btn btn-tare btn-full" onclick="sendBoth()">
      &#8593; APPLY BOTH TO TEENSY
    </button>
  </div>

  <!-- Status bar -->
  <div class="status-bar" id="statusBar">Connecting to Teensy...</div>

</div><!-- /main -->

<script>
// ── Chart ──────────────────────────────────────────────────
const CHART_LEN = 100;
const thrustHistory = new Array(CHART_LEN).fill(0);
const canvas = document.getElementById('thrustChart');
const ctx    = canvas.getContext('2d');
let peakSeen = 0;

function resizeCanvas() {
  canvas.width  = canvas.offsetWidth  * window.devicePixelRatio;
  canvas.height = canvas.offsetHeight * window.devicePixelRatio;
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
}
resizeCanvas();
window.addEventListener('resize', resizeCanvas);

function drawChart(data, maxVal) {
  const W = canvas.offsetWidth, H = canvas.offsetHeight;
  ctx.clearRect(0, 0, W, H);
  ctx.strokeStyle = 'rgba(255,255,255,0.05)';
  ctx.lineWidth = 1;
  for (let i = 1; i < 4; i++) {
    const y = (H / 4) * i;
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();
  }
  if (maxVal < 0.1) return;
  const grad = ctx.createLinearGradient(0,0,0,H);
  grad.addColorStop(0,   'rgba(255,75,31,0.5)');
  grad.addColorStop(0.6, 'rgba(255,75,31,0.1)');
  grad.addColorStop(1,   'rgba(255,75,31,0.0)');
  const step = W / (CHART_LEN - 1);
  ctx.beginPath(); ctx.moveTo(0, H);
  data.forEach((v,i) => ctx.lineTo(i*step, H-(v/maxVal)*H*0.92));
  ctx.lineTo(W, H); ctx.closePath();
  ctx.fillStyle = grad; ctx.fill();
  ctx.beginPath();
  ctx.strokeStyle = '#ff4b1f'; ctx.lineWidth = 1.5;
  ctx.shadowColor = '#ff4b1f'; ctx.shadowBlur = 6;
  data.forEach((v,i) => { const x=i*step, y=H-(v/maxVal)*H*0.92; i===0?ctx.moveTo(x,y):ctx.lineTo(x,y); });
  ctx.stroke(); ctx.shadowBlur = 0;
}

// ── State ──────────────────────────────────────────────────
let isRecording = false;
let hasBurnData = false;
let lastRxOk    = 0;
let pollFails   = 0;

const el = id => document.getElementById(id);

// ── Update UI ─────────────────────────────────────────────
function updateUI(d) {
  const fN  = parseFloat(d.force_N)    || 0;
  const fKg = parseFloat(d.force_kg)   || 0;
  const tC  = parseFloat(d.temp_C)     || 0;
  const pM  = parseFloat(d.pres_MPa)   || 0;
  const imp = parseFloat(d.impulse_Ns) || 0;
  const pkN = parseFloat(d.peak_N)     || 0;
  const pkK = parseFloat(d.peak_kg)    || 0;
  const rc  = parseInt(d.rec_count)    || 0;
  const rec = d.recording === true || d.recording === 1 || d.recording === "1";
  const hd  = d.has_data  === true || d.has_data  === 1 || d.has_data  === "1";

  el('forceN').innerHTML  = fN.toFixed(3) + '<span class="unit">N</span>';
  el('forceKg').textContent = fKg.toFixed(4) + ' kg';

  const BAR_MAX = Math.max(pkN * 1.1, 120);
  const pct = Math.min(100, (fN / BAR_MAX) * 100);
  const bar = el('thrustBar');
  bar.style.width = pct + '%';
  bar.className   = 'bar-fill' + (fN < 0.1 ? ' zero' : '');

  el('tempC').innerHTML   = tC.toFixed(1)  + '<span class="c-unit">°C</span>';
  el('presM').innerHTML   = pM.toFixed(3)  + '<span class="c-unit">MPa</span>';
  el('impulse').innerHTML = imp.toFixed(2) + '<span class="c-unit">N·s</span>';
  el('peakN').innerHTML   = pkN.toFixed(3) + '<span class="c-unit">N</span>';
  el('peakKg').innerHTML  = pkK.toFixed(4) + '<span class="c-unit">kg</span>';
  el('recCount').textContent = rc;

  thrustHistory.shift(); thrustHistory.push(fN);
  peakSeen = Math.max(peakSeen, fN);
  drawChart(thrustHistory, peakSeen < 1 ? 10 : peakSeen);

  isRecording = rec;
  const badge  = el('connBadge');
  const btnRec = el('btnRec');
  if (rec) {
    badge.className = 'conn-badge rec'; badge.textContent = '● REC';
    btnRec.className = 'btn btn-rec active';
    btnRec.innerHTML = '&#9646;&#9646; STOP REC';
  } else {
    badge.className = 'conn-badge live'; badge.textContent = '◉ LIVE';
    btnRec.className = 'btn btn-rec';
    btnRec.innerHTML = '&#9654; START REC';
  }

  hasBurnData = hd || (d.esp_samples && parseInt(d.esp_samples) > 0);
  el('btnSave').disabled = !hasBurnData;

  // Update cal/freq display from ESP32 confirmed values
  if (d.cal)    el('curCal').textContent  = parseFloat(d.cal).toFixed(5);
  if (d.freq_hz) el('curFreq').textContent = d.freq_hz + ' Hz';

  lastRxOk = Date.now(); pollFails = 0;
  el('statusBar').innerHTML = '<span class="ok">● LIVE</span> — Teensy streaming. '
    + 'ESP32 captured: ' + (d.esp_samples || 0) + ' pts';
}

// ── Poll /data every 100 ms ───────────────────────────────
async function poll() {
  try {
    const r = await fetch('/data', { cache: 'no-store' });
    if (!r.ok) throw new Error('HTTP ' + r.status);
    updateUI(await r.json());
  } catch(e) {
    pollFails++;
    if (pollFails > 3) {
      const age = Math.round((Date.now() - lastRxOk) / 1000);
      el('connBadge').className = 'conn-badge';
      el('connBadge').textContent = 'NO SIGNAL';
      el('statusBar').innerHTML =
        '<span class="err">✖ OFFLINE</span> — No response for ' + age + 's.';
    }
  }
  setTimeout(poll, 100);
}

// ── Send command ──────────────────────────────────────────
async function sendCmd(cmd) {
  el('statusBar').innerHTML = '<span class="warn">↑ CMD</span> — Sending: ' + cmd + '...';
  try {
    const r = await fetch('/cmd?c=' + cmd, { cache: 'no-store' });
    el('statusBar').innerHTML = '<span class="ok">✓ CMD</span> — ' + await r.text();
  } catch(e) {
    el('statusBar').innerHTML = '<span class="err">✖ CMD FAILED</span> — ' + e.message;
  }
}

function cmdRec() { sendCmd(isRecording ? 'REC_OFF' : 'REC_ON'); }

// ── Download CSV ──────────────────────────────────────────
async function downloadCSV() {
  el('statusBar').innerHTML = '<span class="warn">↓ DOWNLOAD</span> — Fetching burn data...';
  try {
    const r = await fetch('/download', { cache: 'no-store' });
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const blob = await r.blob();
    const url  = URL.createObjectURL(blob);
    const a    = document.createElement('a');
    const ts   = new Date().toISOString().replace(/[:.]/g,'-').slice(0,19);
    a.href = url; a.download = 'burn_' + ts + '.csv';
    document.body.appendChild(a); a.click();
    document.body.removeChild(a); URL.revokeObjectURL(url);
    el('statusBar').innerHTML = '<span class="ok">✓ SAVED</span> — burn_' + ts + '.csv downloaded.';
  } catch(e) {
    el('statusBar').innerHTML = '<span class="err">✖ DOWNLOAD FAILED</span> — ' + e.message;
  }
}

// ── Calibration helpers ───────────────────────────────────
async function sendCal() {
  const f = parseFloat(el('inCal').value);
  if (isNaN(f) || f <= 100) {
    el('statusBar').innerHTML = '<span class="err">✖ CAL</span> — Value must be > 100'; return;
  }
  el('statusBar').innerHTML = '<span class="warn">↑ CAL</span> — Sending factor: ' + f + '...';
  try {
    const r = await fetch('/cal?factor=' + f, { cache: 'no-store' });
    if (!r.ok) throw new Error(await r.text());
    const d = await r.json();
    el('curCal').textContent = d.cal.toFixed(5);
    el('statusBar').innerHTML = '<span class="ok">✓ CAL</span> — Teensy updated: ' + d.cal.toFixed(5);
  } catch(e) {
    el('statusBar').innerHTML = '<span class="err">✖ CAL FAILED</span> — ' + e.message;
  }
}

async function sendFreq() {
  const p = parseInt(el('inFreq').value);
  el('statusBar').innerHTML = '<span class="warn">↑ FREQ</span> — Setting: ' + p + ' ms...';
  try {
    const r = await fetch('/cal?period=' + p, { cache: 'no-store' });
    if (!r.ok) throw new Error(await r.text());
    const d = await r.json();
    el('curFreq').textContent = d.hz + ' Hz';
    el('statusBar').innerHTML = '<span class="ok">✓ FREQ</span> — Teensy updated: ' + d.hz + ' Hz (' + d.period + ' ms)';
  } catch(e) {
    el('statusBar').innerHTML = '<span class="err">✖ FREQ FAILED</span> — ' + e.message;
  }
}

async function sendBoth() {
  const f = parseFloat(el('inCal').value);
  const p = parseInt(el('inFreq').value);
  if (isNaN(f) || f <= 100) {
    el('statusBar').innerHTML = '<span class="err">✖</span> — Cal factor must be > 100'; return;
  }
  el('statusBar').innerHTML = '<span class="warn">↑ APPLY</span> — Sending CAL + FREQ...';
  try {
    const r = await fetch('/cal?factor=' + f + '&period=' + p, { cache: 'no-store' });
    if (!r.ok) throw new Error(await r.text());
    const d = await r.json();
    el('curCal').textContent  = d.cal.toFixed(5);
    el('curFreq').textContent = d.hz + ' Hz';
    el('statusBar').innerHTML = '<span class="ok">✓ APPLIED</span> — CAL: '
      + d.cal.toFixed(5) + '  FREQ: ' + d.hz + ' Hz';
  } catch(e) {
    el('statusBar').innerHTML = '<span class="err">✖ FAILED</span> — ' + e.message;
  }
}

poll();
</script>
</body>
</html>
)rawhtml";

// ═════════════════════════════════════════════════════════════
//  HTTP HANDLERS
// ═════════════════════════════════════════════════════════════

void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleData() {
  uint32_t age = millis() - live.lastRxMs;
  char json[320];
  snprintf(json, sizeof(json),
    "{"
    "\"force_N\":%.3f,"
    "\"force_kg\":%.4f,"
    "\"temp_C\":%.2f,"
    "\"pres_MPa\":%.4f,"
    "\"impulse_Ns\":%.3f,"
    "\"peak_N\":%.3f,"
    "\"peak_kg\":%.4f,"
    "\"rec_count\":%lu,"
    "\"has_data\":%d,"
    "\"recording\":%d,"
    "\"esp_samples\":%lu,"
    "\"age_ms\":%lu,"
    "\"cal\":%.5f,"
    "\"freq_hz\":%lu"
    "}",
    live.force_N,   live.force_kg,
    live.temp_C,    live.pres_MPa,
    live.impulse_Ns,
    live.peak_N,    live.peak_kg,
    live.recCount,
    live.hasData     ? 1 : 0,
    live.isRecording ? 1 : 0,
    burnCount,
    age,
    currentCal,
    1000UL / currentFreqPeriod
  );
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", json);
}

void handleCmd() {
  if (!server.hasArg("c")) { server.send(400, "text/plain", "Missing ?c="); return; }
  String cmd = server.arg("c");
  if (cmd == "TARE" || cmd == "REC_ON" || cmd == "REC_OFF" || cmd == "RESETPEAK") {
    Serial2.println(cmd);
    Serial.print("CMD → Teensy: "); Serial.println(cmd);
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "text/plain", "OK: " + cmd + " sent to Teensy");
  } else {
    server.send(400, "text/plain", "Unknown command: " + cmd);
  }
}

// GET /cal?factor=X&period=Y
void handleCal() {
  bool didSomething = false;

  if (server.hasArg("factor")) {
    float f = server.arg("factor").toFloat();
    if (f > 100.0f) {
      currentCal = f;
      char cmd[32];
      snprintf(cmd, sizeof(cmd), "CAL:%.5f", f);
      Serial2.println(cmd);
      Serial.printf("→ Teensy: %s\n", cmd);
      didSomething = true;
    } else {
      server.send(400, "text/plain", "ERROR: factor must be > 100"); return;
    }
  }

  if (server.hasArg("period")) {
    uint32_t p = (uint32_t)server.arg("period").toInt();
    if (p >= 10 && p <= 1000) {
      currentFreqPeriod = p;
      char cmd[24];
      snprintf(cmd, sizeof(cmd), "FREQ:%lu", p);
      Serial2.println(cmd);
      Serial.printf("→ Teensy: %s\n", cmd);
      didSomething = true;
    } else {
      server.send(400, "text/plain", "ERROR: period must be 10–1000 ms"); return;
    }
  }

  if (!didSomething) {
    server.send(400, "text/plain", "ERROR: provide ?factor= or ?period= or both"); return;
  }

  char json[96];
  snprintf(json, sizeof(json),
    "{\"cal\":%.5f,\"period\":%lu,\"hz\":%lu}",
    currentCal, currentFreqPeriod, 1000UL / currentFreqPeriod);
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", json);
}

void handleDownload() {
  if (burnCount == 0) {
    server.send(404, "text/plain", "No burn data yet. Start a recording first."); return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=\"burn_data.csv\"");
  server.sendHeader("Cache-Control", "no-cache");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("sample,time_ms,force_N,force_kg,temp_C,pres_MPa,impulse_Ns\r\n");
  for (uint32_t i = 0; i < burnCount; i++) {
    char row[96];
    snprintf(row, sizeof(row), "%lu,%lu,%.4f,%.4f,%.2f,%.4f,%.4f\r\n",
      (unsigned long)(i+1), (unsigned long)burnBuf[i].t_ms,
      burnBuf[i].force_N, burnBuf[i].force_kg,
      burnBuf[i].temp_C,  burnBuf[i].pres_MPa, burnBuf[i].impulse_Ns);
    server.sendContent(row);
    if (i % 50 == 49) delay(1);
  }
  server.sendContent("");
  Serial.printf("CSV download: %lu rows sent.\n", burnCount);
}

void handleClear() {
  burnCount = 0; wasRecording = false;
  snprintf(statusMsg, sizeof(statusMsg), "Buffer cleared.");
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/plain", "Buffer cleared.");
  Serial.println("Buffer cleared by phone.");
}

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP: "); Serial.print(AP_SSID);
  Serial.print("  IP: "); Serial.println(WiFi.softAPIP());

  server.on("/",         HTTP_GET, handleRoot);
  server.on("/data",     HTTP_GET, handleData);
  server.on("/cmd",      HTTP_GET, handleCmd);
  server.on("/cal",      HTTP_GET, handleCal);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/clear",    HTTP_GET, handleClear);

  server.begin();
  Serial.println("HTTP server started.");
  Serial.println("Connect phone to: " + String(AP_SSID));
  Serial.println("Open: http://192.168.4.1");
}

// ═════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════
void loop() {
  server.handleClient();

  if (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      // ── ACK from Teensy (confirms cal/freq was applied) ───
      if (line.startsWith("ACK:CAL:")) {
        currentCal = line.substring(8).toFloat();
        Serial.printf("✓ Teensy ACK — CAL: %.5f\n", currentCal);
      } else if (line.startsWith("ACK:FREQ:")) {
        currentFreqPeriod = (uint32_t)line.substring(9).toInt();
        Serial.printf("✓ Teensy ACK — FREQ: %lu ms\n", currentFreqPeriod);
      } else {
        // Normal data packet
        parseLine(line);
      }
    }
  }
}
