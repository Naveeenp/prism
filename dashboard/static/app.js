/* ==============================================================================
   INDUSTRIAL SCADA JAVASCRIPT CONTROLLER - PMDC MOTOR DIAGNOSTIC BENCH
   Features:
     - 10 Hz Real-Time SSE Ingestion with REST Fallback
     - High-Performance HTML5 Canvas 2D Strip Charts (Zero External Chart Libs)
     - Deceleration Slope Marker Rendering During Coast Test
     - Chronological Alarm & Event Ledger Synchronization
   ============================================================================== */

let isConnected = false;
let heartbeatToggle = false;
let sseSource = null;

// Telemetry Buffers for Real-Time Charts (120 points = 12 seconds window @ 10 Hz)
const MAX_POINTS = 120;
const chartBuffers = {
  time: [],
  rpm: [],
  current: [],
  vibration: [],
  temp: []
};

let coastSlopeMarkers = [];
let lastTelemetryTime = Date.now();
let lastPacketCount = 0;
let lastPacketSeq = 0;
let packetRateHz = 10.0;
let lastRateCheckTime = Date.now();

// =============================================================================
// AUDIO ALARM SYNTHESIZER (WEB AUDIO API - ZERO EXTERNAL ASSETS)
// =============================================================================
let audioCtx = null;
let audioEnabled = true;
let lastAudioAlertTime = 0;
let lastAlertProblemCount = 0;

function getAudioContext() {
  if (!audioCtx) {
    const AudioContext = window.AudioContext || window.webkitAudioContext;
    if (AudioContext) {
      audioCtx = new AudioContext();
    }
  }
  if (audioCtx && audioCtx.state === "suspended") {
    audioCtx.resume();
  }
  return audioCtx;
}

// Single Fault: Audible Clear Ping Alert (1000 Hz bell chime)
function playPingAlert() {
  if (!audioEnabled) return;
  try {
    const ctx = getAudioContext();
    if (!ctx) return;
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();

    osc.type = "sine";
    osc.frequency.setValueAtTime(1050, ctx.currentTime);
    osc.frequency.exponentialRampToValueAtTime(800, ctx.currentTime + 0.18);

    gain.gain.setValueAtTime(0.28, ctx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.22);

    osc.connect(gain);
    gain.connect(ctx.destination);

    osc.start(ctx.currentTime);
    osc.stop(ctx.currentTime + 0.23);
  } catch (err) {
    console.warn("Audio ping error:", err);
  }
}

// Multiple Problems: Two-Tone Industrial Warning Chime (880 Hz -> 587 Hz Urgent Clang)
function playMultiProblemChime() {
  if (!audioEnabled) return;
  try {
    const ctx = getAudioContext();
    if (!ctx) return;

    const playTone = (freq, delay, dur) => {
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();

      osc.type = "triangle";
      osc.frequency.setValueAtTime(freq, ctx.currentTime + delay);

      gain.gain.setValueAtTime(0.35, ctx.currentTime + delay);
      gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + delay + dur);

      osc.connect(gain);
      gain.connect(ctx.destination);

      osc.start(ctx.currentTime + delay);
      osc.stop(ctx.currentTime + delay + dur);
    };

    // Urgent 3-pulse descending industrial chime sequence
    playTone(987.77, 0.00, 0.16); // B5
    playTone(783.99, 0.14, 0.16); // G5
    playTone(523.25, 0.28, 0.35); // C5
  } catch (err) {
    console.warn("Audio chime error:", err);
  }
}

function toggleAudioMute() {
  audioEnabled = !audioEnabled;
  const btn = document.getElementById("btnMuteAudio");
  if (btn) {
    if (audioEnabled) {
      btn.textContent = "ALARM AUDIO: ON";
      btn.classList.remove("audio-muted");
      playPingAlert(); // Test chirp
    } else {
      btn.textContent = "ALARM AUDIO: MUTED";
      btn.classList.add("audio-muted");
    }
  }
}

// =============================================================================
// INITIALIZATION
// =============================================================================
window.addEventListener("DOMContentLoaded", () => {
  initCanvases();
  refreshPorts();
  startTelemetryStream();
  setInterval(pollAlarms, 1000);

  // Resume Web Audio on first user interaction anywhere
  const unlockAudio = () => {
    getAudioContext();
    window.removeEventListener("click", unlockAudio);
    window.removeEventListener("keydown", unlockAudio);
  };
  window.addEventListener("click", unlockAudio);
  window.addEventListener("keydown", unlockAudio);
});

// =============================================================================
// API & SERIAL CONNECTION CONTROLS
// =============================================================================
async function refreshPorts() {
  try {
    const res = await fetch("/api/ports");
    const data = await res.json();
    const sel = document.getElementById("portSelect");
    sel.innerHTML = "";

    if (data.ports && data.ports.length > 0) {
      data.ports.forEach(p => {
        const opt = document.createElement("option");
        opt.value = p.device;
        opt.textContent = `${p.device} (${p.description || "ESP32"})`;
        sel.appendChild(opt);
      });
    } else {
      const opt = document.createElement("option");
      opt.value = "COM3";
      opt.textContent = "COM3 (Simulated / Offline)";
      sel.appendChild(opt);
    }

    updateConnectionUI(data.connected);
  } catch (e) {
    console.warn("Could not fetch ports:", e);
  }
}

async function toggleConnection() {
  const port = document.getElementById("portSelect").value;
  const baud = document.getElementById("baudSelect").value;

  if (!isConnected) {
    try {
      const res = await fetch("/api/serial/connect", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ port, baudrate: parseInt(baud, 10) })
      });
      const data = await res.json();
      updateConnectionUI(data.success);
    } catch (e) {
      alert("Failed to connect: " + e);
    }
  } else {
    try {
      await fetch("/api/serial/disconnect", { method: "POST" });
      updateConnectionUI(false);
    } catch (e) {
      console.warn("Disconnect error:", e);
    }
  }
}

function updateConnectionUI(connected) {
  isConnected = connected;
  const btn = document.getElementById("btnConnect");
  if (connected) {
    btn.textContent = "DISCONNECT";
    btn.classList.add("connected");
  } else {
    btn.textContent = "CONNECT";
    btn.classList.remove("connected");
  }
}

let driverPowerActive = false;

// =============================================================================
// OPERATING COMMAND DISPATCHERS
// =============================================================================
async function toggleDriverPower() {
  const targetState = !driverPowerActive;
  try {
    const res = await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "driver_power", enable: targetState })
    });
    const data = await res.json();
    if (data.success) {
      updateDriverPowerUI(targetState);
    }
  } catch (e) {
    console.error("Driver power toggle error:", e);
  }
}

function updateDriverPowerUI(active) {
  driverPowerActive = active;
  const btn = document.getElementById("btnDriverPwr");
  const badge = document.getElementById("badgeDriverPwr");

  if (!btn || !badge) return;

  if (active) {
    btn.textContent = "DISABLE MOTOR DRIVER (POWER OFF)";
    btn.classList.add("driver-active");
    badge.textContent = "ARMED / ON";
    badge.className = "pwr-status-badge badge-on";
  } else {
    btn.textContent = "ENABLE MOTOR DRIVER (ARM POWER)";
    btn.classList.remove("driver-active");
    badge.textContent = "DISARMED";
    badge.className = "pwr-status-badge badge-off";
  }
}

async function setSpeed(pct) {
  pct = Math.max(0, Math.min(100, pct));
  const slider = document.getElementById("speedSlider");
  const label = document.getElementById("speedSliderVal");
  if (slider) slider.value = pct;
  if (label) {
    const duty8bit = Math.round((pct / 100.0) * 255);
    label.textContent = `${pct}% (${duty8bit} / 255)`;
  }

  try {
    await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "set_speed", speed: pct })
    });
  } catch (e) {
    console.error("Speed command error:", e);
  }
}

let sliderDebounceTimer = null;
function onSpeedSliderChange(val) {
  const pct = parseInt(val, 10);
  const duty8bit = Math.round((pct / 100.0) * 255);
  const label = document.getElementById("speedSliderVal");
  if (label) label.textContent = `${pct}% (${duty8bit} / 255)`;

  if (sliderDebounceTimer) clearTimeout(sliderDebounceTimer);
  sliderDebounceTimer = setTimeout(() => {
    setSpeed(pct);
  }, 120);
}

async function sendEStop() {
  try {
    await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "stop" })
    });
  } catch (e) {
    console.error("E-STOP error:", e);
  }
}

async function triggerRaTest() {
  try {
    await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "ra_test" })
    });
  } catch (e) {
    console.error("Ra test trigger error:", e);
  }
}

async function triggerCoastTest() {
  try {
    await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "coast_test" })
    });
  } catch (e) {
    console.error("Coast test trigger error:", e);
  }
}

// =============================================================================
// FAULT INJECTION LAB (SIMULATION MODE CONTROLS)
// =============================================================================
const activeFaults = {
  overheat: false,
  overcurrent: false,
  stall: false,
  bearing_drag: false,
  vibration: false,
  voltage_drop: false
};

const faultBtnIds = {
  overheat: "fltOverheat",
  overcurrent: "fltOvercurrent",
  stall: "fltStall",
  bearing_drag: "fltBearing",
  vibration: "fltVib",
  voltage_drop: "fltVoltDrop"
};

async function toggleFault(faultName) {
  if (activeFaults[faultName] === undefined) return;
  activeFaults[faultName] = !activeFaults[faultName];

  const btn = document.getElementById(faultBtnIds[faultName]);
  if (btn) {
    if (activeFaults[faultName]) {
      btn.classList.add("fault-active");
    } else {
      btn.classList.remove("fault-active");
    }
  }

  try {
    await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        action: "inject_fault",
        faults: { [faultName]: activeFaults[faultName] }
      })
    });
  } catch (e) {
    console.error("Fault injection error:", e);
  }
}

async function clearAllFaults() {
  for (const k in activeFaults) {
    activeFaults[k] = false;
    const btn = document.getElementById(faultBtnIds[k]);
    if (btn) btn.classList.remove("fault-active");
  }

  try {
    await fetch("/api/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "clear_faults" })
    });
  } catch (e) {
    console.error("Clear faults error:", e);
  }
}

// =============================================================================
// WEBSOCKET REAL-TIME TELEMETRY CONSUMER
// =============================================================================
let telemetrySocket = null;
let reconnectTimer = null;

function startTelemetryStream() {
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;

  try {
    telemetrySocket = new WebSocket(wsUrl);

    telemetrySocket.onopen = () => {
      console.log("WebSocket connection established to", wsUrl);
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
      }
    };

    telemetrySocket.onmessage = (event) => {
      try {
        const packet = JSON.parse(event.data);
        handleTelemetryUpdate(packet);
      } catch (err) {
        console.error("Telemetry JSON parse error:", err);
      }
    };

    telemetrySocket.onclose = () => {
      console.warn("WebSocket closed. Attempting reconnect in 1.5s...");
      scheduleReconnect();
    };

    telemetrySocket.onerror = (err) => {
      console.error("WebSocket error:", err);
      try { telemetrySocket.close(); } catch (e) {}
    };
  } catch (err) {
    console.error("Failed to initiate WebSocket:", err);
    scheduleReconnect();
  }
}

function scheduleReconnect() {
  if (!reconnectTimer) {
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null;
      startTelemetryStream();
    }, 1500);
  }
}

function handleTelemetryUpdate(t) {
  lastTelemetryTime = Date.now();
  lastPacketCount++;

  // 1. Toggle Heartbeat Indicator
  heartbeatToggle = !heartbeatToggle;
  const hbLed = document.getElementById("heartbeatLed");
  if (heartbeatToggle) {
    hbLed.classList.add("active");
  } else {
    hbLed.classList.remove("active");
  }

  // Clear stale readouts style if active
  const readoutContainer = document.getElementById("readoutContainer");
  if (readoutContainer) readoutContainer.classList.remove("readout-stale");

  // Sync Driver Power UI if reported in telemetry
  if (t.driver_pwr !== undefined && t.driver_pwr !== (driverPowerActive ? 1 : 0)) {
    updateDriverPowerUI(Boolean(t.driver_pwr));
  }

  // 2. Update Digital Metric Readouts
  document.getElementById("valVBus").textContent = (t.v_bus || 0).toFixed(2);
  document.getElementById("valIMa").textContent = (t.i_ma || 0).toFixed(1);
  document.getElementById("valRpm").textContent = (t.rpm || 0).toFixed(1);
  document.getElementById("valTemp").textContent = (t.temp_c || 0).toFixed(1);
  document.getElementById("valVib").textContent = (t.vib_g || 0).toFixed(3);
  document.getElementById("valRa").textContent = (t.ra_ohms || 2.40).toFixed(2);
  document.getElementById("valCoast").textContent = (t.coast_duration_s || 0.0).toFixed(2);

  const dutyElem = document.getElementById("valDuty");
  if (dutyElem) dutyElem.textContent = (t.duty !== undefined ? t.duty : 0);

  const bciElem = document.getElementById("valBci");
  const bciVal = (t.bci_pct !== undefined ? t.bci_pct : 100.0);
  bciElem.textContent = bciVal.toFixed(1);

  if (bciVal < 60.0) {
    bciElem.style.color = "#F44336";
  } else if (bciVal < 85.0) {
    bciElem.style.color = "#FFC107";
  } else {
    bciElem.style.color = "#4CAF50";
  }

  // 3. Update Equipment Status Panel
  const statSerial = document.getElementById("statSerial");
  if (statSerial) {
    if (t.connected) {
      statSerial.textContent = `CONNECTED (${t.port || "ONLINE"})`;
      statSerial.className = "badge-status badge-ok";
    } else {
      statSerial.textContent = "SIMULATION / OFFLINE";
      statSerial.className = "badge-status badge-warn";
    }
  }

  const statController = document.getElementById("statController");
  if (statController) {
    statController.textContent = t.connected ? "ESP-32S DUAL-CORE" : "PYTHON SIMULATOR";
    statController.className = "badge-status badge-ok";
  }

  const statMode = document.getElementById("statMode");
  if (statMode) {
    statMode.textContent = t.mode || "STANDBY";
  }

  const statActiveTest = document.getElementById("statActiveTest");
  if (statActiveTest) {
    if (t.mode === "RA_TEST") {
      statActiveTest.textContent = "MEASURING Ra...";
      statActiveTest.className = "badge-status badge-warn";
    } else if (t.mode && t.mode.startsWith("COAST")) {
      statActiveTest.textContent = "COAST-DOWN RECORDING";
      statActiveTest.className = "badge-status badge-warn";
    } else {
      statActiveTest.textContent = "MONITORING READY";
      statActiveTest.className = "badge-status badge-ok";
    }
  }

  // 4. Update Telemetry Diagnostics Panel
  const pktSeqElem = document.getElementById("diagPktSeq");
  if (pktSeqElem) pktSeqElem.textContent = t.seq || 0;

  const pulsesElem = document.getElementById("diagPulses");
  if (pulsesElem) pulsesElem.textContent = t.pulses || 0;

  const inaElem = document.getElementById("diagInaOk");
  if (inaElem) {
    const ok = t.ina_ok !== undefined ? Boolean(t.ina_ok) : true;
    inaElem.textContent = ok ? "ONLINE" : "FAULT / OFFLINE";
    inaElem.className = ok ? "diag-stat-val badge-ok" : "diag-stat-val badge-err";
  }

  const mpuElem = document.getElementById("diagMpuOk");
  if (mpuElem) {
    const ok = t.mpu_ok !== undefined ? Boolean(t.mpu_ok) : true;
    mpuElem.textContent = ok ? "ONLINE" : "FAULT / OFFLINE";
    mpuElem.className = ok ? "diag-stat-val badge-ok" : "diag-stat-val badge-err";
  }

  // 5. Sync Fault Injection Buttons from Telemetry if provided
  if (t.active_faults) {
    for (const k in t.active_faults) {
      activeFaults[k] = Boolean(t.active_faults[k]);
      const btn = document.getElementById(faultBtnIds[k]);
      if (btn) {
        if (activeFaults[k]) {
          btn.classList.add("fault-active");
        } else {
          btn.classList.remove("fault-active");
        }
      }
    }
  }

  // 6. Sound Alarms Trigger Logic
  // Single fault -> audible ping; multiple faults -> warning chime & motor limiting
  const activeAlarmsList = t.active_alarms || [];
  const pCount = t.problem_count !== undefined ? t.problem_count : activeAlarmsList.length;
  const nowMs = Date.now();

  if (pCount >= 2) {
    // Multiple problems arose: warning chime (rate-limited every 2.5s)
    if (nowMs - lastAudioAlertTime > 2500) {
      lastAudioAlertTime = nowMs;
      playMultiProblemChime();
    }
  } else if (pCount === 1) {
    // Single problem arose: audible ping alert (rate-limited every 2.0s)
    if (nowMs - lastAudioAlertTime > 2000) {
      lastAudioAlertTime = nowMs;
      playPingAlert();
    }
  }
  lastAlertProblemCount = pCount;

  // 7. Update Chart Buffers & Canvas Redraw
  coastSlopeMarkers = t.slope_markers || [];
  appendChartData(t.rpm || 0, t.i_ma || 0, t.vib_g || 0, t.temp_c || 25.0);

  renderChartSpeed();
  renderChartElectVib();
  renderChartTemp();
}

function appendChartData(rpm, current, vibration, temp) {
  chartBuffers.rpm.push(rpm);
  chartBuffers.current.push(current);
  chartBuffers.vibration.push(vibration * 400.0); // Scaled for dual-trace visibility
  chartBuffers.temp.push(temp);

  if (chartBuffers.rpm.length > MAX_POINTS) {
    chartBuffers.rpm.shift();
    chartBuffers.current.shift();
    chartBuffers.vibration.shift();
    chartBuffers.temp.shift();
  }
}

// Data Staleness & Packet Rate Monitor (runs at 2 Hz)
setInterval(() => {
  const now = Date.now();
  const dtSec = (now - lastTelemetryTime) / 1000.0;
  const stalenessElem = document.getElementById("statStaleness");
  const readoutContainer = document.getElementById("readoutContainer");

  if (dtSec > 1.2) {
    if (stalenessElem) {
      stalenessElem.textContent = `STALE (${dtSec.toFixed(1)}s)`;
      stalenessElem.className = "badge-status badge-crit";
    }
    if (readoutContainer) {
      readoutContainer.classList.add("readout-stale");
    }
  } else {
    if (stalenessElem) {
      stalenessElem.textContent = `ONLINE (${dtSec.toFixed(1)}s)`;
      stalenessElem.className = "badge-status badge-ok";
    }
    if (readoutContainer) {
      readoutContainer.classList.remove("readout-stale");
    }
  }

  // Packet rate calculation
  const elapsedSinceRateCheck = (now - lastRateCheckTime) / 1000.0;
  if (elapsedSinceRateCheck >= 1.0) {
    const rate = lastPacketCount / elapsedSinceRateCheck;
    lastPacketCount = 0;
    lastRateCheckTime = now;
    const rateElem = document.getElementById("diagPktRate");
    if (rateElem) rateElem.textContent = `${rate.toFixed(1)} Hz`;
  }
}, 500);

// =============================================================================
// ALARM LEDGER LOG AUDIT TRAIL
// =============================================================================
async function pollAlarms() {
  try {
    const res = await fetch("/api/logs");
    const data = await res.json();
    if (data.logs) {
      renderAlarmLedger(data.logs);
    }
  } catch (e) {}
}

function renderAlarmLedger(logs) {
  const tbody = document.getElementById("alarmTbody");
  tbody.innerHTML = "";

  logs.forEach(ev => {
    const row = document.createElement("tr");

    const tdTime = document.createElement("td");
    tdTime.textContent = ev.timestamp;
    row.appendChild(tdTime);

    const tdSev = document.createElement("td");
    tdSev.textContent = ev.severity;
    tdSev.className = `severity-${ev.severity}`;
    tdSev.style.textAlign = "center";
    row.appendChild(tdSev);

    const tdParam = document.createElement("td");
    tdParam.textContent = ev.parameter;
    row.appendChild(tdParam);

    const tdCond = document.createElement("td");
    tdCond.textContent = ev.condition;
    row.appendChild(tdCond);

    tbody.appendChild(row);
  });
}

// =============================================================================
// HTML5 CANVAS STRIP CHART RENDERING ENGINE
// =============================================================================
let canvasSpeed, ctxSpeed, canvasElect, ctxElect, canvasTemp, ctxTemp;

function initCanvases() {
  canvasSpeed = document.getElementById("chartSpeed");
  ctxSpeed = canvasSpeed.getContext("2d");
  canvasElect = document.getElementById("chartElectVib");
  ctxElect = canvasElect.getContext("2d");
  canvasTemp = document.getElementById("chartTemp");
  ctxTemp = canvasTemp ? canvasTemp.getContext("2d") : null;

  resizeCanvases();
  window.addEventListener("resize", resizeCanvases);
}

function resizeCanvases() {
  if (canvasSpeed) {
    const rect1 = canvasSpeed.getBoundingClientRect();
    canvasSpeed.width = rect1.width;
    canvasSpeed.height = 135;
  }

  if (canvasElect) {
    const rect2 = canvasElect.getBoundingClientRect();
    canvasElect.width = rect2.width;
    canvasElect.height = 135;
  }

  if (canvasTemp) {
    const rect3 = canvasTemp.getBoundingClientRect();
    canvasTemp.width = rect3.width;
    canvasTemp.height = 110;
  }

  renderChartSpeed();
  renderChartElectVib();
  renderChartTemp();
}

function drawGrid(ctx, w, h, yMin, yMax, unit) {
  ctx.fillStyle = "#0F0F0F";
  ctx.fillRect(0, 0, w, h);

  const padLeft = 45;
  const padRight = 10;
  const padTop = 15;
  const padBottom = 20;
  const plotW = w - padLeft - padRight;
  const plotH = h - padTop - padBottom;

  ctx.strokeStyle = "#2D2D2D";
  ctx.lineWidth = 1;
  ctx.setLineDash([2, 4]);

  // Horizontal ticks
  const ticks = 4;
  for (let i = 0; i <= ticks; i++) {
    const yVal = yMin + (i / ticks) * (yMax - yMin);
    const py = padTop + plotH - (i / ticks) * plotH;

    ctx.beginPath();
    ctx.moveTo(padLeft, py);
    ctx.lineTo(padLeft + plotW, py);
    ctx.stroke();

    ctx.fillStyle = "#8A8A8A";
    ctx.font = "9px Consolas, monospace";
    ctx.textAlign = "right";
    ctx.fillText(yVal.toFixed(0), padLeft - 6, py + 3);
  }

  // Vertical time ticks
  for (let j = 1; j <= 5; j++) {
    const px = padLeft + (j / 6) * plotW;
    ctx.beginPath();
    ctx.moveTo(px, padTop);
    ctx.lineTo(px, padTop + plotH);
    ctx.stroke();
  }

  ctx.setLineDash([]);
  return { padLeft, padRight, padTop, padBottom, plotW, plotH };
}

function renderChartSpeed() {
  if (!ctxSpeed || !canvasSpeed) return;
  const w = canvasSpeed.width;
  const h = canvasSpeed.height;
  const yMin = 0;
  const yMax = 2500;

  const { padLeft, padTop, plotW, plotH } = drawGrid(ctxSpeed, w, h, yMin, yMax, "RPM");

  const buf = chartBuffers.rpm;
  if (buf.length < 2) return;

  const dx = plotW / (MAX_POINTS - 1);
  const xOffset = padLeft + (MAX_POINTS - buf.length) * dx;

  // Draw Speed Trace
  ctxSpeed.strokeStyle = "#00E5FF";
  ctxSpeed.lineWidth = 2;
  ctxSpeed.beginPath();

  for (let i = 0; i < buf.length; i++) {
    const px = xOffset + i * dx;
    const py = padTop + plotH - (Math.max(0, Math.min(yMax, buf[i])) / (yMax - yMin)) * plotH;
    if (i === 0) ctxSpeed.moveTo(px, py);
    else ctxSpeed.lineTo(px, py);
  }
  ctxSpeed.stroke();

  // Draw Deceleration Slope Annotation Markers if test active
  if (coastSlopeMarkers.length >= 2) {
    ctxSpeed.strokeStyle = "#FF5252";
    ctxSpeed.lineWidth = 2;
    ctxSpeed.setLineDash([4, 2]);

    const p1x = padLeft + plotW - 45;
    const p2x = padLeft + plotW - 10;
    const p1y = padTop + 20;
    const p2y = padTop + plotH - 10;

    ctxSpeed.beginPath();
    ctxSpeed.moveTo(p1x, p1y);
    ctxSpeed.lineTo(p2x, p2y);
    ctxSpeed.stroke();
    ctxSpeed.setLineDash([]);

    ctxSpeed.fillStyle = "#FF5252";
    ctxSpeed.font = "9px Consolas, monospace";
    ctxSpeed.textAlign = "right";
    ctxSpeed.fillText("dω/dt FIT", p1x - 6, p1y + 12);
  }
}

function renderChartElectVib() {
  if (!ctxElect || !canvasElect) return;
  const w = canvasElect.width;
  const h = canvasElect.height;
  const yMin = 0;
  const yMax = 1000;

  const { padLeft, padTop, plotW, plotH } = drawGrid(ctxElect, w, h, yMin, yMax, "mA");

  const curBuf = chartBuffers.current;
  const vibBuf = chartBuffers.vibration;
  if (curBuf.length < 2) return;

  const dx = plotW / (MAX_POINTS - 1);
  const xOffset = padLeft + (MAX_POINTS - curBuf.length) * dx;

  // 1. Draw Current Trace (#FFB300)
  ctxElect.strokeStyle = "#FFB300";
  ctxElect.lineWidth = 1.5;
  ctxElect.beginPath();
  for (let i = 0; i < curBuf.length; i++) {
    const px = xOffset + i * dx;
    const py = padTop + plotH - (Math.max(0, Math.min(yMax, curBuf[i])) / (yMax - yMin)) * plotH;
    if (i === 0) ctxElect.moveTo(px, py);
    else ctxElect.lineTo(px, py);
  }
  ctxElect.stroke();

  // 2. Draw Scaled Vibration Trace (#E040FB)
  ctxElect.strokeStyle = "#E040FB";
  ctxElect.lineWidth = 1.5;
  ctxElect.beginPath();
  for (let j = 0; j < vibBuf.length; j++) {
    const px = xOffset + j * dx;
    const py = padTop + plotH - (Math.max(0, Math.min(yMax, vibBuf[j])) / (yMax - yMin)) * plotH;
    if (j === 0) ctxElect.moveTo(px, py);
    else ctxElect.lineTo(px, py);
  }
  ctxElect.stroke();
}

function renderChartTemp() {
  if (!ctxTemp || !canvasTemp) return;
  const w = canvasTemp.width;
  const h = canvasTemp.height;
  const yMin = 0;
  const yMax = 80;

  const { padLeft, padTop, plotW, plotH } = drawGrid(ctxTemp, w, h, yMin, yMax, "°C");

  const tempBuf = chartBuffers.temp;
  if (tempBuf.length < 2) return;

  const dx = plotW / (MAX_POINTS - 1);
  const xOffset = padLeft + (MAX_POINTS - tempBuf.length) * dx;

  // Draw Temperature Trace (#FF7043)
  ctxTemp.strokeStyle = "#FF7043";
  ctxTemp.lineWidth = 2;
  ctxTemp.beginPath();
  for (let k = 0; k < tempBuf.length; k++) {
    const px = xOffset + k * dx;
    const py = padTop + plotH - (Math.max(0, Math.min(yMax, tempBuf[k])) / (yMax - yMin)) * plotH;
    if (k === 0) ctxTemp.moveTo(px, py);
    else ctxTemp.lineTo(px, py);
  }
  ctxTemp.stroke();
}
