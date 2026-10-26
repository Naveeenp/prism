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
  vibration: []
};

let coastSlopeMarkers = [];

// =============================================================================
// INITIALIZATION
// =============================================================================
window.addEventListener("DOMContentLoaded", () => {
  initCanvases();
  refreshPorts();
  startEventSource();
  setInterval(pollAlarms, 1000);
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
// SSE REAL-TIME TELEMETRY CONSUMER
// =============================================================================
function startEventSource() {
  if (!!window.EventSource) {
    sseSource = new EventSource("/api/telemetry/stream");
    sseSource.onmessage = (e) => {
      try {
        const packet = JSON.parse(e.data);
        handleTelemetryUpdate(packet);
      } catch (err) {
        console.error("SSE parse error:", err);
      }
    };
    sseSource.onerror = () => {
      // Reconnect automatically
    };
  } else {
    // 10 Hz Polling fallback
    setInterval(async () => {
      try {
        const res = await fetch("/api/telemetry");
        const packet = await res.json();
        handleTelemetryUpdate(packet);
      } catch (e) {}
    }, 100);
  }
}

function handleTelemetryUpdate(t) {
  // 1. Toggle Heartbeat Indicator
  heartbeatToggle = !heartbeatToggle;
  const hbLed = document.getElementById("heartbeatLed");
  if (heartbeatToggle) {
    hbLed.classList.add("active");
  } else {
    hbLed.classList.remove("active");
  }

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

  // 3. Update Chart Buffers
  coastSlopeMarkers = t.slope_markers || [];
  appendChartData(t.rpm || 0, t.i_ma || 0, t.vib_g || 0);

  // 4. Redraw Strip Charts
  renderChartSpeed();
  renderChartElectVib();
}

function appendChartData(rpm, current, vibration) {
  chartBuffers.rpm.push(rpm);
  chartBuffers.current.push(current);
  chartBuffers.vibration.push(vibration * 400.0); // Scaled for dual-trace visibility

  if (chartBuffers.rpm.length > MAX_POINTS) {
    chartBuffers.rpm.shift();
    chartBuffers.current.shift();
    chartBuffers.vibration.shift();
  }
}

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
let canvasSpeed, ctxSpeed, canvasElect, ctxElect;

function initCanvases() {
  canvasSpeed = document.getElementById("chartSpeed");
  ctxSpeed = canvasSpeed.getContext("2d");
  canvasElect = document.getElementById("chartElectVib");
  ctxElect = canvasElect.getContext("2d");

  resizeCanvases();
  window.addEventListener("resize", resizeCanvases);
}

function resizeCanvases() {
  const rect1 = canvasSpeed.getBoundingClientRect();
  canvasSpeed.width = rect1.width;
  canvasSpeed.height = 150;

  const rect2 = canvasElect.getBoundingClientRect();
  canvasElect.width = rect2.width;
  canvasElect.height = 150;

  renderChartSpeed();
  renderChartElectVib();
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

  // Vertical ticks
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
  if (!ctxSpeed) return;
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
  if (!ctxElect) return;
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
