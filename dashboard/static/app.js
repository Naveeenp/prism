/**
 * ==============================================================================
 * DC MOTOR HEALTH MONITOR - CLIENT APPLICATION CONTROLLER (app.js)
 * Clean, modern, uncluttered UI logic with Web Audio alerts and fault lab.
 * ==============================================================================
 */

// Global State
let currentSource = "SIMULATION"; // "SIMULATION" or "HARDWARE"
let isPicoConnected = false;
let eventSource = null;
let lastPacketTime = performance.now();
let packetCounter = 0;
let packetsThisSec = 0;
let lastRateCheck = performance.now();

// Active Injected Faults Cache
let activeFaults = {
  overheat: false,
  overcurrent: false,
  vibration: false,
  stall: false,
  bearing_drag: false,
  voltage_drop: false
};

// Web Audio API Alert Engine
let audioContext = null;
let isAudioMuted = false;
let lastAlertChimeTime = 0;

// Automated Test Sequence State
let testCycleActive = false;
let testCycleTimer = null;
let testCycleStep = 0;
const TEST_STEPS = [
  { speed: 0, timeMs: 3000 },
  { speed: 30, timeMs: 4000 },
  { speed: 70, timeMs: 5000 },
  { speed: 100, timeMs: 4000 },
  { speed: 0, timeMs: 4500 }
];

// Rolling History for Strip Charts (120 samples ~ 12 seconds)
const CHART_CAPACITY = 120;
const telemetryHistory = {
  rpm: [],
  current: [],
  vib: [],
  temp: []
};

// ==============================================================================
// 1. INITIALIZATION & LIFECYCLE
// ==============================================================================
document.addEventListener("DOMContentLoaded", () => {
  // Load saved mute preference
  const savedMute = localStorage.getItem("scada_audio_muted");
  if (savedMute === "true") {
    isAudioMuted = true;
    updateAudioButtonUI();
  }

  // Initialize Canvas DPI scaling
  initCanvasScaling();
  window.addEventListener("resize", initCanvasScaling);

  // Scan COM ports
  refreshSerialPorts();

  // Connect to live telemetry stream (SSE)
  connectTelemetryStream();

  // Start continuous 60fps chart rendering loop
  requestAnimationFrame(drawChartsLoop);
});

// ==============================================================================
// 2. WEB AUDIO ALERT ENGINE
// ==============================================================================
function initAudio() {
  if (!audioContext) {
    const AudioCtx = window.AudioContext || window.webkitAudioContext;
    if (AudioCtx) {
      audioContext = new AudioCtx();
    }
  }
  if (audioContext && audioContext.state === "suspended") {
    audioContext.resume();
  }
}

function playWarningChime() {
  if (isAudioMuted) return;
  initAudio();
  if (!audioContext) return;

  const now = audioContext.currentTime;

  // Synthesize clean two-tone alert chime (880 Hz -> 587 Hz)
  const osc = audioContext.createOscillator();
  const gain = audioContext.createGain();

  osc.type = "sine";
  osc.frequency.setValueAtTime(880, now); // A5 note
  osc.frequency.setValueAtTime(587.33, now + 0.15); // D5 note

  gain.gain.setValueAtTime(0.001, now);
  gain.gain.exponentialRampToValueAtTime(0.25, now + 0.03);
  gain.gain.exponentialRampToValueAtTime(0.001, now + 0.45);

  osc.connect(gain);
  gain.connect(audioContext.destination);

  osc.start(now);
  osc.stop(now + 0.46);
}

function toggleAudioAlerts() {
  initAudio();
  isAudioMuted = !isAudioMuted;
  localStorage.setItem("scada_audio_muted", isAudioMuted);
  updateAudioButtonUI();

  if (!isAudioMuted) {
    playWarningChime(); // Test beep on unmute
  }
}

function updateAudioButtonUI() {
  const btn = document.getElementById("btnAudioToggle");
  const icon = document.getElementById("audioIcon");
  const text = document.getElementById("audioText");

  if (isAudioMuted) {
    btn.classList.add("muted");
    icon.textContent = "🔇";
    text.textContent = "Alert Audio: MUTED";
  } else {
    btn.classList.remove("muted");
    icon.textContent = "🔊";
    text.textContent = "Alert Audio: ON";
  }
}

// ==============================================================================
// 3. SERVER-SENT EVENTS (SSE) STREAM
// ==============================================================================
function connectTelemetryStream() {
  if (eventSource) {
    eventSource.close();
  }

  eventSource = new EventSource("/api/telemetry/stream");

  eventSource.onmessage = (event) => {
    try {
      const telem = JSON.parse(event.data);
      handleTelemetryPacket(telem);
    } catch (err) {
      console.error("Packet parse error:", err);
    }
  };

  eventSource.onerror = () => {
    document.getElementById("statusBadge").textContent = "RECONNECTING";
  };
}

// ==============================================================================
// 4. TELEMETRY PROCESSING & PLAIN-ENGLISH WARNING RENDERING
// ==============================================================================
function handleTelemetryPacket(data) {
  packetCounter++;
  packetsThisSec++;

  const nowPerf = performance.now();
  if (nowPerf - lastRateCheck >= 1000) {
    const rate = (packetsThisSec / ((nowPerf - lastRateCheck) / 1000)).toFixed(0);
    document.getElementById("valPacketRate").textContent = `${rate} Hz`;
    packetsThisSec = 0;
    lastRateCheck = nowPerf;
  }

  // 1. Update Primary Metrics
  document.getElementById("valRpm").textContent = Math.round(data.rpm).toLocaleString();
  document.getElementById("valPwm").textContent = `${Math.round(data.pwm_pct)}%`;
  
  const omega = (data.rpm * (2 * Math.PI / 60)).toFixed(1);
  document.getElementById("valOmega").textContent = omega;

  document.getElementById("valCurrent").textContent = Math.round(data.curr_ma);
  const powerW = (data.power_mw / 1000.0).toFixed(1);
  document.getElementById("valPower").textContent = `${powerW} W`;

  document.getElementById("valTemp").textContent = data.temp_c.toFixed(1);
  document.getElementById("valVib").textContent = data.vib_rms_g.toFixed(2);

  // Secondary metrics
  document.getElementById("valVoltage").textContent = `${data.volts.toFixed(2)} V`;
  const sag = (12.0 - data.volts).toFixed(2);
  document.getElementById("valSag").textContent = `${sag > 0 ? sag : "0.00"} V`;
  document.getElementById("valDecel").textContent = `${data.decel_rad_s2.toFixed(1)} rad/s²`;

  // 2. Human-Readable Health Status & Warnings
  updateHealthStatusAndWarnings(data);

  // 3. Sync Injected Fault Button States
  if (data.faults_injected) {
    syncFaultButtonStyles(data.faults_injected);
  }

  // 4. Append to Rolling Chart Buffers
  telemetryHistory.rpm.push(data.rpm);
  telemetryHistory.current.push(data.curr_ma);
  telemetryHistory.vib.push(data.vib_rms_g);
  telemetryHistory.temp.push(data.temp_c);

  if (telemetryHistory.rpm.length > CHART_CAPACITY) {
    telemetryHistory.rpm.shift();
    telemetryHistory.current.shift();
    telemetryHistory.vib.shift();
    telemetryHistory.temp.shift();
  }
}

function updateHealthStatusAndWarnings(data) {
  const banner = document.getElementById("statusBanner");
  const badge = document.getElementById("statusBadge");
  const summary = document.getElementById("statusSummary");
  const warningsContainer = document.getElementById("warningItemsContainer");

  const warnings = data.warnings || [];
  const hasCritical = warnings.some(w => w.severity === "CRITICAL");
  const hasWarning = warnings.length > 0 && !hasCritical;

  // Card Alert Borders
  const alarms = data.alarms || {};
  document.getElementById("cardTemp").classList.toggle("alert-state", alarms.overtemp);
  document.getElementById("cardCurrent").classList.toggle("alert-state", alarms.overcurrent);
  document.getElementById("cardVib").classList.toggle("alert-state", alarms.vibration);
  document.getElementById("cardSpeed").classList.toggle("alert-state", alarms.stall);

  if (hasCritical) {
    banner.className = "status-banner state-alarm";
    badge.textContent = "CRITICAL ALERT";
    summary.textContent = "Immediate attention required: Critical threshold exceeded.";
    warningsContainer.style.display = "flex";

    // Trigger Audible Alert Chime (throttled every 3.5 seconds)
    const now = performance.now();
    if (now - lastAlertChimeTime >= 3500) {
      playWarningChime();
      lastAlertChimeTime = now;
    }
  } else if (hasWarning) {
    banner.className = "status-banner state-warning";
    badge.textContent = "WARNING";
    summary.textContent = "Motor operating with elevated parameters.";
    warningsContainer.style.display = "flex";
  } else {
    banner.className = "status-banner";
    badge.textContent = "HEALTHY";
    summary.textContent = "All systems operating normally within nominal safety thresholds.";
    warningsContainer.style.display = "none";
    warningsContainer.innerHTML = "";
    return;
  }

  // Render clear plain-English warning cards
  warningsContainer.innerHTML = "";
  warnings.forEach(w => {
    const card = document.createElement("div");
    card.className = `warning-card ${w.severity.toLowerCase()}`;

    const icon = w.severity === "CRITICAL" ? "🛑" : "⚠️";
    card.innerHTML = `
      <div class="warning-icon">${icon}</div>
      <div class="warning-text">
        <div class="warning-title">${w.title}</div>
        <div class="warning-desc">${w.message}</div>
      </div>
    `;
    warningsContainer.appendChild(card);
  });
}

// ==============================================================================
// 5. INTERACTIVE SIMULATION FAULT INJECTION LAB
// ==============================================================================
function toggleFault(faultName) {
  activeFaults[faultName] = !activeFaults[faultName];

  fetch("/api/simulation/faults", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ [faultName]: activeFaults[faultName] })
  })
  .then(res => res.json())
  .then(() => {
    updateFaultButtonState(faultName, activeFaults[faultName]);
  })
  .catch(err => console.error("Fault toggle error:", err));
}

function resetAllFaults() {
  fetch("/api/simulation/reset", { method: "POST" })
    .then(res => res.json())
    .then(() => {
      for (const k in activeFaults) {
        activeFaults[k] = false;
        updateFaultButtonState(k, false);
      }
    });
}

function updateFaultButtonState(faultName, isActive) {
  const map = {
    overheat: "btnFaultOverheat",
    overcurrent: "btnFaultOvercurrent",
    vibration: "btnFaultVib",
    stall: "btnFaultStall",
    bearing_drag: "btnFaultBearing",
    voltage_drop: "btnFaultVoltage"
  };
  const btnId = map[faultName];
  if (btnId) {
    const el = document.getElementById(btnId);
    if (el) el.classList.toggle("active", Boolean(isActive));
  }
}

function syncFaultButtonStyles(faultsObj) {
  for (const k in faultsObj) {
    activeFaults[k] = Boolean(faultsObj[k]);
    updateFaultButtonState(k, activeFaults[k]);
  }
}

// ==============================================================================
// 6. SOURCE SWITCHER & HARDWARE SERIAL
// ==============================================================================
function selectDataSource(source) {
  currentSource = source;
  document.getElementById("tabSim").classList.toggle("active", source === "SIMULATION");
  document.getElementById("tabHw").classList.toggle("active", source === "HARDWARE");

  const drawer = document.getElementById("hwDrawer");
  if (source === "HARDWARE") {
    drawer.style.display = "block";
    refreshSerialPorts();
  } else {
    drawer.style.display = "none";
  }

  fetch("/api/mode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode: source })
  })
  .then(res => res.json())
  .then(resp => {
    if (source === "HARDWARE") {
      isPicoConnected = resp.success;
      updatePicoConnectButton();
    }
  });
}

function refreshSerialPorts() {
  fetch("/api/ports")
    .then(res => res.json())
    .then(data => {
      const select = document.getElementById("comPortSelect");
      select.innerHTML = "";
      if (data.ports && data.ports.length > 0) {
        data.ports.forEach(p => {
          const opt = document.createElement("option");
          opt.value = p.device;
          opt.textContent = `${p.device} (${p.description || "Serial Device"})`;
          if (p.device === data.current_port) opt.selected = true;
          select.appendChild(opt);
        });
      } else {
        const opt = document.createElement("option");
        opt.value = "COM3";
        opt.textContent = "COM3 (Raspberry Pi Pico)";
        select.appendChild(opt);
      }
    });
}

function toggleHardwareConnection() {
  const port = document.getElementById("comPortSelect").value;
  const baud = document.getElementById("baudSelect").value;

  if (isPicoConnected) {
    fetch("/api/serial/disconnect", { method: "POST" })
      .then(res => res.json())
      .then(() => {
        isPicoConnected = false;
        updatePicoConnectButton();
      });
  } else {
    fetch("/api/serial/connect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ port: port, baudrate: parseInt(baud) })
    })
    .then(res => res.json())
    .then(resp => {
      isPicoConnected = resp.success;
      updatePicoConnectButton();
    });
  }
}

function updatePicoConnectButton() {
  const btn = document.getElementById("btnConnect");
  const pill = document.getElementById("connStatusPill");

  if (isPicoConnected) {
    btn.textContent = "Disconnect Pico";
    btn.classList.add("connected");
    pill.textContent = "Connected (Online)";
    pill.className = "connection-status-pill online";
  } else {
    btn.textContent = "Connect to Pico";
    btn.classList.remove("connected");
    pill.textContent = "Offline";
    pill.className = "connection-status-pill";
  }
}

// ==============================================================================
// 7. MOTOR DRIVE CONTROLLER
// ==============================================================================
function updateSliderBadge(val) {
  document.getElementById("sliderValBadge").textContent = `${val}%`;
}

function applySpeed(speed) {
  document.getElementById("speedSlider").value = speed;
  updateSliderBadge(speed);

  fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "set_speed", speed: parseFloat(speed) })
  });
}

function triggerEmergencyStop() {
  if (testCycleActive) toggleTestCycle();

  document.getElementById("speedSlider").value = 0;
  updateSliderBadge(0);

  fetch("/api/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "stop" })
  });
}

function toggleTestCycle() {
  testCycleActive = !testCycleActive;
  const btn = document.getElementById("btnTestCycle");

  if (testCycleActive) {
    btn.textContent = "Halt Test Cycle";
    btn.classList.add("active");
    testCycleStep = 0;
    runNextTestStep();
  } else {
    btn.textContent = "Start Automated Test Cycle";
    btn.classList.remove("active");
    if (testCycleTimer) clearTimeout(testCycleTimer);
  }
}

function runNextTestStep() {
  if (!testCycleActive) return;

  const step = TEST_STEPS[testCycleStep];
  applySpeed(step.speed);

  testCycleStep = (testCycleStep + 1) % TEST_STEPS.length;
  testCycleTimer = setTimeout(runNextTestStep, step.timeMs);
}

// ==============================================================================
// 8. 60FPS DUAL CANVAS STRIP CHARTS
// ==============================================================================
let chartCanvases = {};
let chartContexts = {};

function initCanvasScaling() {
  const ids = ["canvasRpm", "canvasCurrent", "canvasVib", "canvasTemp"];
  const dpr = window.devicePixelRatio || 1;

  ids.forEach(id => {
    const cvs = document.getElementById(id);
    if (!cvs) return;
    const rect = cvs.getBoundingClientRect();
    cvs.width = rect.width * dpr;
    cvs.height = rect.height * dpr;
    const ctx = cvs.getContext("2d");
    ctx.scale(dpr, dpr);
    chartCanvases[id] = cvs;
    chartContexts[id] = ctx;
  });
}

function drawChartsLoop() {
  drawStrip("canvasRpm", telemetryHistory.rpm, 0, 3200, "#38bdf8", null);
  drawStrip("canvasCurrent", telemetryHistory.current, 0, 1200, "#f59e0b", 900);
  drawStrip("canvasVib", telemetryHistory.vib, 0, 3.0, "#fb923c", 1.80);
  drawStrip("canvasTemp", telemetryHistory.temp, 20, 65, "#f87171", 50.0);

  requestAnimationFrame(drawChartsLoop);
}

function drawStrip(canvasId, series, minY, maxY, color, thresholdVal) {
  const cvs = chartCanvases[canvasId];
  const ctx = chartContexts[canvasId];
  if (!cvs || !ctx) return;

  const w = cvs.getBoundingClientRect().width;
  const h = cvs.getBoundingClientRect().height;

  // Clear background
  ctx.fillStyle = "#090d16";
  ctx.fillRect(0, 0, w, h);

  // Subtle grid lines
  ctx.strokeStyle = "#162133";
  ctx.lineWidth = 1;
  const divs = 3;
  for (let i = 0; i <= divs; i++) {
    const y = (h / divs) * i;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  // Draw Alarm Threshold Line if set
  if (thresholdVal !== null) {
    const threshY = h - ((thresholdVal - minY) / (maxY - minY)) * h;
    ctx.strokeStyle = "rgba(239, 68, 68, 0.6)";
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(0, threshY);
    ctx.lineTo(w, threshY);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  if (series.length < 2) return;

  // Draw Line
  ctx.strokeStyle = color;
  ctx.lineWidth = 2.0;
  ctx.beginPath();

  const stepX = w / (CHART_CAPACITY - 1);
  const offset = CHART_CAPACITY - series.length;

  for (let i = 0; i < series.length; i++) {
    const x = (offset + i) * stepX;
    const clamped = Math.max(minY, Math.min(maxY, series[i]));
    const y = h - ((clamped - minY) / (maxY - minY)) * h;

    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}
