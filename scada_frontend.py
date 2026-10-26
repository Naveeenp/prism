#!/usr/bin/env python3
"""
================================================================================
INDUSTRIAL SCADA HUMAN-MACHINE INTERFACE (HMI) - PMDC MOTOR DIAGNOSTICS
================================================================================
Compliance: ISA-101 High-Contrast Industrial Ergonomics
Color Palette:
  - Matte Charcoal Background : #1E1E1E
  - Industrial Grey Cards     : #2A2A2A
  - Subtle Borders            : #3D3D3D
  - Display Fields            : #141414
  - Normal / OK Accent        : #4CAF50
  - Caution / Advisory        : #FFC107
  - Critical / Trip Alert     : #F44336
  - Control Accent            : #0288D1
Typography: Monospaced Consolas / JetBrains Mono for High-Precision Readouts
Zero Glassmorphism | Zero Blur | Zero AI Jargon
================================================================================
"""

import sys
import time
import math
import threading
import collections
from typing import List, Tuple, Dict, Any, Optional


try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    TKINTER_AVAILABLE = True
except ImportError:
    TKINTER_AVAILABLE = False

from backend_engine import SCADABackendEngine, AlarmSeverity



# ==============================================================================
# ISA-101 COLOR & FONT DESIGN TOKENS
# ==============================================================================
BG_DARK = "#1E1E1E"        # Primary App Background
BG_PANEL = "#2A2A2A"       # Panel Cards
BG_FIELD = "#141414"       # Digital Readout Inset
BORDER_COL = "#3D3D3D"     # Strict Industrial Border
TEXT_PRIMARY = "#E0E0E0"   # High-contrast readable off-white
TEXT_MUTED = "#8A8A8A"     # Secondary labels and units
TEXT_EMPHASIS = "#FFFFFF"  # Primary engineering readouts

# Engineering State Colors (ISA-101 Standardized)
COLOR_NORMAL = "#4CAF50"    # Green
COLOR_CAUTION = "#FFC107"   # Amber
COLOR_CRITICAL = "#F44336"  # Red
COLOR_CYAN = "#00E5FF"      # Speed Plot Trace
COLOR_AMBER = "#FFB300"     # Current Plot Trace
COLOR_MAGENTA = "#E040FB"   # Vibration Plot Trace
COLOR_GRID = "#2D2D2D"      # Chart Gridlines

FONT_TITLE = ("Consolas", 11, "bold")
FONT_HEADER = ("Consolas", 10, "bold")
FONT_LABEL = ("Segoe UI", 9)
FONT_VALUE_LG = ("Consolas", 22, "bold")
FONT_VALUE_MD = ("Consolas", 16, "bold")
FONT_UNIT = ("Consolas", 9, "normal")
FONT_BUTTON = ("Segoe UI", 9, "bold")
FONT_MONO_SM = ("Consolas", 8)


# ==============================================================================
# REAL-TIME 2D STRIP-CHART CANVAS (ISA-101 HIGH-CONTRAST VECTOR PLOT)
# ==============================================================================
class RealTimeStripChart(tk.Canvas):
    """
    High-performance, lightweight 2D vector strip-chart:
    - High-contrast matte canvas with subtle engineering grid
    - Multi-channel synchronized line traces with min/max autoscaling
    - Deceleration slope markers overlay during coast-down testing
    - Zero flickering via double-buffered canvas redraws
    """
    def __init__(self, parent, title: str, channels: List[Dict[str, Any]], y_min: float = 0.0, y_max: float = 100.0, max_points: int = 120, **kwargs):
        super().__init__(parent, bg=BG_FIELD, highlightthickness=1, highlightbackground=BORDER_COL, **kwargs)
        self.title = title
        self.channels = channels  # [{"name": "RPM", "color": "#00E5FF", "unit": "RPM"}]
        self.y_min_fixed = y_min
        self.y_max_fixed = y_max
        self.max_points = max_points

        # Deque for history buffers: {channel_name: deque of values}
        self.data_buffers = {ch["name"]: collections.deque(maxlen=max_points) for ch in channels}
        self.time_buffer = collections.deque(maxlen=max_points)
        self.slope_markers: List[Tuple[float, float]] = []

        self.bind("<Configure>", lambda e: self.redraw())

    def append_data(self, timestamp: float, channel_values: Dict[str, float]):
        self.time_buffer.append(timestamp)
        for ch in self.channels:
            val = channel_values.get(ch["name"], 0.0)
            self.data_buffers[ch["name"]].append(val)

    def set_slope_markers(self, markers: List[Tuple[float, float]]):
        self.slope_markers = list(markers)

    def redraw(self):
        self.delete("all")
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 50 or h < 50:
            return

        pad_left = 55
        pad_right = 20
        pad_top = 28
        pad_bottom = 26
        plot_w = w - pad_left - pad_right
        plot_h = h - pad_top - pad_bottom

        # 1. Background Inset
        self.create_rectangle(pad_left, pad_top, pad_left + plot_w, pad_top + plot_h, fill="#0F0F0F", outline=BORDER_COL)

        # 2. Determine Y-Scale Dynamic Bounds
        all_vals = []
        for ch in self.channels:
            all_vals.extend(list(self.data_buffers[ch["name"]]))

        if all_vals:
            cur_min = min(all_vals)
            cur_max = max(all_vals)
            y_min = min(self.y_min_fixed, cur_min)
            y_max = max(self.y_max_fixed, cur_max * 1.08 + 1.0)
        else:
            y_min = self.y_min_fixed
            y_max = self.y_max_fixed

        y_range = max(0.001, y_max - y_min)

        # 3. Horizontal Engineering Gridlines (5 Ticks)
        ticks = 4
        for i in range(ticks + 1):
            y_val = y_min + (i / ticks) * y_range
            py = pad_top + plot_h - (i / ticks) * plot_h
            # Gridline
            self.create_line(pad_left, py, pad_left + plot_w, py, fill=COLOR_GRID, dash=(2, 4))
            # Label
            val_str = f"{y_val:.0f}" if y_max >= 10 else f"{y_val:.2f}"
            self.create_text(pad_left - 6, py, text=val_str, fill=TEXT_MUTED, font=FONT_MONO_SM, anchor="e")

        # 4. Vertical Gridlines
        for x_step in range(1, 6):
            px = pad_left + (x_step / 6) * plot_w
            self.create_line(px, pad_top, px, pad_top + plot_h, fill=COLOR_GRID, dash=(1, 5))

        # 5. Title & Legend in Header
        legend_x = pad_left
        self.create_text(legend_x, 14, text=self.title.upper(), fill=TEXT_EMPHASIS, font=FONT_HEADER, anchor="w")

        legend_x += len(self.title) * 8 + 30
        for ch in self.channels:
            # Color badge
            self.create_rectangle(legend_x, 10, legend_x + 12, 18, fill=ch["color"], outline="")
            latest_val = self.data_buffers[ch["name"]][-1] if self.data_buffers[ch["name"]] else 0.0
            label_text = f"{ch['name']}: {latest_val:.1f} {ch['unit']}"
            self.create_text(legend_x + 16, 14, text=label_text, fill=TEXT_PRIMARY, font=FONT_MONO_SM, anchor="w")
            legend_x += len(label_text) * 7 + 25

        # 6. Plot Data Traces
        n_pts = len(self.time_buffer)
        if n_pts >= 2:
            dx = plot_w / (self.max_points - 1)
            x_offset = pad_left + (self.max_points - n_pts) * dx

            for ch in self.channels:
                buf = list(self.data_buffers[ch["name"]])
                coords = []
                for idx, v in enumerate(buf):
                    px = x_offset + idx * dx
                    py = pad_top + plot_h - ((v - y_min) / y_range) * plot_h
                    coords.extend([px, py])

                if len(coords) >= 4:
                    self.create_line(coords, fill=ch["color"], width=2, smooth=False)

        # 7. Draw Deceleration Slope Marker if present
        if len(self.slope_markers) >= 2 and n_pts >= 2:
            m1, m2 = self.slope_markers[0], self.slope_markers[1]
            # Draw an annotation line over the deceleration region
            p1_y = pad_top + plot_h - ((m1[1] - y_min) / y_range) * plot_h
            p2_y = pad_top + plot_h - ((m2[1] - y_min) / y_range) * plot_h
            p1_x = pad_left + plot_w - 40
            p2_x = pad_left + plot_w - 10
            self.create_line(p1_x, p1_y, p2_x, p2_y, fill="#FF5252", width=3, dash=(4, 2))
            self.create_text(p1_x - 10, p1_y - 8, text="SLOPE: dω/dt", fill="#FF5252", font=FONT_MONO_SM, anchor="e")


# ==============================================================================
# MAIN INDUSTRIAL SCADA APPLICATION WINDOW
# ==============================================================================
class IndustrialSCADAApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DC Motor Diagnostic SCADA Workstation - ESP32 Backend [ISA-101]")
        self.geometry("1240x820")
        self.minsize(1080, 720)
        self.configure(bg=BG_DARK)

        # Instantiate Backend Engine
        self.backend = SCADABackendEngine()

        # Heartbeat state
        self.heartbeat_state = False

        # Apply Global Tkinter Styles
        self._setup_styles()

        # Build SCADA Layout
        self._build_top_status_bar()
        self._build_main_workspace()

        # Schedule Real-Time Refresh Loop (10 Hz)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(100, self._ui_update_loop)

    def _setup_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")

        # General Frames
        style.configure("TFrame", background=BG_DARK)
        style.configure("Panel.TFrame", background=BG_PANEL, relief="solid", borderwidth=1)

        # Combobox
        style.configure("TCombobox", fieldbackground=BG_FIELD, background=BG_PANEL, foreground=TEXT_PRIMARY, bordercolor=BORDER_COL)

        # Treeview (Alarm Ledger)
        style.configure("Treeview",
                        background=BG_FIELD,
                        foreground=TEXT_PRIMARY,
                        fieldbackground=BG_FIELD,
                        borderwidth=0,
                        rowheight=22,
                        font=FONT_MONO_SM)
        style.configure("Treeview.Heading",
                        background=BG_PANEL,
                        foreground=TEXT_PRIMARY,
                        font=FONT_TITLE,
                        relief="flat")
        style.map("Treeview", background=[("selected", "#004D40")])

    # --------------------------------------------------------------------------
    # 1. TOP STATUS BAR (ISA-101 COMPLIANT HEADER)
    # --------------------------------------------------------------------------
    def _build_top_status_bar(self):
        bar = tk.Frame(self, bg=BG_PANEL, height=54, highlightthickness=1, highlightbackground=BORDER_COL)
        bar.pack(side="top", fill="x", padx=8, pady=(8, 4))
        bar.pack_propagate(False)

        # Title & System ID
        title_box = tk.Frame(bar, bg=BG_PANEL)
        title_box.pack(side="left", padx=12, pady=8)

        lbl_title = tk.Label(title_box, text="PMDC HEALTH SCADA", font=("Consolas", 12, "bold"), fg=TEXT_EMPHASIS, bg=BG_PANEL)
        lbl_title.pack(side="top", anchor="w")
        lbl_sub = tk.Label(title_box, text="ESP-32S DUAL-CORE TELEMETRY BENCH", font=("Segoe UI", 8), fg=TEXT_MUTED, bg=BG_PANEL)
        lbl_sub.pack(side="top", anchor="w")

        # Serial Port Configuration Controls
        port_box = tk.Frame(bar, bg=BG_PANEL)
        port_box.pack(side="left", padx=16)

        tk.Label(port_box, text="PORT:", font=FONT_LABEL, fg=TEXT_MUTED, bg=BG_PANEL).pack(side="left", padx=4)
        self.port_combo = ttk.Combobox(port_box, width=12, state="readonly")
        self.port_combo.pack(side="left", padx=4)
        self._refresh_serial_ports()

        btn_rescan = tk.Button(port_box, text="SCAN", font=FONT_BUTTON, bg=BG_PANEL, fg=TEXT_PRIMARY,
                               activebackground=BORDER_COL, activeforeground=TEXT_EMPHASIS,
                               relief="solid", bd=1, padx=6, command=self._refresh_serial_ports)
        btn_rescan.pack(side="left", padx=4)

        tk.Label(port_box, text="BAUD:", font=FONT_LABEL, fg=TEXT_MUTED, bg=BG_PANEL).pack(side="left", padx=(10, 4))
        self.baud_combo = ttk.Combobox(port_box, values=["115200", "57600", "9600"], width=8, state="readonly")
        self.baud_combo.set("115200")
        self.baud_combo.pack(side="left", padx=4)

        # Connect / Disconnect Toggle Button
        self.btn_connect = tk.Button(bar, text="CONNECT", font=FONT_BUTTON, bg="#2E7D32", fg="#FFFFFF",
                                     activebackground="#1B5E20", relief="solid", bd=1, padx=12,
                                     command=self._toggle_connection)
        self.btn_connect.pack(side="left", padx=12)

        # Heartbeat Indicator LED
        hb_frame = tk.Frame(bar, bg=BG_PANEL)
        hb_frame.pack(side="left", padx=16)
        self.hb_canvas = tk.Canvas(hb_frame, width=16, height=16, bg=BG_PANEL, highlightthickness=0)
        self.hb_canvas.pack(side="left", padx=4)
        self.hb_led = self.hb_canvas.create_oval(2, 2, 14, 14, fill="#424242", outline=BORDER_COL)
        self.lbl_hb_text = tk.Label(hb_frame, text="HEARTBEAT", font=FONT_MONO_SM, fg=TEXT_MUTED, bg=BG_PANEL)
        self.lbl_hb_text.pack(side="left")

        # Emergency Stop (E-STOP) Button (Prominent Industrial Red)
        btn_estop = tk.Button(bar, text="EMERGENCY STOP (E-STOP)", font=("Consolas", 11, "bold"),
                              bg="#D32F2F", fg="#FFFFFF", activebackground="#B71C1C", activeforeground="#FFFFFF",
                              relief="solid", bd=2, padx=18, pady=4, cursor="hand2",
                              command=self._on_estop_clicked)
        btn_estop.pack(side="right", padx=12)

    def _refresh_serial_ports(self):
        ports = self.backend.get_serial_ports()
        port_names = [p["device"] for p in ports]
        if not port_names:
            port_names = ["COM3 (Simulated)", "COM4", "COM5"]
        self.port_combo["values"] = port_names
        if port_names:
            self.port_combo.current(0)

    def _toggle_connection(self):
        if not self.backend.serial_worker.is_connected:
            selected_port = self.port_combo.get().split()[0]
            try:
                baud = int(self.baud_combo.get())
            except ValueError:
                baud = 115200

            success, msg = self.backend.serial_worker.connect(selected_port, baud)
            if success:
                self.btn_connect.config(text="DISCONNECT", bg="#C62828")
                self.backend._add_alarm(AlarmSeverity.NORMAL, "SERIAL", f"Connected to ESP32 on {selected_port}")
            else:
                messagebox.showwarning("Serial Connection Failed", f"Could not open {selected_port}:\n{msg}\n\nFalling back to Simulation Engine.")
        else:
            self.backend.serial_worker.disconnect()
            self.btn_connect.config(text="CONNECT", bg="#2E7D32")
            self.backend._add_alarm(AlarmSeverity.CAUTION, "SERIAL", "ESP32 Serial interface disconnected")

    def _on_estop_clicked(self):
        self.backend.emergency_stop()

    def _toggle_driver_power(self):
        current_state = bool(self.backend.latest_telemetry.get("driver_pwr", False))
        new_state = not current_state
        self.backend.set_driver_power(new_state)
        self._update_driver_power_button(new_state)

    def _update_driver_power_button(self, is_enabled: bool):
        if is_enabled:
            self.btn_driver_pwr.config(text="DISABLE MOTOR DRIVER (POWER OFF)", bg="#C62828", activebackground="#B71C1C")
        else:
            self.btn_driver_pwr.config(text="ENABLE MOTOR DRIVER (ARM POWER)", bg="#2E7D32", activebackground="#1B5E20")

    # --------------------------------------------------------------------------
    # 2. MAIN WORKSPACE ARCHITECTURE
    # --------------------------------------------------------------------------
    def _build_main_workspace(self):
        main_pane = tk.PanedWindow(self, orient="horizontal", bg=BG_DARK, sashrelief="flat", sashwidth=4)
        main_pane.pack(fill="both", expand=True, padx=8, pady=4)

        # Left Column: Process Readouts + Controls
        left_col = tk.Frame(main_pane, bg=BG_DARK, width=440)
        left_col.pack_propagate(False)
        main_pane.add(left_col)

        self._build_operating_controls(left_col)
        self._build_digital_readouts(left_col)

        # Right Column: Real-Time Strip Charts + Alarm Ledger
        right_col = tk.Frame(main_pane, bg=BG_DARK)
        main_pane.add(right_col)

        self._build_strip_charts(right_col)
        self._build_alarm_ledger(right_col)

    # --------------------------------------------------------------------------
    # 3. OPERATING CONTROL PANEL
    # --------------------------------------------------------------------------
    def _build_operating_controls(self, parent):
        card = tk.LabelFrame(parent, text=" 2. OPERATING CONTROL PANEL ", font=FONT_TITLE, fg=TEXT_PRIMARY,
                             bg=BG_PANEL, bd=1, relief="solid")
        card.pack(fill="x", pady=(0, 6), ipady=4)

        # Row 0: Driver Master Arm / Power Toggle
        pwr_frame = tk.Frame(card, bg=BG_FIELD, highlightthickness=1, highlightbackground=BORDER_COL)
        pwr_frame.pack(fill="x", padx=10, pady=(4, 8), ipady=3)

        lbl_pwr_head = tk.Label(pwr_frame, text="L298N DRIVER MASTER POWER:", font=FONT_LABEL, fg=TEXT_MUTED, bg=BG_FIELD)
        lbl_pwr_head.pack(side="left", padx=8)

        self.btn_driver_pwr = tk.Button(
            pwr_frame, text="ENABLE MOTOR DRIVER", font=FONT_BUTTON,
            bg="#2E7D32", fg="#FFFFFF", activebackground="#1B5E20",
            relief="solid", bd=1, padx=10, pady=2,
            command=self._toggle_driver_power
        )
        self.btn_driver_pwr.pack(side="right", padx=8)

        # Row 1: Target Speed Direct Presets
        lbl_presets = tk.Label(card, text="TARGET SPEED PRESETS (LEDC PWM):", font=FONT_LABEL, fg=TEXT_MUTED, bg=BG_PANEL)
        lbl_presets.pack(anchor="w", padx=10, pady=(4, 2))

        btn_row = tk.Frame(card, bg=BG_PANEL)
        btn_row.pack(fill="x", padx=10, pady=2)

        speeds = [
            ("IDLE / 0%", 0.0, "#424242"),
            ("RUN 50%", 50.0, "#1565C0"),
            ("RUN 75%", 75.0, "#1565C0"),
            ("RUN 100%", 100.0, "#0D47A1")
        ]
        for text, spd, col in speeds:
            b = tk.Button(btn_row, text=text, font=FONT_BUTTON, bg=col, fg="#FFFFFF",
                          activebackground="#455A64", relief="solid", bd=1, padx=6, pady=4,
                          command=lambda s=spd: self.backend.set_target_speed(s))
            b.pack(side="left", fill="x", expand=True, padx=2)

        # Row 2: Automated Diagnostic Triggers
        lbl_diag = tk.Label(card, text="AUTOMATED DIAGNOSTIC ROUTINES:", font=FONT_LABEL, fg=TEXT_MUTED, bg=BG_PANEL)
        lbl_diag.pack(anchor="w", padx=10, pady=(8, 2))

        diag_row = tk.Frame(card, bg=BG_PANEL)
        diag_row.pack(fill="x", padx=10, pady=2)

        btn_ra = tk.Button(diag_row, text="[ MEASURE Ra ]", font=FONT_BUTTON, bg="#00695C", fg="#FFFFFF",
                           activebackground="#004D40", relief="solid", bd=1, pady=5,
                           command=self.backend.trigger_ra_measurement)
        btn_ra.pack(side="left", fill="x", expand=True, padx=2)

        btn_coast = tk.Button(diag_row, text="[ START COAST-DOWN TEST ]", font=FONT_BUTTON, bg="#4E342E", fg="#FFFFFF",
                              activebackground="#3E2723", relief="solid", bd=1, pady=5,
                              command=self.backend.trigger_coast_down_test)
        btn_coast.pack(side="left", fill="x", expand=True, padx=2)

    # --------------------------------------------------------------------------
    # 4. DIGITAL PROCESS VALUES (ENGINEERING READOUT CARDS)
    # --------------------------------------------------------------------------
    def _build_digital_readouts(self, parent):
        card = tk.LabelFrame(parent, text=" 3. DIGITAL PROCESS VALUES ", font=FONT_TITLE, fg=TEXT_PRIMARY,
                             bg=BG_PANEL, bd=1, relief="solid")
        card.pack(fill="both", expand=True, pady=(0, 4))

        # Grid of Process Metric Blocks
        self.readout_widgets = {}

        metrics = [
            ("V_BUS", "VOLTAGE (Vt)", "12.04", "V", 0, 0),
            ("I_MA", "ARMATURE CURRENT (Ia)", "320.5", "mA", 0, 1),
            ("RPM", "SHAFT VELOCITY (ω)", "1820.0", "RPM", 1, 0),
            ("TEMP", "CASING TEMPERATURE", "31.4", "°C", 1, 1),
            ("VIB", "RMS VIBRATION", "0.18", "G", 2, 0),
            ("RA", "CALCULATED Ra", "2.40", "Ω", 2, 1),
            ("COAST", "COAST-DOWN TIME", "2.20", "sec", 3, 0),
            ("BCI", "BEARING CONDITION (BCI)", "100.0", "%", 3, 1),
        ]

        card.grid_columnconfigure(0, weight=1)
        card.grid_columnconfigure(1, weight=1)

        for key, label, default_val, unit, r, c in metrics:
            cell = tk.Frame(card, bg=BG_FIELD, highlightthickness=1, highlightbackground=BORDER_COL)
            cell.grid(row=r, column=c, padx=6, pady=4, sticky="nsew")

            lbl_name = tk.Label(cell, text=label, font=FONT_MONO_SM, fg=TEXT_MUTED, bg=BG_FIELD)
            lbl_name.pack(anchor="w", padx=6, pady=(4, 0))

            val_box = tk.Frame(cell, bg=BG_FIELD)
            val_box.pack(anchor="w", padx=6, pady=(0, 4))

            lbl_val = tk.Label(val_box, text=default_val, font=FONT_VALUE_LG, fg=TEXT_EMPHASIS, bg=BG_FIELD)
            lbl_val.pack(side="left")

            lbl_u = tk.Label(val_box, text=f" {unit}", font=FONT_UNIT, fg=TEXT_MUTED, bg=BG_FIELD)
            lbl_u.pack(side="left", pady=(8, 0))

            self.readout_widgets[key] = (lbl_val, lbl_name)

    # --------------------------------------------------------------------------
    # 5. STRIP-CHART TRENDS (REAL-TIME 2D LINE PLOTS)
    # --------------------------------------------------------------------------
    def _build_strip_charts(self, parent):
        chart_container = tk.Frame(parent, bg=BG_DARK)
        chart_container.pack(side="top", fill="both", expand=True, pady=(0, 4))

        # Plot 1: Shaft Speed (RPM) vs. Time with Deceleration Slope Markers
        self.plot_speed = RealTimeStripChart(
            chart_container,
            title="Plot 1: Shaft Speed (RPM) vs Time [Coast-Down Slope]",
            channels=[{"name": "RPM", "color": COLOR_CYAN, "unit": "RPM"}],
            y_min=0.0,
            y_max=2500.0,
            height=180
        )
        self.plot_speed.pack(side="top", fill="both", expand=True, pady=(0, 4))

        # Plot 2: Armature Current (mA) and Vibration (G) Dual-Trace vs Time
        self.plot_electrical_vib = RealTimeStripChart(
            chart_container,
            title="Plot 2: Current (mA) & Vibration (G) vs Time",
            channels=[
                {"name": "Current", "color": COLOR_AMBER, "unit": "mA"},
                {"name": "Vibration", "color": COLOR_MAGENTA, "unit": "G"}
            ],
            y_min=0.0,
            y_max=1000.0,
            height=180
        )
        self.plot_electrical_vib.pack(side="top", fill="both", expand=True)

    # --------------------------------------------------------------------------
    # 6. ALARM & EVENT LEDGER (ISA-101 CHRONOLOGICAL AUDIT TRAIL)
    # --------------------------------------------------------------------------
    def _build_alarm_ledger(self, parent):
        card = tk.LabelFrame(parent, text=" 5. THRESHOLD ALARM & EVENT LEDGER ", font=FONT_TITLE, fg=TEXT_PRIMARY,
                             bg=BG_PANEL, bd=1, relief="solid", height=190)
        card.pack(side="bottom", fill="x", pady=(4, 0))
        card.pack_propagate(False)

        columns = ("timestamp", "severity", "parameter", "condition")
        self.tree_alarms = ttk.Treeview(card, columns=columns, show="headings", height=6)
        self.tree_alarms.heading("timestamp", text="TIMESTAMP", anchor="w")
        self.tree_alarms.heading("severity", text="SEVERITY", anchor="center")
        self.tree_alarms.heading("parameter", text="PARAMETER", anchor="w")
        self.tree_alarms.heading("condition", text="TRIP CONDITION / STATUS", anchor="w")

        self.tree_alarms.column("timestamp", width=100, stretch=False)
        self.tree_alarms.column("severity", width=100, stretch=False, anchor="center")
        self.tree_alarms.column("parameter", width=160, stretch=False)
        self.tree_alarms.column("condition", width=400, stretch=True)

        scrollbar = ttk.Scrollbar(card, orient="vertical", command=self.tree_alarms.yview)
        self.tree_alarms.configure(yscrollcommand=scrollbar.set)

        self.tree_alarms.pack(side="left", fill="both", expand=True, padx=(4, 0), pady=4)
        scrollbar.pack(side="right", fill="y", padx=(0, 4), pady=4)

        # Style tags for severity coloring
        self.tree_alarms.tag_configure(AlarmSeverity.NORMAL, foreground=COLOR_NORMAL)
        self.tree_alarms.tag_configure(AlarmSeverity.CAUTION, foreground=COLOR_CAUTION)
        self.tree_alarms.tag_configure(AlarmSeverity.CRITICAL, foreground=COLOR_CRITICAL)

        self.last_alarm_count = 0

    # --------------------------------------------------------------------------
    # REAL-TIME UI REFRESH CYCLE (DETERMINISTIC 10 HZ DISPATCH)
    # --------------------------------------------------------------------------
    def _ui_update_loop(self):
        try:
            # 1. Fetch Latest Telemetry Snapshot
            t = dict(self.backend.latest_telemetry)
            now_sec = time.time()

            # 2. Update Digital Metric Readouts
            self.readout_widgets["V_BUS"][0].config(text=f"{t.get('v_bus', 0.0):.2f}")
            self.readout_widgets["I_MA"][0].config(text=f"{t.get('i_ma', 0.0):.1f}")
            self.readout_widgets["RPM"][0].config(text=f"{t.get('rpm', 0.0):.1f}")
            self.readout_widgets["TEMP"][0].config(text=f"{t.get('temp_c', 0.0):.1f}")
            self.readout_widgets["VIB"][0].config(text=f"{t.get('vib_g', 0.0):.3f}")
            self.readout_widgets["RA"][0].config(text=f"{t.get('ra_ohms', 2.40):.2f}")
            self.readout_widgets["COAST"][0].config(text=f"{t.get('coast_duration_s', 0.0):.2f}")
            self.readout_widgets["BCI"][0].config(text=f"{t.get('bci_pct', 100.0):.1f}")

            # Highlight severity colors on critical metrics
            status = t.get("status", AlarmSeverity.NORMAL)
            if status == AlarmSeverity.CRITICAL:
                self.readout_widgets["BCI"][0].config(fg=COLOR_CRITICAL)
            elif status == AlarmSeverity.CAUTION:
                self.readout_widgets["BCI"][0].config(fg=COLOR_CAUTION)
            else:
                self.readout_widgets["BCI"][0].config(fg=COLOR_NORMAL)

            # Sync Driver Power Button UI
            driver_pwr = bool(t.get("driver_pwr", False))
            self._update_driver_power_button(driver_pwr)

            # 3. Stream to Strip Charts
            self.plot_speed.append_data(now_sec, {"RPM": t.get("rpm", 0.0)})
            self.plot_speed.set_slope_markers(self.backend.coast_analyzer.slope_markers)
            self.plot_speed.redraw()

            self.plot_electrical_vib.append_data(now_sec, {
                "Current": t.get("i_ma", 0.0),
                "Vibration": t.get("vib_g", 0.0) * 400.0 # Scale vibration for visible dual-trace
            })
            self.plot_electrical_vib.redraw()

            # 4. Refresh Alarm Ledger Table
            current_alarms = list(self.backend.alarm_ledger)
            if len(current_alarms) != self.last_alarm_count:
                self.last_alarm_count = len(current_alarms)
                # Repopulate most recent alarms (top first)
                for item in self.tree_alarms.get_children():
                    self.tree_alarms.delete(item)

                for ev in reversed(current_alarms[-50:]):
                    self.tree_alarms.insert(
                        "",
                        "end",
                        values=(ev.timestamp, ev.severity, ev.parameter, ev.condition),
                        tags=(ev.severity,)
                    )

            # 5. Heartbeat LED Toggle
            self.heartbeat_state = not self.heartbeat_state
            hb_color = "#00E676" if self.heartbeat_state else "#1B5E20"
            self.hb_canvas.itemconfig(self.hb_led, fill=hb_color)

        except Exception as e:
            print(f"UI update exception: {e}")

        # Reschedule next 100ms cycle
        self.after(100, self._ui_update_loop)

    def _on_close(self):
        self.backend.shutdown()
        self.destroy()


# ==============================================================================
# MAIN ENTRY POINT
# ==============================================================================
if __name__ == "__main__":
    if not TKINTER_AVAILABLE:
        print("\n" + "=" * 70)
        print(" [!] NOTICE: Python 'tkinter' module is not installed in this environment.")
        print(" [*] Automatically launching Web SCADA Dashboard instead at http://localhost:8080...")
        print("=" * 70 + "\n")
        import webbrowser
        from dashboard.server import start_server
        threading.Thread(target=lambda: (time.sleep(1.0), webbrowser.open("http://localhost:8080")), daemon=True).start()
        start_server(host="0.0.0.0", port=8080)
    else:
        app = IndustrialSCADAApp()
        app.mainloop()

