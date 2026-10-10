"""SDLView: lightweight live viewer for Suzuki SDL ECUs (built for Swift G10A, ECU ID 1453).

Run:
    python sdlview.py        (pick the COM port, then Connect)

Protocol and formulas: see README.md. Read-only (0x10, 0x13), except the
idle command (0x15 C0 <target>), which is always closed with a stop frame.
"""

import argparse
import csv
import ctypes
import math
import queue
import statistics
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports

BAUD = 7812
ADDRS = list(range(0x30))  # 0x00-0x2F respond; max 37 addresses per request
CHUNK = 24
STOP_FRAME = bytes(8)
IDLE_MIN, IDLE_MAX = 600, 1600
AUTO_STOP_HIGH, AUTO_STOP_LOW = 2500, 300

# Colour palette (dark)
C = {
    "bg": "#0f1115",        # window
    "surface": "#171a21",   # panels
    "tile": "#1d212a",      # dashboard tiles
    "field": "#232833",     # inputs
    "border": "#2c3240",
    "text": "#e6e8ee",
    "muted": "#8a93a3",
    "dim": "#5c6473",
    "accent": "#4ea1ff",
    "accent_dim": "#2b5f99",
    "ok": "#3ecf8e",
    "warn": "#f5b14c",
    "danger": "#ff5c5c",
    "changed": "#3b3420",
    "grid": "#262b36",
}

# Line colours for the graph channels
CHANNEL_COLOURS = ["#4ea1ff", "#3ecf8e", "#f5b14c", "#d982ff"]

# Fault codes of ECU 1453: (byte, bit) -> (code, description).
# Byte 0/1 = addresses 0x00/0x01 (current); 0x02/0x03 (stored) use the same bit layout.
# Bit 5 of byte 0 (code 23) was verified on the car by unplugging the IAT sensor.
FAULTS_1453 = {
    (0, 0): (13, "Oxygen (lambda) sensor: no or slow response"),
    (0, 1): (14, "Coolant temperature sensor: signal too high (open circuit)"),
    (0, 2): (15, "Coolant temperature sensor: signal too low (short to ground)"),
    (0, 3): (21, "Throttle position sensor: signal too high"),
    (0, 4): (22, "Throttle position sensor: signal too low"),
    (0, 5): (23, "Intake air temperature sensor: signal too high (open circuit)"),
    (0, 6): (24, "Vehicle speed sensor: no signal"),
    (0, 7): (25, "Intake air temperature sensor: signal too low (short to ground)"),
    (1, 0): (31, "MAP sensor: signal too high"),
    (1, 1): (32, "MAP sensor: signal too low"),
    (1, 2): (41, "Ignition: fail-safe signal missing"),
    (1, 3): (42, "Crank angle sensor: no signal while cranking"),
    (1, 4): (51, "EGR system malfunction"),
}


def app_dir() -> Path:
    return Path(sys.executable).parent if getattr(sys, "frozen", False) else Path(__file__).parent


def resource(name: str) -> Path | None:
    """File bundled in the exe (PyInstaller _MEIPASS) or next to the script in assets/."""
    for path in (Path(getattr(sys, "_MEIPASS", app_dir())) / name, app_dir() / "assets" / name):
        if path.exists():
            return path
    return None


def set_window_icon(root: tk.Tk) -> None:
    try:
        # own taskbar group/icon instead of the generic Python one when run as a script
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("SDLView.SDLView")
    except Exception:  # noqa: BLE001
        pass
    try:
        ico = resource("sdlview.ico")
        if ico:
            root.iconbitmap(default=str(ico))  # crisp small sizes in the title bar
        png = resource("sdlview.png")
        if png:
            root._icon_img = tk.PhotoImage(file=str(png))  # keep a reference
            root.iconphoto(True, root._icon_img)
    except tk.TclError:
        pass


def checksum(body: bytes) -> int:
    return (-sum(body)) & 0xFF


# ---------------------------------------------------------------- link

class SDLPort:
    """Half-duplex SDL over a KKL/FTDI cable: every byte sent is echoed back first."""

    def __init__(self, port: str):
        self.ser = serial.Serial(port, BAUD, bytesize=8, parity="N", stopbits=1, timeout=0.3, write_timeout=1)

    def close(self) -> None:
        self.ser.close()

    def request(self, header: int, data: bytes = b"") -> tuple[str, list[int], str]:
        body = bytes([header, 3 + len(data)]) + data
        msg = body + bytes([checksum(body)])
        self.ser.reset_input_buffer()
        self.ser.write(msg)
        echo = self.ser.read(len(msg))
        head = self.ser.read(2)
        rest = self.ser.read(head[1] - 2) if len(head) == 2 and head[1] >= 3 else b""
        raw = head + rest
        if echo != msg:
            status = "no echo"
        elif not raw:
            status = "no response"
        elif len(raw) < 3 or raw[0] != header or len(raw) != raw[1] or sum(raw) & 0xFF:
            status = "invalid response"
        else:
            return "ok", list(raw[2:-1]), raw.hex(" ")
        time.sleep(0.3)  # give the ECU time and let the line settle
        self.ser.reset_input_buffer()
        return status, [], raw.hex(" ")


class Link(threading.Thread):
    """Background thread: polls all addresses continuously and executes control commands.

    Messages to the GUI (out queue):
        ("connected", ecu_id), ("sample", t, {addr: value}), ("ack", text), ("error", text),
        ("idle_state", target_rpm_or_None, stop_pending), ("closed", text, is_error, stop_failed)
    Commands from the GUI (cmds queue): ("idle", rpm) or ("stop",).

    Safety: as soon as an idle frame has been written, the target is treated as possibly active
    (even if the ack was lost). It is only cleared by an acknowledged stop frame. While a stop is
    pending it is retried every poll cycle, and on halt the stop is sent first (up to 3 tries).
    The thread is not a daemon, so closing the window waits for it.
    """

    def __init__(self, out: queue.Queue, port: str):
        super().__init__(daemon=False)
        self.out, self.port = out, port
        self.cmds: queue.Queue = queue.Queue()
        self.halt = threading.Event()
        self.target: int | None = None   # idle target that may be active in the ECU
        self.stop_pending = False

    def run(self) -> None:
        try:
            self._run_serial()
        except Exception as exc:  # noqa: BLE001 - report everything to the GUI
            self.out.put(("closed", f"Error: {exc}", True, False))

    def _publish_idle(self) -> None:
        self.out.put(("idle_state", self.target, self.stop_pending))

    def _send_stop(self, sdl: "SDLPort", label: str) -> bool:
        status, _, rx = sdl.request(0x15, STOP_FRAME)
        self.out.put(("ack", f"{label}: {status} (rx {rx})"))
        if status == "ok":
            self.target, self.stop_pending = None, False
        else:
            self.stop_pending = True
        self._publish_idle()
        return status == "ok"

    def _run_serial(self) -> None:
        try:
            sdl = SDLPort(self.port)
        except serial.SerialException as exc:
            text = str(exc)
            if "PermissionError" in text or "denied" in text.lower() or "geweigerd" in text:
                msg = f"{self.port} is busy (is another diagnostic program open?)"
            elif "FileNotFoundError" in text:
                msg = f"{self.port} not found (is the cable plugged in?)"
            else:
                msg = f"Cannot open {self.port}: {exc}"
            self.out.put(("closed", msg, True, False))
            return
        closing_msg, closing_error = "Disconnected", False
        try:
            status, data, _ = sdl.request(0x10)
            if status != "ok":
                closing_msg, closing_error = f"No response from ECU ({status}). Ignition on?", True
                return
            self.out.put(("connected", bytes(data).hex().upper()))
            failures = 0
            while not self.halt.is_set():
                self._handle_commands(sdl)
                if self.stop_pending and not self.halt.is_set():
                    self._send_stop(sdl, "STOP (retry)")
                values = {}
                for i in range(0, len(ADDRS), CHUNK):
                    if self.halt.is_set():
                        break
                    part = ADDRS[i:i + CHUNK]
                    status, data, _ = sdl.request(0x13, bytes(part))
                    if status == "ok":
                        values.update(zip(part, data))
                if len(values) == len(ADDRS):
                    failures = 0
                    self.out.put(("sample", time.time(), values))
                elif not self.halt.is_set():
                    failures += 1
                    if failures >= 10:
                        self.out.put(("error", "ECU stopped responding (ignition off or cable loose?)"))
                        failures = 0
        except Exception as exc:  # noqa: BLE001 - e.g. cable unplugged mid-session
            closing_msg, closing_error = f"Connection lost: {exc}", True
        finally:
            stop_failed = False
            if self.target is not None or self.stop_pending:
                ok = False
                for attempt in range(3):
                    try:
                        if self._send_stop(sdl, f"STOP on close ({attempt + 1}/3)"):
                            ok = True
                            break
                    except Exception:  # noqa: BLE001
                        time.sleep(0.3)
                stop_failed = not ok
            try:
                sdl.close()
            except Exception:  # noqa: BLE001
                pass
            self.out.put(("closed", closing_msg, closing_error, stop_failed))

    def _handle_commands(self, sdl: "SDLPort") -> None:
        while not self.halt.is_set():
            try:
                cmd = self.cmds.get_nowait()
            except queue.Empty:
                return
            if cmd[0] == "idle":
                frame = bytes([0xC0, round(cmd[1] / 7.84375), 0, 0, 0, 0, 0, 0])
                # From here on the target may be active in the ECU, even if the ack gets lost.
                self.target = cmd[1]
                status, _, rx = sdl.request(0x15, frame)
                self.out.put(("ack", f"Target {cmd[1]} rpm: {status} (rx {rx})"))
                self._publish_idle()
            else:
                self._send_stop(sdl, "STOP")



# ---------------------------------------------------------------- conversions

def temp_c(raw: int) -> float:
    return raw / 255 * 159 - 40


def pressure_mbar(raw: int) -> float:
    return raw / 255 * 1666.3 - 200


def derive(v: dict, history: deque) -> dict:
    """Converted values. history = recent (t, values) for the RPM median and the full injector pulse."""
    rpms = [((h[0x04] & 0x3F) * 256 + h[0x05]) / 5.1 for _, h in history if h[0x04] & 0xC0 == 0]
    injs = [((h[0x0D] << 8) | h[0x0E]) * 0.002 for _, h in history]
    return {
        "rpm": statistics.median(rpms) if rpms else None,  # None = no valid sample in the window
        "target_idle": math.floor(v[0x06] * 7.84375),
        "vss": v[0x07],
        "ect": temp_c(v[0x08]),
        "iat": temp_c(v[0x09]),
        "map": pressure_mbar(v[0x10]),
        "baro": pressure_mbar(v[0x11]),
        "tps_v": v[0x0B] / 255 * 5,
        "tps_angle": v[0x0A],
        "isc": math.floor(v[0x12] / 255 * 100),
        "inj": max(injs) if injs else 0.0,
        "adv": v[0x0F] / 255 * 90 - 12,
        "adv_raw": v[0x0F],
        "batt": v[0x16] * 20 / 256,
        "ctp": bool(v[0x18] & 0x10),
        "fan": v[0x19] == 128,
        "mix": (v[0x13], v[0x14], v[0x15]),
    }


def decode_faults(table: dict, byte_index: int, value: int) -> list[tuple[int | None, str]]:
    """byte_index 0/1 = current (0x00/0x01), 2/3 = stored (0x02/0x03, same bit layout as 0/1)."""
    out = []
    for bit in range(8):
        if value & (1 << bit):
            hit = table.get((byte_index, bit)) or table.get((byte_index % 2, bit))
            out.append(hit or (None, f"unknown (byte {byte_index}, bit {bit})"))
    return out


# ---------------------------------------------------------------- GUI

TILES = [  # key, title, unit, format, unconfirmed formula
    ("rpm", "Engine speed", "rpm", "{:.0f}", False),
    ("target_idle", "Target idle", "rpm", "{:.0f}", False),
    ("ect", "Coolant temp", "°C", "{:.0f}", False),
    ("iat", "Intake air temp", "°C", "{:.0f}", False),
    ("map", "MAP", "mbar", "{:.0f}", True),
    ("baro", "Barometric", "mbar", "{:.0f}", True),
    ("tps_v", "Throttle", "V", "{:.2f}", False),
    ("tps_angle", "Throttle angle", "raw", "{:.0f}", False),
    ("isc", "ISC duty", "%", "{:.0f}", False),
    ("inj", "Injector pulse", "ms", "{:.2f}", False),
    ("adv", "Ignition advance", "° (raw)", "{:.0f}", True),
    ("batt", "Battery", "V", "{:.2f}", False),
    ("ctp", "Closed throttle", "CTP switch", "{}", False),
    ("fan", "Radiator fan", "", "{}", False),
    ("vss", "Vehicle speed", "km/h", "{:.0f}", False),
    ("mix", "Mixture control", "0x13 / 0x14 / 0x15", "{}", True),
]


# Graph signals: name -> (unit, default range, getter(derived, raw))
SIGNALS: dict[str, tuple] = {
    "Engine speed": ("rpm", (0, 3000), lambda d, v: d["rpm"]),
    "Target idle": ("rpm", (0, 2000), lambda d, v: d["target_idle"]),
    "Coolant temp": ("°C", (-20, 120), lambda d, v: d["ect"]),
    "Intake air temp": ("°C", (-20, 60), lambda d, v: d["iat"]),
    "MAP": ("mbar", (0, 1100), lambda d, v: d["map"]),
    "Barometric": ("mbar", (900, 1100), lambda d, v: d["baro"]),
    "Throttle": ("V", (0, 5), lambda d, v: d["tps_v"]),
    "Throttle angle": ("raw", (0, 160), lambda d, v: d["tps_angle"]),
    "ISC duty": ("%", (0, 100), lambda d, v: d["isc"]),
    "Injector pulse": ("ms", (0, 20), lambda d, v: d["inj"]),
    "Ignition advance": ("°", (-15, 45), lambda d, v: d["adv"]),
    "Battery": ("V", (8, 16), lambda d, v: d["batt"]),
    "Closed throttle": ("on/off", (0, 1), lambda d, v: 1.0 if d["ctp"] else 0.0),
    "Radiator fan": ("on/off", (0, 1), lambda d, v: 1.0 if d["fan"] else 0.0),
    "Vehicle speed": ("km/h", (0, 150), lambda d, v: d["vss"]),
    "Mixture 0x13": ("raw", (0, 255), lambda d, v: v[0x13]),
    "Mixture 0x14": ("raw", (0, 255), lambda d, v: v[0x14]),
    "Mixture 0x15": ("raw", (0, 255), lambda d, v: v[0x15]),
}
for _a in ADDRS:
    SIGNALS[f"Raw 0x{_a:02X}"] = ("raw", (0, 255), lambda d, v, a=_a: v[a])
NO_SIGNAL = "—"
DEFAULT_CHANNELS = ["Engine speed", "ISC duty", "Injector pulse", "Coolant temp"]
WINDOWS = {"10 s": 10, "30 s": 30, "1 min": 60, "2 min": 120, "5 min": 300}


class GraphTab:
    """Four stacked live line charts, each with a selectable signal."""

    def __init__(self, nb: ttk.Notebook):
        self.points: deque = deque()  # (t, {signal: value})
        self.paused_at: float | None = None
        outer = ttk.Frame(nb, padding=6)
        nb.add(outer, text="Graph")
        self.frame = outer

        side = ttk.Frame(outer, padding=(4, 4, 10, 4))
        side.pack(side="left", fill="y")
        self.channels: list[tk.StringVar] = []
        self.value_labels: list[tk.Label] = []
        names = [NO_SIGNAL] + list(SIGNALS)
        for i in range(4):
            box = ttk.LabelFrame(side, text=f"Channel {i + 1}", padding=6)
            box.pack(fill="x", pady=4)
            var = tk.StringVar(value=DEFAULT_CHANNELS[i])
            combo = ttk.Combobox(box, textvariable=var, values=names, state="readonly", width=18)
            combo.pack(fill="x")
            combo.bind("<<ComboboxSelected>>", lambda _e: self.redraw())
            label = tk.Label(box, text="--", fg=CHANNEL_COLOURS[i], bg=C["surface"], font=("Segoe UI", 16, "bold"))
            label.pack(anchor="w", pady=(4, 0))
            self.channels.append(var)
            self.value_labels.append(label)

        opts = ttk.LabelFrame(side, text="View", padding=6)
        opts.pack(fill="x", pady=(10, 4))
        ttk.Label(opts, text="Time window").pack(anchor="w")
        self.window = tk.StringVar(value="1 min")
        win = ttk.Combobox(opts, textvariable=self.window, values=list(WINDOWS), state="readonly", width=8)
        win.pack(anchor="w", pady=(2, 6))
        win.bind("<<ComboboxSelected>>", lambda _e: self.redraw())
        self.autoscale = tk.BooleanVar(value=False)
        ttk.Checkbutton(opts, text="Auto scale", variable=self.autoscale, command=self.redraw).pack(anchor="w")
        buttons = ttk.Frame(opts)
        buttons.pack(anchor="w", pady=(8, 0))
        self.pause_btn = ttk.Button(buttons, text="Pause", width=7, command=self.toggle_pause)
        self.pause_btn.pack(side="left")
        ttk.Button(buttons, text="Clear", width=7, command=self.clear).pack(side="left", padx=(4, 0))

        self.canvas = tk.Canvas(outer, bg=C["tile"], highlightthickness=1, highlightbackground=C["border"])
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _e: self.redraw())

    def add(self, t: float, d: dict, v: dict) -> None:
        self.points.append((t, {name: get(d, v) for name, (_, _, get) in SIGNALS.items()}))
        # keep the longest window; while paused, keep everything after the paused view (hard cap ~1 h)
        oldest = (self.paused_at if self.paused_at is not None else t) - max(WINDOWS.values()) - 5
        while self.points and (self.points[0][0] < oldest or len(self.points) > 20000):
            self.points.popleft()

    def clear(self) -> None:
        self.points.clear()
        self.redraw()

    def toggle_pause(self) -> None:
        if self.paused_at is None:
            self.paused_at = self.points[-1][0] if self.points else time.time()
            self.pause_btn.configure(text="Resume")
        else:
            self.paused_at = None
            self.pause_btn.configure(text="Pause")
        self.redraw()

    def visible(self) -> bool:
        return str(self.frame) == self.frame.master.select()

    def redraw(self) -> None:
        cv = self.canvas
        cv.delete("all")
        w, h = cv.winfo_width(), cv.winfo_height()
        if w < 50 or h < 50:
            return
        span = WINDOWS[self.window.get()]
        end = self.paused_at or (self.points[-1][0] if self.points else time.time())
        start = end - span
        pts = [(t, vals) for t, vals in self.points if start <= t <= end]
        active = [(i, var.get()) for i, var in enumerate(self.channels) if var.get() != NO_SIGNAL]
        left, right, top_pad, bottom = 64, 16, 8, 34
        plot_w = w - left - right
        if plot_w < 40:
            return
        strip_h = (h - bottom) / max(len(active), 1)
        font = ("Segoe UI", 9)

        for row, (ch, name) in enumerate(active):
            unit, (lo, hi), _ = SIGNALS[name]
            values = [vals[name] for _, vals in pts if vals[name] is not None]
            if values:
                if self.autoscale.get():
                    lo, hi = min(values), max(values)
                else:
                    lo, hi = min(lo, min(values)), max(hi, max(values))
                if hi - lo < 1e-6:
                    lo, hi = lo - 1, hi + 1
                pad = (hi - lo) * 0.05 if self.autoscale.get() else 0
                lo, hi = lo - pad, hi + pad
            y0 = row * strip_h + top_pad
            y1 = (row + 1) * strip_h - 4
            colour = CHANNEL_COLOURS[ch]
            # frame + grid
            cv.create_rectangle(left, y0, left + plot_w, y1, outline=C["border"])
            for frac in (0.25, 0.5, 0.75):
                yy = y1 - frac * (y1 - y0)
                cv.create_line(left, yy, left + plot_w, yy, fill=C["grid"])
            for k in range(1, 6):
                xx = left + plot_w * k / 6
                cv.create_line(xx, y0, xx, y1, fill=C["grid"])
            cv.create_text(left - 6, y0 + 2, text=f"{hi:.4g}", anchor="ne", fill=C["muted"], font=font)
            cv.create_text(left - 6, y1 - 2, text=f"{lo:.4g}", anchor="se", fill=C["muted"], font=font)
            cv.create_text(left + 6, y0 + 4, text=f"{name} ({unit})", anchor="nw", fill=colour, font=font)
            if len(pts) >= 2:
                step = max(1, len(pts) // (plot_w * 2))  # thin out very dense data
                sel = pts[::step]
                if sel[-1] is not pts[-1]:
                    sel.append(pts[-1])  # always draw up to the newest point
                segment: list[float] = []
                for t, vals in sel + [(None, None)]:  # sentinel flushes the last segment
                    val = vals[name] if vals else None
                    if val is None:  # gap (e.g. no valid rpm sample): break the line
                        if len(segment) >= 4:
                            cv.create_line(*segment, fill=colour, width=2)
                        segment = []
                        continue
                    x = left + (t - start) / span * plot_w
                    y = y1 - (val - lo) / (hi - lo) * (y1 - y0)
                    segment += [x, min(max(y, y0), y1)]
            current = pts[-1][1][name] if pts else None  # newest point; None = no valid value now
            self.value_labels[ch].configure(text="--" if current is None else f"{current:.4g} {unit}")
        for ch, var in enumerate(self.channels):
            if var.get() == NO_SIGNAL:
                self.value_labels[ch].configure(text="")
        # time axis
        for k in range(0, 7):
            xx = left + plot_w * k / 6
            secs = -span + span * k / 6
            cv.create_text(xx, h - bottom + 4, text="now" if k == 6 else f"{secs:.0f} s",
                           anchor="ne" if k == 6 else "n", fill=C["muted"], font=font)
        if self.paused_at is not None:
            cv.create_text(left + plot_w - 6, 4, text="PAUSED", anchor="ne", fill=C["warn"],
                           font=("Segoe UI", 10, "bold"))


def enable_dark_titlebar(root: tk.Tk) -> None:
    """Windows 10/11: dark window title bar (ignored elsewhere)."""
    try:
        root.update_idletasks()
        hwnd = ctypes.windll.user32.GetParent(root.winfo_id())
        value = ctypes.c_int(1)
        for attr in (20, 19):  # DWMWA_USE_IMMERSIVE_DARK_MODE (new / old builds)
            if ctypes.windll.dwmapi.DwmSetWindowAttribute(hwnd, attr, ctypes.byref(value), ctypes.sizeof(value)) == 0:
                break
    except Exception:  # noqa: BLE001
        pass


def apply_dark_theme(root: tk.Tk) -> None:
    root.configure(bg=C["bg"])
    style = ttk.Style(root)
    style.theme_use("clam")
    base = {"background": C["surface"], "foreground": C["text"], "bordercolor": C["border"],
            "lightcolor": C["surface"], "darkcolor": C["surface"], "font": ("Segoe UI", 10)}
    style.configure(".", **base, troughcolor=C["field"], focuscolor=C["accent"],
                    selectbackground=C["accent_dim"], selectforeground=C["text"],
                    fieldbackground=C["field"], insertcolor=C["text"])
    style.configure("TFrame", background=C["surface"])
    style.configure("Bg.TFrame", background=C["bg"])
    style.configure("TLabel", background=C["surface"], foreground=C["text"])
    style.configure("Muted.TLabel", background=C["surface"], foreground=C["muted"])
    style.configure("TLabelframe", background=C["surface"], bordercolor=C["border"])
    style.configure("TLabelframe.Label", background=C["surface"], foreground=C["muted"])
    style.configure("TButton", background=C["field"], foreground=C["text"], bordercolor=C["border"],
                    padding=(10, 4))
    style.map("TButton", background=[("disabled", C["surface"]), ("pressed", C["accent_dim"]),
                                     ("active", C["border"])],
              foreground=[("disabled", C["dim"])])
    style.configure("TCheckbutton", background=C["surface"], foreground=C["text"], indicatorbackground=C["field"],
                    indicatorforeground=C["accent"])
    style.map("TCheckbutton", background=[("active", C["surface"])],
              indicatorbackground=[("selected", C["field"]), ("active", C["border"])])
    style.configure("Accent.TButton", background=C["accent_dim"], foreground="#ffffff")
    style.map("Accent.TButton", background=[("disabled", C["surface"]), ("active", C["accent"])])
    for widget in ("TCombobox", "TSpinbox", "TEntry"):
        style.configure(widget, fieldbackground=C["field"], background=C["field"], foreground=C["text"],
                        arrowcolor=C["muted"], bordercolor=C["border"])
        style.map(widget, fieldbackground=[("readonly", C["field"]), ("disabled", C["surface"])],
                  foreground=[("disabled", C["dim"])], selectbackground=[("readonly", C["field"])],
                  selectforeground=[("readonly", C["text"])])
    style.configure("TNotebook", background=C["bg"], bordercolor=C["border"], tabmargins=(6, 6, 6, 0))
    style.configure("TNotebook.Tab", background=C["surface"], foreground=C["muted"], padding=(16, 6),
                    bordercolor=C["border"])
    style.map("TNotebook.Tab", background=[("selected", C["tile"])], foreground=[("selected", C["text"])],
              lightcolor=[("selected", C["accent"])], bordercolor=[("selected", C["border"])])
    style.configure("Treeview", background=C["surface"], fieldbackground=C["surface"], foreground=C["text"],
                    bordercolor=C["border"], rowheight=22)
    style.configure("Treeview.Heading", background=C["field"], foreground=C["muted"], bordercolor=C["border"])
    style.map("Treeview", background=[("selected", C["accent_dim"])])
    style.configure("Vertical.TScrollbar", background=C["field"], troughcolor=C["surface"],
                    bordercolor=C["border"], arrowcolor=C["muted"])
    # dropdown list of the combobox
    root.option_add("*TCombobox*Listbox.background", C["field"])
    root.option_add("*TCombobox*Listbox.foreground", C["text"])
    root.option_add("*TCombobox*Listbox.selectBackground", C["accent_dim"])
    root.option_add("*TCombobox*Listbox.selectForeground", C["text"])


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.q: queue.Queue = queue.Queue()
        self.link: Link | None = None
        self.ecu_id = ""
        self.history: deque = deque()
        self.prev: dict | None = None
        self.sample_times: deque = deque(maxlen=20)
        self.log_writer = None
        self.log_file = None
        self.log_path: Path | None = None
        self.log_rows = 0
        self.idle_confirmed = False
        self.engine_ran = False
        self.idle_target: int | None = None   # as reported by the link thread
        self.stop_pending = False
        self.auto_stop_sent = False
        self.link_error = False
        self.closing = False

        root.title("SDLView")
        scale = root.winfo_fpixels("1i") / 96  # Windows display scaling (1.0 = 100 %)
        width = min(int(1100 * scale), int(root.winfo_screenwidth() * 0.95))
        height = min(int(740 * scale), int(root.winfo_screenheight() * 0.88))
        root.geometry(f"{width}x{height}")
        root.minsize(min(width, int(900 * scale)), min(height, int(600 * scale)))
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        set_window_icon(root)
        apply_dark_theme(root)
        self._build()
        enable_dark_titlebar(root)
        self.refresh_ports()
        self.root.after(100, self.poll)

    # ---- layout
    def _build(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=34, state="readonly")
        self.port_box.pack(side="left", padx=6)
        ttk.Button(top, text="↻", width=3, command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Connect", style="Accent.TButton", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=6)
        self.status_dot = tk.Label(top, text="●", fg=C["dim"], bg=C["surface"], font=("Segoe UI", 12))
        self.status_dot.pack(side="left", padx=(14, 4))
        self.status_var = tk.StringVar(value="Not connected")
        ttk.Label(top, textvariable=self.status_var).pack(side="left")
        self.rate_var = tk.StringVar()
        ttk.Label(top, textvariable=self.rate_var, style="Muted.TLabel").pack(side="right")

        bottom = ttk.Frame(self.root, padding=8)
        bottom.pack(side="bottom", fill="x")  # packed before the notebook so it always stays visible

        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=8, pady=(4, 0))
        self._build_dashboard(nb)
        self.graph = GraphTab(nb)
        self._build_faults(nb)
        self._build_raw(nb)
        self.notebook = nb
        nb.bind("<<NotebookTabChanged>>", lambda _e: self.graph.redraw() if self.graph.visible() else None)

        logbox = ttk.LabelFrame(bottom, text="Logging", padding=8)
        logbox.pack(side="left", fill="y")
        self.log_btn = ttk.Button(logbox, text="Start log", command=self.toggle_log)
        self.log_btn.pack(side="left")
        self.log_var = tk.StringVar(value="")
        ttk.Label(logbox, textvariable=self.log_var, width=32, style="Muted.TLabel").pack(side="left", padx=6)

        warn = ttk.LabelFrame(bottom, text="Coolant warning", padding=8)
        warn.pack(side="left", fill="y", padx=8)
        self.ect_limit = tk.IntVar(value=105)
        ttk.Spinbox(warn, from_=40, to=120, increment=5, textvariable=self.ect_limit, width=5).pack(side="left")
        ttk.Label(warn, text="°C").pack(side="left", padx=4)

        idle = ttk.LabelFrame(bottom, text="Idle command", padding=8)
        idle.pack(side="right", fill="y")
        self.idle_rpm = tk.IntVar(value=1000)
        ttk.Spinbox(idle, from_=IDLE_MIN, to=IDLE_MAX, increment=50, textvariable=self.idle_rpm, width=6).pack(side="left")
        ttk.Label(idle, text="rpm").pack(side="left", padx=4)
        self.idle_send = ttk.Button(idle, text="Send target", command=self.send_idle, state="disabled")
        self.idle_send.pack(side="left", padx=4)
        self.idle_stop = ttk.Button(idle, text="Restore (stop)", command=self.send_stop, state="disabled")
        self.idle_stop.pack(side="left")
        status_col = ttk.Frame(idle)
        status_col.pack(side="left", padx=6)
        self.idle_state_var = tk.StringVar(value="ECU in control")
        self.idle_state_lbl = tk.Label(status_col, textvariable=self.idle_state_var, fg=C["muted"], bg=C["surface"],
                                       font=("Segoe UI", 10, "bold"), anchor="w", width=30)
        self.idle_state_lbl.pack(anchor="w")
        self.idle_var = tk.StringVar(value="")
        ttk.Label(status_col, textvariable=self.idle_var, width=36, style="Muted.TLabel").pack(anchor="w")

    def _build_dashboard(self, nb: ttk.Notebook) -> None:
        frame = tk.Frame(nb, bg=C["bg"], padx=6, pady=6)
        nb.add(frame, text="Dashboard")
        self.tiles = {}
        for n, (key, title, unit, _, unconfirmed) in enumerate(TILES):
            tile = tk.Frame(frame, bg=C["tile"], padx=12, pady=8, highlightthickness=1,
                            highlightbackground=C["border"])
            tile.grid(row=n // 4, column=n % 4, sticky="nsew", padx=5, pady=5)
            head = tk.Frame(tile, bg=C["tile"])
            head.pack(fill="x")
            tk.Label(head, text=title, fg=C["muted"], bg=C["tile"], font=("Segoe UI", 10)).pack(side="left")
            if unconfirmed:
                tk.Label(head, text="⚠", fg=C["warn"], bg=C["tile"], font=("Segoe UI", 10)).pack(side="left", padx=4)
            val = tk.Label(tile, text="--", fg=C["text"], bg=C["tile"], font=("Segoe UI", 26, "bold"))
            val.pack(anchor="w")
            tk.Label(tile, text=unit, fg=C["dim"], bg=C["tile"], font=("Segoe UI", 9)).pack(anchor="w")
            self.tiles[key] = val
        for c in range(4):
            frame.columnconfigure(c, weight=1, uniform="tiles")
        for r in range(4):
            frame.rowconfigure(r, weight=1, uniform="tiles")
        tk.Label(frame, text="⚠ = formula not yet confirmed for this ECU (see README)",
                 fg=C["dim"], bg=C["bg"]).grid(row=4, column=0, columnspan=4, sticky="w", padx=6, pady=(2, 0))

    def _build_faults(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="Fault codes")
        self.fault_text = tk.Text(frame, height=20, font=("Consolas", 11), state="disabled", bg=C["tile"],
                                  fg=C["text"], relief="flat", padx=10, pady=8, highlightthickness=1,
                                  highlightbackground=C["border"], insertbackground=C["text"])
        self.fault_text.tag_configure("head", foreground=C["muted"])
        self.fault_text.tag_configure("current", foreground=C["danger"])
        self.fault_text.tag_configure("stored", foreground=C["warn"])
        self.fault_text.tag_configure("none", foreground=C["ok"])
        self.fault_text.pack(fill="both", expand=True)
        ttk.Label(frame, text="Clearing: disconnect the battery negative for ~1 minute "
                              "(no SDL clear command is known yet).", style="Muted.TLabel").pack(anchor="w", pady=6)

    def _build_raw(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=6)
        nb.add(frame, text="Raw")
        self.raw_tree = ttk.Treeview(frame, columns=("dec", "hex", "bits"), height=24)
        self.raw_tree.heading("#0", text="Address")
        for col, w in (("dec", 80), ("hex", 80), ("bits", 140)):
            self.raw_tree.heading(col, text=col)
            self.raw_tree.column(col, width=w, anchor="center")
        self.raw_tree.column("#0", width=90)
        self.raw_tree.tag_configure("changed", background=C["changed"], foreground=C["warn"])
        for a in ADDRS:
            self.raw_tree.insert("", "end", iid=str(a), text=f"0x{a:02X}", values=("", "", ""))
        scroll = ttk.Scrollbar(frame, command=self.raw_tree.yview)
        self.raw_tree.configure(yscrollcommand=scroll.set)
        self.raw_tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="left", fill="y")

    # ---- connection
    def refresh_ports(self) -> None:
        ports = list(serial.tools.list_ports.comports())
        labels = [f"{p.device} – {p.description}" for p in ports]
        self.port_box["values"] = labels
        preferred = next((l for l, p in zip(labels, ports) if "SDL Interface" in (p.description or "")
                          or (p.serial_number or "").startswith("R1")), None)
        preferred = preferred or next((l for l, p in zip(labels, ports) if p.vid == 0x0403), None)
        self.port_var.set(preferred or (labels[0] if labels else ""))

    def set_status(self, text: str, colour: str) -> None:
        self.status_var.set(text)
        self.status_dot.configure(fg=colour)

    def toggle_connect(self) -> None:
        if self.link:
            self.stop_link()
            return
        port = self.port_var.get().split(" ")[0]
        if not port:
            messagebox.showwarning("SDLView", "No COM port found. Is the cable plugged in?")
            return
        self.start_link(port=port)

    def start_link(self, port: str) -> None:
        self.history.clear()
        self.graph.clear()
        self.sample_times.clear()
        self.prev = None
        self.engine_ran = False
        self.link_error = False
        self.set_idle_state(None, False)
        self.link = Link(self.q, port)
        self.link.start()
        self.set_status(f"Connecting to {port}…", C["warn"])
        self.connect_btn.configure(text="Disconnect")

    def stop_link(self) -> None:
        if self.link:
            self.link.halt.set()  # the thread sends a stop frame itself if a target may be active
            self.set_status("Disconnecting…", C["warn"])

    # ---- idle command
    def send_idle(self) -> None:
        if not self.link:
            return
        try:
            rpm = max(IDLE_MIN, min(IDLE_MAX, int(self.idle_rpm.get())))
        except (tk.TclError, ValueError):
            messagebox.showwarning("Idle command", f"Enter a target between {IDLE_MIN} and {IDLE_MAX} rpm.")
            return
        if not self.idle_confirmed:
            ok = messagebox.askokcancel(
                "Idle command",
                f"Temporarily set the ECU target idle to {rpm} rpm?\n\n"
                "Restore (stop) sets it back. It is also stopped automatically on disconnect, "
                f"on closing, or on abnormal engine speed (> {AUTO_STOP_HIGH} rpm or stall).")
            if not ok:
                return
            self.idle_confirmed = True
        self.auto_stop_sent = False
        self.link.cmds.put(("idle", rpm))
        self.idle_var.set(f"Sending target {rpm} rpm…")

    def send_stop(self) -> None:
        if self.link:
            self.link.cmds.put(("stop",))
            self.idle_var.set("Sending stop…")

    def set_idle_state(self, target: int | None, stop_pending: bool) -> None:
        """Driven by the link thread: target that may be active in the ECU, and whether a stop is outstanding."""
        self.idle_target, self.stop_pending = target, stop_pending
        # no new target while a stop is still unconfirmed (the retry would cancel it straight away)
        self.idle_send.configure(state="normal" if self.link and not stop_pending else "disabled")
        if stop_pending:
            self.idle_state_var.set("STOP NOT CONFIRMED – retrying")
            self.idle_state_lbl.configure(fg=C["danger"])
        elif target is not None:
            self.idle_state_var.set(f"Target active: {target} rpm")
            self.idle_state_lbl.configure(fg=C["accent"])
        else:
            self.idle_state_var.set("ECU in control")
            self.idle_state_lbl.configure(fg=C["muted"])

    # ---- logging
    def toggle_log(self) -> None:
        if self.log_file:
            self.close_log("Saved:")
            return
        path = None
        for folder in (app_dir() / "logs", Path.home() / "Documents" / "SDLView logs"):
            try:
                folder.mkdir(parents=True, exist_ok=True)
                path = folder / f"sdl_{datetime.now():%Y%m%d_%H%M%S}.csv"
                self.log_file = open(path, "w", newline="", encoding="utf-8")
                break
            except OSError:
                self.log_file, path = None, None
        if not self.log_file:
            messagebox.showerror("Logging", "Cannot create a log file (folder not writable).")
            return
        self.log_path = path
        self.log_rows = 0
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(["time", "ecu"] + [f"0x{a:02X}" for a in ADDRS] +
                                 ["rpm", "target_idle", "coolant_c", "iat_c", "map_mbar", "baro_mbar", "tps_v",
                                  "tps_angle", "isc_pct", "inj_ms", "advance_unconfirmed", "battery_v", "ctp", "fan",
                                  "current_codes", "stored_codes"])
        self.log_btn.configure(text="Stop log")
        self.log_var.set(f"Logging to {path.name}")

    def close_log(self, prefix: str) -> None:
        try:
            self.log_file.close()
        except OSError:
            pass
        self.log_file = self.log_writer = None
        self.log_btn.configure(text="Start log")
        self.log_var.set(f"{prefix} {self.log_path.name}" if self.log_path else "")

    def write_log(self, t: float, v: dict, d: dict, cur: list, sto: list) -> None:
        if not self.log_writer:
            return
        codes = lambda lst: " ".join(str(c) for c, _ in lst)  # noqa: E731
        try:
            self.log_writer.writerow(
                [datetime.fromtimestamp(t).isoformat(timespec="milliseconds"), self.ecu_id] + [v[a] for a in ADDRS] +
                ["" if d["rpm"] is None else f"{d['rpm']:.0f}", d["target_idle"], f"{d['ect']:.1f}",
                 f"{d['iat']:.1f}", f"{d['map']:.0f}", f"{d['baro']:.0f}", f"{d['tps_v']:.2f}", d["tps_angle"],
                 d["isc"], f"{d['inj']:.3f}", f"{d['adv']:.1f}", f"{d['batt']:.2f}", int(d["ctp"]), int(d["fan"]),
                 codes(cur), codes(sto)])
            self.log_rows += 1
            if self.log_rows % 20 == 0:
                self.log_file.flush()
        except OSError as exc:
            self.close_log("Log stopped (write error):")
            text = f"Writing the log failed and logging was stopped:\n{exc}"
            # after_idle: a modal dialog inside poll() would pause sample processing (and auto-stop)
            self.root.after_idle(lambda: messagebox.showerror("Logging", text))

    # ---- processing
    def poll(self) -> None:
        try:
            msgs = []
            while True:
                try:
                    msgs.append(self.q.get_nowait())
                except queue.Empty:
                    break
            last_sample = max((i for i, m in enumerate(msgs) if m[0] == "sample"), default=-1)
            for i, msg in enumerate(msgs):
                try:
                    if msg[0] == "sample":
                        # all samples are logged/graphed; only the newest one is rendered (no GUI backlog)
                        self.on_sample(msg[1], msg[2], render=(i == last_sample))
                    else:
                        self.handle(msg)
                except Exception as exc:  # noqa: BLE001 - never let one bad message stop the GUI loop
                    self.set_status(f"Internal error: {exc}", C["danger"])
        finally:
            self.poll_job = self.root.after(100, self.poll)

    def handle(self, msg: tuple) -> None:
        kind = msg[0]
        if kind == "connected":
            self.ecu_id = msg[1]
            known = self.ecu_id == "1453"  # formulas are verified for ECU 1453 only
            self.set_status(f"Connected · ECU {self.ecu_id}" + ("" if known else
                            " (formulas verified for ECU 1453 only)"), C["ok"] if known else C["warn"])
            self.idle_send.configure(state="normal")
            self.idle_stop.configure(state="normal")
        elif kind == "ack":
            self.idle_var.set(msg[1])
        elif kind == "idle_state":
            self.set_idle_state(msg[1], msg[2])
        elif kind == "error":
            self.link_error = True
            self.set_status(msg[1], C["warn"])
        elif kind == "closed":
            _, text, is_error, stop_failed = msg
            self.link = None
            self.set_status(text, C["danger"] if is_error else C["dim"])
            self.connect_btn.configure(text="Connect")
            self.idle_send.configure(state="disabled")
            self.idle_stop.configure(state="disabled")
            self.rate_var.set("")
            warning = ("The stop frame could not be confirmed by the ECU.\n\n"
                       "The changed target idle may still be active. Turn the ignition off and on "
                       "to return the ECU to its own idle control.")
            if stop_failed:
                self.set_idle_state(self.idle_target, True)
            else:
                self.set_idle_state(None, False)
            if self.closing:
                # Closing: show the warning (if any) before the window goes away, then close.
                if stop_failed:
                    messagebox.showwarning("Idle command", warning)
                self.root.after_idle(self.finish_close)
            elif stop_failed:
                # Not from inside poll(): a modal dialog there would pause sample processing.
                self.root.after_idle(lambda: messagebox.showwarning("Idle command", warning))

    def on_sample(self, t: float, v: dict, render: bool = True) -> None:
        if self.link_error:
            self.link_error = False
            self.set_status(f"Connected · ECU {self.ecu_id}", C["ok"])
        self.history.append((t, v))
        while self.history and t - self.history[0][0] > 1.2:
            self.history.popleft()
        self.sample_times.append(t)
        if len(self.sample_times) > 1:
            span = self.sample_times[-1] - self.sample_times[0]
            if span > 0:
                self.rate_var.set(f"{(len(self.sample_times) - 1) / span:.1f} samples/s")

        d = derive(v, self.history)
        table = FAULTS_1453  # only ECU 1453 is mapped; other ECUs get a warning in the status bar
        cur = decode_faults(table, 0, v[0x00]) + decode_faults(table, 1, v[0x01])
        sto = decode_faults(table, 2, v[0x02]) + decode_faults(table, 3, v[0x03])
        self.safety(d)  # first, so a display problem can never block the auto-stop
        self.graph.add(t, d, v)
        self.write_log(t, v, d, cur, sto)
        if render:
            self.render(d, v, cur, sto)
        self.prev = v

    def render(self, d: dict, v: dict, cur: list, sto: list) -> None:
        for key, _, _, fmt, _ in TILES:
            val = d[key]
            colour = C["text"]
            if val is None:
                text = "--"
            elif key in ("ctp", "fan"):
                text = "ON" if val else "OFF"
                colour = C["ok"] if val else C["muted"]
            elif key == "mix":
                text = " / ".join(str(x) for x in val)
            elif key == "adv":
                text = f"{val:.0f} ({d['adv_raw']})"
            else:
                text = fmt.format(val)
            self.tiles[key].configure(text=text, fg=colour)
        try:
            ect_limit = float(self.ect_limit.get())
        except (tk.TclError, ValueError):
            ect_limit = 105.0
        if d["ect"] >= ect_limit:
            self.tiles["ect"].configure(fg=C["danger"])
        if d["batt"] < 11.8:
            self.tiles["batt"].configure(fg=C["warn"])
        if self.idle_target is not None or self.stop_pending:
            self.tiles["target_idle"].configure(fg=C["danger"] if self.stop_pending else C["accent"])
        self.show_faults(cur, sto, v)
        self.show_raw(v)
        if self.graph.visible() and self.graph.paused_at is None:
            self.graph.redraw()

    def show_faults(self, cur: list, sto: list, v: dict) -> None:
        parts = [(f"ECU {self.ecu_id}\n\n", "head"), ("CURRENT  (0x00, 0x01)\n", "head")]
        parts += [(f"  {c}  {txt}\n", "current") for c, txt in cur] or [("  none\n", "none")]
        parts += [("\nSTORED  (0x02, 0x03)\n", "head")]
        parts += [(f"  {c}  {txt}\n", "stored") for c, txt in sto] or [("  none\n", "none")]
        parts += [(f"\n0x20 = {v[0x20]}   0x21 = {v[0x21]}   (meaning unknown)\n", "head")]
        text = "".join(p for p, _ in parts)
        if self.fault_text.get("1.0", "end-1c") in (text, text.rstrip("\n")):
            return
        self.fault_text.configure(state="normal")
        self.fault_text.delete("1.0", "end")
        for chunk, tag in parts:
            self.fault_text.insert("end", chunk, tag)
        self.fault_text.configure(state="disabled")

    def show_raw(self, v: dict) -> None:
        for a in ADDRS:
            changed = self.prev is not None and self.prev.get(a) != v[a]
            self.raw_tree.item(str(a), values=(v[a], f"0x{v[a]:02X}", f"{v[a]:08b}"),
                               tags=("changed",) if changed else ())

    def safety(self, d: dict) -> None:
        rpm = d["rpm"]
        if rpm is not None and rpm > 400:
            self.engine_ran = True
        if not self.link or self.idle_target is None or self.auto_stop_sent:
            return
        stalled = self.engine_ran and (rpm is None or rpm < AUTO_STOP_LOW)
        if (rpm is not None and rpm > AUTO_STOP_HIGH) or stalled:
            self.link.cmds.put(("stop",))
            self.auto_stop_sent = True  # the thread retries until the stop is acknowledged
            self.idle_var.set(f"Auto stop ({'stall' if stalled else f'{rpm:.0f} rpm'})")

    def on_close(self) -> None:
        """Close the window. With an open link, the window stays until the link's "closed" message
        has been handled (stop sent and, if needed, the warning shown); see handle()."""
        if self.link is None:
            self.finish_close()
            return
        if not self.closing:
            self.closing = True
            self.link.halt.set()
            self.set_status("Closing: restoring ECU and closing the port…", C["warn"])

    def finish_close(self) -> None:
        if self.log_file:
            self.close_log("Saved:")
        if getattr(self, "poll_job", None):
            self.root.after_cancel(self.poll_job)
        self.root.destroy()


def main() -> None:
    argparse.ArgumentParser(description="SDLView: live viewer for Suzuki SDL ECUs").parse_args()
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)  # sharp text on scaled displays
    except Exception:  # noqa: BLE001
        pass
    root = tk.Tk()
    app = App(root)
    try:
        root.mainloop()
    finally:
        # Safety net if the main loop ends without on_close (e.g. Ctrl+C): stop the link thread
        # so it sends its stop frame and releases the port instead of running on invisibly.
        if app.link is not None:
            app.link.halt.set()
            app.link.join(timeout=10)


if __name__ == "__main__":
    main()
