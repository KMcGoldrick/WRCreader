"""
tcm_plotter.py
--------------
Reads and plots TCM sensor data from either a live serial (RS-485/UART)
port or a log file.

Auto-detects frame format:
  Binary: 0xAA | case (1 byte) | length (1 byte) | payload (raw bytes)
  Text:   <case>,<val1>,<val2>,...\n

Supported cases (plottable)
  1  - Heading (deg) + Velocity N/E (m/s)
  2  - Roll / Pitch / Yaw (rad)
  3  - Accel raw (int16 x3) + scaled (float x3)
  4  - Mag   raw (int16 x3) + scaled (float x3)
  5  - Temp  raw + scaled,  Batt raw + scaled

Cases 6-11 are calibration/config: logged to status bar, not plotted.

Auto-save: when started in serial mode a log file is created automatically
in the same directory as this script, named:
  tcm_YYYYMMDD_HHMMSS.csv   (text mode)
  tcm_YYYYMMDD_HHMMSS.bin   (binary mode)
The file format is determined by the detected stream format and matches
what "Read From Log File" can read back.

Usage:
  pip install pyserial matplotlib
  python tcm_plotter.py
"""

import math
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import tkinter.scrolledtext as scrolledtext
import threading
import queue
import struct
import os
import datetime
from pathlib import Path

import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

# ── Constants ─────────────────────────────────────────────────────────────────
WINDOW           = 200
BAUDRATE         = 115200
INTERVAL         = 50           # animation refresh ms
TCM_BIN_SOF      = 0xAA
RAW_WIN_MAX_LINES = 2000

# LOG_DIR: prefer the user's Downloads folder, fall back to script directory
try:
    _script_dir = Path(__file__).resolve().parent
except Exception:
    _script_dir = Path(os.path.abspath(os.path.dirname(__file__)))
_downloads = Path.home() / "Downloads"
try:
    _downloads.mkdir(parents=True, exist_ok=True)
    LOG_DIR = str(_downloads)
except Exception:
    LOG_DIR = str(_script_dir)

# ── Case metadata ──────────────────────────────────────────────────────────────
CASES = {
    1: {
        "label":      "1 - Heading + Velocity",
        "channels":   ["Heading (deg)", "Velocity N (m/s)", "Velocity E (m/s)"],
        "parse_text": lambda v: [float(v[0]), float(v[1]), float(v[2])],
        "parse_bin":  lambda p: list(struct.unpack_from("<fff", p)),
    },
    2: {
        "label":      "2 - Roll / Pitch / Yaw",
        "channels":   ["Roll (rad)", "Pitch (rad)", "Yaw (rad)"],
        "parse_text": lambda v: [float(v[0]), float(v[1]), float(v[2])],
        "parse_bin":  lambda p: list(struct.unpack_from("<fff", p)),
    },
    3: {
        "label":      "3 - Accelerometer",
        "channels":   ["Acc X raw", "Acc Y raw", "Acc Z raw",
                       "Acc X scaled", "Acc Y scaled", "Acc Z scaled"],
        "parse_text": lambda v: [int(v[0]),   int(v[1]),   int(v[2]),
                                 float(v[3]), float(v[4]), float(v[5])],
        "parse_bin":  lambda p: list(struct.unpack_from("<hhhfff", p)),
    },
    4: {
        "label":      "4 - Magnetometer",
        "channels":   ["Mag X raw", "Mag Y raw", "Mag Z raw",
                       "Mag X scaled", "Mag Y scaled", "Mag Z scaled"],
        "parse_text": lambda v: [int(v[0]),   int(v[1]),   int(v[2]),
                                 float(v[3]), float(v[4]), float(v[5])],
        "parse_bin":  lambda p: list(struct.unpack_from("<hhhfff", p)),
    },
    5: {
        "label":      "5 - Temp + Battery",
        "channels":   ["Temp raw", "Temp scaled", "Batt raw", "Batt scaled"],
        "parse_text": lambda v: [int(v[0]),   float(v[1]),
                                 int(v[2]),   float(v[3])],
        "parse_bin":  lambda p: list(struct.unpack_from("<HfHf", p)),
    },
}

NON_PLOT_CASES = {6, 7, 8, 9, 10, 11}


# ── Format auto-detector ───────────────────────────────────────────────────────
class FormatDetector:
    SNIFF = 8

    def __init__(self):
        self._buf   = bytearray()
        self.format = None

    def feed(self, byte: int):
        if self.format:
            return
        self._buf.append(byte)
        if len(self._buf) >= self.SNIFF:
            self._decide()

    def _decide(self):
        for i in range(len(self._buf) - 1):
            if self._buf[i] == TCM_BIN_SOF and self._buf[i + 1] in range(12):
                self.format = "binary"
                return
        self.format = "text"

    def detected(self) -> bool:
        return self.format is not None


# ── Binary frame reader ────────────────────────────────────────────────────────
class BinaryFrameReader:
    WAIT_SOF, WAIT_CASE, WAIT_LEN, WAIT_PAYLOAD = range(4)

    def __init__(self):
        self._state   = self.WAIT_SOF
        self._case_id = 0
        self._length  = 0
        self._payload = bytearray()
        self._frames  = []

    def push(self, byte: int):
        if self._state == self.WAIT_SOF:
            if byte == TCM_BIN_SOF:
                self._state = self.WAIT_CASE
        elif self._state == self.WAIT_CASE:
            self._case_id = byte
            self._state   = self.WAIT_LEN
        elif self._state == self.WAIT_LEN:
            self._length  = byte
            self._payload = bytearray()
            if self._length == 0:
                self._emit()
            else:
                self._state = self.WAIT_PAYLOAD
        elif self._state == self.WAIT_PAYLOAD:
            self._payload.append(byte)
            if len(self._payload) == self._length:
                self._emit()

    def _emit(self):
        self._frames.append((self._case_id, bytes(self._payload)))
        self._state = self.WAIT_SOF

    def pop_frames(self):
        out, self._frames = self._frames, []
        return out


# ── Text line parser ───────────────────────────────────────────────────────────
def parse_text_line(line: str):
    line = line.strip()
    if not line:
        return None
    parts = line.split(",")
    if len(parts) < 2:
        return None
    try:
        case_id = int(parts[0])
    except ValueError:
        return None
    return case_id, parts[1:]


# ── Auto-save log writer ───────────────────────────────────────────────────────
class LogWriter:
    """
    Writes incoming raw data to a log file whose format (text/binary)
    matches the detected stream format.  Thread-safe.
    """
    def __init__(self, fmt: str):
        """fmt: 'text' or 'binary'"""
        self.fmt      = fmt
        self._lock    = threading.Lock()
        ts            = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        ext           = ".csv" if fmt == "text" else ".bin"
        self.filepath = os.path.join(LOG_DIR, f"tcm_{ts}{ext}")
        mode          = "w" if fmt == "text" else "wb"
        self._fh      = open(self.filepath, mode)

    def write_text_line(self, line: str):
        with self._lock:
            if self._fh and not self._fh.closed:
                self._fh.write(line if line.endswith("\n") else line + "\n")

    def write_binary_frame(self, case_id: int, payload: bytes):
        """Reconstruct the binary frame (SOF | case | len | payload) and write."""
        with self._lock:
            if self._fh and not self._fh.closed:
                frame = bytes([TCM_BIN_SOF, case_id, len(payload)]) + payload
                self._fh.write(frame)

    def close(self):
        with self._lock:
            if self._fh and not self._fh.closed:
                self._fh.flush()
                self._fh.close()


# ── Serial reader thread ───────────────────────────────────────────────────────
class SerialReader(threading.Thread):
    """
    Reads raw bytes, auto-detects format, dispatches to queue.
    Once format is known, creates a LogWriter and saves all frames.

    Queue messages:
      ("DATA",   case_id, values, fmt)
      ("STATUS", message)
      ("ERROR",  message)
      ("RAW",    raw_string)
      ("LOGFILE", filepath)     -- sent once log file is created
    """
    def __init__(self, port, baud, q: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.port        = port
        self.baud        = baud
        self.q           = q
        self.stop_event  = stop_event
        self.ser         = None
        self._write_lock = threading.Lock()
        self._log        = None     # LogWriter, set after format detected

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                self.ser   = ser
                detector   = FormatDetector()
                bin_reader = BinaryFrameReader()
                text_buf   = bytearray()

                self.q.put(("STATUS", "Detecting frame format..."))

                while not self.stop_event.is_set():
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue

                    for byte in chunk:
                        # ── detection phase ──
                        if not detector.detected():
                            detector.feed(byte)
                            if detector.detected():
                                fmt = detector.format
                                self._log = LogWriter(fmt)
                                self.q.put(("STATUS",
                                    f"Format detected: {fmt.upper()}  |  "
                                    f"Logging → {os.path.basename(self._log.filepath)}"))
                                self.q.put(("LOGFILE", self._log.filepath))
                            continue  # still sniffing; don't process yet

                        # ── processing phase ──
                        if detector.format == "binary":
                            bin_reader.push(byte)
                            for cid, payload in bin_reader.pop_frames():
                                if self._log:
                                    self._log.write_binary_frame(cid, payload)
                                raw_str = f"BIN,{cid},{payload.hex()}"
                                self.q.put(("RAW", raw_str))
                                self._dispatch_binary(cid, payload)
                        else:
                            text_buf.append(byte)
                            if byte == ord('\n'):
                                line = text_buf.decode("ascii", errors="replace")
                                text_buf.clear()
                                if self._log:
                                    self._log.write_text_line(line)
                                self.q.put(("RAW", line))
                                result = parse_text_line(line)
                                if result:
                                    cid, values = result
                                    self.q.put(("DATA", cid, values, "text"))

        except serial.SerialException as e:
            self.q.put(("ERROR", str(e)))
        finally:
            if self._log:
                self._log.close()
            self.ser = None

    def _dispatch_binary(self, cid, payload):
        if cid in NON_PLOT_CASES:
            self.q.put(("DATA", cid, payload, "binary"))
            return
        if cid not in CASES:
            return
        try:
            values = CASES[cid]["parse_bin"](payload)
            self.q.put(("DATA", cid, values, "binary"))
        except struct.error:
            pass

    def write(self, data: bytes) -> bool:
        try:
            with self._write_lock:
                if self.ser and getattr(self.ser, "is_open", False):
                    self.ser.write(data)
                    return True
        except Exception as e:
            try:
                self.q.put(("ERROR", f"Write failed: {e}"))
            except Exception:
                pass
        return False

    def write_text(self, text: str, encoding="ascii") -> bool:
        try:
            return self.write(text.encode(encoding, errors="replace"))
        except Exception:
            return False


# ── Main application ───────────────────────────────────────────────────────────
class TCMPlotter(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("TCM Data Plotter")
        self.resizable(True, True)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.source_var  = tk.StringVar(value="serial")
        self.port_var    = tk.StringVar()
        self.baud_var    = tk.StringVar(value=str(BAUDRATE))
        self.case_var    = tk.IntVar(value=1)
        self.file_path   = tk.StringVar(value="")
        self.status_var  = tk.StringVar(value="Ready.")
        self.format_var  = tk.StringVar(value="--")
        self.logfile_var = tk.StringVar(value="")

        self.serial_stop   = threading.Event()
        self.serial_thread = None
        self.data_queue    = queue.Queue()
        self.anim          = None
        self.buffers       = {}
        self.fig           = None
        self.axes          = []
        self.lines         = []
        self.pv_labels     = []     # list of tk.StringVar, one per channel
        self.canvas_widget = None
        self.running       = False

        self.raw_win        = None
        self.raw_text       = None
        self.raw_autoscroll = tk.BooleanVar(value=True)
        self.send_win       = None
        self.send_entry     = None
        self.append_nl      = tk.BooleanVar(value=True)

        # Show angles in degrees when True, radians when False
        self.show_degrees = tk.BooleanVar(value=True)

        # Track last message sent (display in main window) - must exist before building controls
        self.last_sent_var = tk.StringVar(value="")

        # Pending auto-switch case (scheduled from animation callback)
        self._pending_auto_case = None

        self._build_controls()
        self._refresh_ports()

        # When True we suppress sending a case-change command to the device.
        # Useful when the combobox is changed programmatically.
        self._suppress_case_send = False

    # ── UI ─────────────────────────────────────────────────────────────────────
    def _build_controls(self):
        ctrl = ttk.Frame(self, padding=8)
        ctrl.pack(side=tk.TOP, fill=tk.X)

        # Source radios
        ttk.Label(ctrl, text="Source:").grid(row=0, column=0, sticky=tk.W)
        ttk.Radiobutton(ctrl, text="Serial Port", variable=self.source_var,
                        value="serial", command=self._on_source_change
                        ).grid(row=0, column=1, sticky=tk.W)
        ttk.Radiobutton(ctrl, text="Read From Log File", variable=self.source_var,
                        value="file", command=self._on_source_change
                        ).grid(row=0, column=2, sticky=tk.W)

        # Format indicator
        ttk.Label(ctrl, text="   Format:").grid(row=0, column=3, sticky=tk.W)
        ttk.Label(ctrl, textvariable=self.format_var,
                  foreground="#0070c0",
                  font=("TkDefaultFont", 9, "bold")
                  ).grid(row=0, column=4, sticky=tk.W)

        # Angle units toggle (rad/deg)
        ttk.Checkbutton(ctrl, text="Degrees",
                        variable=self.show_degrees,
                        command=self._on_angle_toggle).grid(row=0, column=5, sticky=tk.W, padx=(8,0))

        # Last-sent indicator (main window)
        ttk.Label(ctrl, text="Last Sent:").grid(row=0, column=6, sticky=tk.W)
        ttk.Label(ctrl, textvariable=self.last_sent_var).grid(row=0, column=7, sticky=tk.W)

        # Port row
        self.port_frame = ttk.Frame(ctrl)
        self.port_frame.grid(row=1, column=0, columnspan=8, sticky=tk.W, pady=2)
        ttk.Label(self.port_frame, text="Port:").pack(side=tk.LEFT)
        self.port_cb = ttk.Combobox(self.port_frame, textvariable=self.port_var, width=14)
        self.port_cb.pack(side=tk.LEFT, padx=4)
        ttk.Button(self.port_frame, text="Refresh", width=7,
                   command=self._refresh_ports).pack(side=tk.LEFT)
        ttk.Label(self.port_frame, text="  Baud:").pack(side=tk.LEFT)
        ttk.Entry(self.port_frame, textvariable=self.baud_var, width=8
                  ).pack(side=tk.LEFT, padx=4)

        # File row (hidden initially)
        self.file_frame = ttk.Frame(ctrl)
        self.file_frame.grid(row=2, column=0, columnspan=8, sticky=tk.W, pady=2)
        ttk.Label(self.file_frame, text="File:").pack(side=tk.LEFT)
        ttk.Entry(self.file_frame, textvariable=self.file_path, width=44
                  ).pack(side=tk.LEFT, padx=4)
        ttk.Button(self.file_frame, text="Browse...",
                   command=self._browse).pack(side=tk.LEFT)
        self.file_frame.grid_remove()

        # Auto-save log path display (serial mode only)
        self.logfile_frame = ttk.Frame(ctrl)
        self.logfile_frame.grid(row=3, column=0, columnspan=8, sticky=tk.W, pady=2)
        ttk.Label(self.logfile_frame, text="Auto-log:").pack(side=tk.LEFT)
        ttk.Label(self.logfile_frame, textvariable=self.logfile_var,
                  foreground="#007700", font=("TkDefaultFont", 8)
                  ).pack(side=tk.LEFT, padx=4)

        # Case selector
        ttk.Label(ctrl, text="Case:").grid(row=4, column=0, sticky=tk.W, pady=(6, 2))
        case_opts = [CASES[k]["label"] for k in sorted(CASES)]
        self.case_cb = ttk.Combobox(ctrl, values=case_opts, state="readonly", width=36)
        self.case_cb.current(0)
        self.case_cb.grid(row=4, column=1, columnspan=5, sticky=tk.W, pady=(6, 2))
        self.case_cb.bind("<<ComboboxSelected>>", self._on_case_change)

        # Buttons
        btn_frame = ttk.Frame(ctrl)
        btn_frame.grid(row=5, column=0, columnspan=8, sticky=tk.W, pady=6)
        self.start_btn = ttk.Button(btn_frame, text="Start", command=self._start)
        self.start_btn.pack(side=tk.LEFT, padx=(0, 6))
        self.stop_btn = ttk.Button(btn_frame, text="Stop",
                                   command=self._stop, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btn_frame, text="Save PNG...",
                   command=self._save_png).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="Raw...",
                   command=self._open_raw_window).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(btn_frame, text="Send...",
                   command=self._open_send_window).pack(side=tk.LEFT, padx=(6, 0))

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _on_source_change(self):
        if self.source_var.get() == "serial":
            self.port_frame.grid()
            self.file_frame.grid_remove()
            self.logfile_frame.grid()
        else:
            self.port_frame.grid_remove()
            self.file_frame.grid()
            self.logfile_frame.grid_remove()
        self.format_var.set("--")
        self.logfile_var.set("")

    def _on_angle_toggle(self):
        """
        Called when the Degrees checkbutton changes.
        Rebuild the plot so channel labels and units update immediately.
        """
        val = self.show_degrees.get()
        self.status_var.set("Angle units: " + ("DEG" if val else "RAD"))
        if self.running:
            # Rebuild plot to update labels and derived conversions
            self._rebuild_plot()

    def _on_case_change(self, _=None):
        """
        Called when the Case combobox selection changes.

        - For manual (user) changes on a serial session: disable the combobox,
          send the "[<case>]\n" command on a background thread, wait for write
          result and only then update the UI/plot (or revert on failure).
        - For programmatic changes (self._suppress_case_send True) or when not
          in serial mode, apply immediately.
        """
        keys_sorted = sorted(CASES)
        idx = self.case_cb.current()
        if idx < 0 or idx >= len(keys_sorted):
            return
        new_case = keys_sorted[idx]
        prev_case = self.case_var.get()

        # Programmatic changes should not send command; just apply
        if getattr(self, "_suppress_case_send", False):
            self.case_var.set(new_case)
            if self.running:
                self._rebuild_plot()
            return

        # If not serial or no active serial thread, apply immediately
        if self.source_var.get() != "serial" or not self.serial_thread:
            self.case_var.set(new_case)
            if self.running:
                self._rebuild_plot()
            return

        # Manual change in serial mode: send command and wait for write result.
        # Disable combobox to prevent further user changes while send is in-flight.
        try:
            self.case_cb.config(state="disabled")
        except Exception:
            pass
        self.status_var.set(f"Sending case-change {new_case}...")

        # Background sender thread: perform write_text and marshal result back to UI thread.
        def _send_case_and_apply(case_to_send, prev):
            # Use square-bracket command format as requested
            cmd = f"[{case_to_send}]\n"
            ok = False
            try:
                ok = self.serial_thread.write_text(cmd)
            except Exception:
                ok = False

            def _on_result():
                # re-enable combobox
                try:
                    self.case_cb.config(state="readonly")
                except Exception:
                    pass

                if ok:
                    # Apply new case and rebuild plot (if running)
                    self.case_var.set(case_to_send)
                    if self.running:
                        self._rebuild_plot()
                    self.status_var.set(f"Sent -> {cmd.strip()}")
                    # record last sent command for user
                    try:
                        self.last_sent_var.set(cmd.strip())
                    except Exception:
                        pass
                else:
                    # revert combobox to previous selection and notify user
                    self.status_var.set("Send failed")
                    # restore combobox selection without re-sending
                    try:
                        self._suppress_case_send = True
                        prev_idx = keys_sorted.index(prev) if prev in keys_sorted else 0
                        self.case_cb.current(prev_idx)
                        self.case_var.set(prev)
                    finally:
                        self._suppress_case_send = False
                    messagebox.showerror("Send failed",
                                         f"Could not send case-change command: {cmd.strip()}")

            # schedule UI update on main thread
            try:
                self.after(0, _on_result)
            except Exception:
                _on_result()

        t = threading.Thread(target=_send_case_and_apply, args=(new_case, prev_case), daemon=True)
        t.start()

    def _browse(self):
        path = filedialog.askopenfilename(
            title="Open TCM log file",
            filetypes=[("Log files", "*.csv *.txt *.log *.bin"),
                       ("All files", "*.*")]
        )
        if path:
            self.file_path.set(path)

    # ── Present-value panel ────────────────────────────────────────────────────
    def _rebuild_pv_panel(self, channels):
        """Rebuild the right-hand present-value display for the given channels."""
        if not hasattr(self, "main_frame") or self.main_frame is None:
            self.main_frame = ttk.Frame(self)
            self.main_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        if not hasattr(self, "pv_frame") or self.pv_frame is None:
            self.pv_frame = ttk.LabelFrame(self.main_frame, text="Present Values", padding=10)
            self.pv_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(4, 6), pady=6)

        for w in self.pv_frame.winfo_children():
            w.destroy()
        self.pv_labels = []

        for ch in channels:
            sv = tk.StringVar(value="--")
            self.pv_labels.append(sv)
            row = ttk.Frame(self.pv_frame)
            row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=ch + ":", anchor=tk.W, width=20).pack(side=tk.LEFT)
            lbl = tk.Label(row, textvariable=sv,
                           font=("Courier", 11, "bold"),
                           fg="#0070c0",
                           width=14, anchor=tk.E)
            lbl.pack(side=tk.RIGHT)

    def _update_pv(self, parsed):
        """Update present-value StringVars with the latest sample."""
        if parsed is None:
            return
        for sv, val in zip(self.pv_labels, parsed):
            if isinstance(val, float):
                sv.set(f"{val:+.4f}")
            else:
                sv.set(str(val))

    # ── Raw window ────────────────────────────────────────────────────────────
    def _open_raw_window(self):
        if self.raw_win and tk.Toplevel.winfo_exists(self.raw_win):
            self.raw_win.lift()
            return
        self.raw_win = tk.Toplevel(self)
        self.raw_win.title("Raw Data")
        self.raw_win.protocol("WM_DELETE_WINDOW", self._close_raw_window)
        frm = ttk.Frame(self.raw_win, padding=6)
        frm.pack(fill=tk.BOTH, expand=True)
        top_row = ttk.Frame(frm)
        top_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Checkbutton(top_row, text="Autoscroll",
                        variable=self.raw_autoscroll).pack(side=tk.LEFT)
        ttk.Button(top_row, text="Clear",
                   command=self._clear_raw).pack(side=tk.RIGHT)
        self.raw_text = scrolledtext.ScrolledText(
            frm, wrap=tk.NONE, state=tk.DISABLED, height=20)
        try:
            self.raw_text.configure(background="#0f0f0f",
                                    foreground="#e6e6e6",
                                    insertbackground="#e6e6e6")
        except Exception:
            pass
        self.raw_text.pack(fill=tk.BOTH, expand=True)

    def _close_raw_window(self):
        if self.raw_win:
            try:
                self.raw_win.destroy()
            except Exception:
                pass
        self.raw_win  = None
        self.raw_text = None

    def _clear_raw(self):
        if not self.raw_text:
            return
        try:
            self.raw_text.configure(state=tk.NORMAL)
            self.raw_text.delete("1.0", tk.END)
        finally:
            self.raw_text.configure(state=tk.DISABLED)

    def _append_raw_line(self, line: str):
        if not self.raw_text:
            return
        try:
            self.raw_text.configure(state=tk.NORMAL)
            self.raw_text.insert(tk.END,
                                 line if line.endswith("\n") else line + "\n")
            try:
                total = int(self.raw_text.index("end-1c").split(".")[0])
                if total > RAW_WIN_MAX_LINES:
                    self.raw_text.delete("1.0", f"{total - RAW_WIN_MAX_LINES}.0")
            except Exception:
                pass
            if self.raw_autoscroll.get():
                self.raw_text.see(tk.END)
        finally:
            self.raw_text.configure(state=tk.DISABLED)

    # ── Send window ───────────────────────────────────────────────────────────
    def _open_send_window(self):
        if self.send_win and tk.Toplevel.winfo_exists(self.send_win):
            self.send_win.lift()
            return
        self.send_win = tk.Toplevel(self)
        self.send_win.title("Send Text")
        self.send_win.transient(self)
        frm = ttk.Frame(self.send_win, padding=6)
        frm.pack(fill=tk.BOTH, expand=True)
        ttk.Label(frm, text="Text:").grid(row=0, column=0, sticky=tk.W)
        self.send_entry = ttk.Entry(frm, width=50)
        self.send_entry.grid(row=0, column=1, sticky=tk.W, padx=(6, 0))
        self.send_entry.focus_set()
        ttk.Checkbutton(frm, text="Append newline",
                        variable=self.append_nl).grid(row=1, column=1, sticky=tk.W, pady=(6, 0))
        btn_row = ttk.Frame(frm)
        btn_row.grid(row=2, column=0, columnspan=2, pady=(8, 0), sticky=tk.E)
        ttk.Button(btn_row, text="Send",
                   command=self._on_send_clicked).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btn_row, text="Close",
                   command=self._close_send_window).pack(side=tk.LEFT)
        self.send_entry.bind("<Return>", lambda e: self._on_send_clicked())

    def _close_send_window(self):
        if self.send_win:
            try:
                self.send_win.destroy()
            except Exception:
                pass
        self.send_win   = None
        self.send_entry = None

    def _on_send_clicked(self):
        text = (self.send_entry.get() if self.send_entry else "")
        if self.append_nl.get():
            text += "\n"
        self._send_text_to_device(text)

    def _send_text_to_device(self, text: str):
        if self.source_var.get() != "serial":
            messagebox.showerror("Not serial",
                                 "Switch to Serial source to send text.")
            return
        if not self.serial_thread:
            messagebox.showerror("No serial",
                                 "Start a serial session first.")
            return
        ok = self.serial_thread.write_text(text)
        if ok:
            disp = text if len(text) <= 80 else text[:77] + "..."
            self.status_var.set(f"Sent -> {disp!r}")
            try:
                # store plain text (trim newline) for display
                self.last_sent_var.set(disp.rstrip("\n"))
            except Exception:
                pass
        else:
            messagebox.showerror("Send failed",
                                 "Could not write to serial port.")

    # ── Plot builder ───────────────────────────────────────────────────────────
    def _rebuild_plot(self):
        # Stop and remove existing animation/figure safely
        if self.anim:
            try:
                self.anim.event_source.stop()
            except Exception:
                pass
            self.anim = None
        if self.canvas_widget:
            try:
                self.canvas_widget.get_tk_widget().destroy()
            except Exception:
                pass
            try:
                plt.close(self.fig)
            except Exception:
                pass

        # Defensive containers
        if not hasattr(self, "main_frame") or self.main_frame is None:
            self.main_frame = ttk.Frame(self)
            self.main_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)
        if not hasattr(self, "plot_frame") or self.plot_frame is None:
            self.plot_frame = ttk.Frame(self.main_frame)
            self.plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        if not hasattr(self, "pv_frame") or self.pv_frame is None:
            self.pv_frame = ttk.LabelFrame(self.main_frame, text="Present Values", padding=10)
            self.pv_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(4, 6), pady=6)

        case_id  = self.case_var.get()
        meta     = CASES[case_id]
        channels = list(meta["channels"])

        # add derived magnitude channel for case 3 and case 4
        if case_id == 3:
            channels = channels + ["Acc magnitude (scaled)"]
        elif case_id == 4:
            channels = channels + ["Mag magnitude (scaled)"]

        # Adjust angle channel labels when toggling rad/deg
        if case_id == 1:
            if self.show_degrees.get():
                channels[0] = "Heading (deg)"
            else:
                channels[0] = "Heading (rad)"
        elif case_id == 2:
            if self.show_degrees.get():
                channels = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"]
            else:
                channels = ["Roll (rad)", "Pitch (rad)", "Yaw (rad)"]
            if case_id == 2 and (case_id == 3 or case_id == 4):
                pass  # no-op, kept for style parity

        n        = len(channels)
        colours  = plt.rcParams["axes.prop_cycle"].by_key()["color"]

        # Rebuild present-value panel for new channel set
        self._rebuild_pv_panel(channels)

        self.buffers = {ch: deque([0.0] * WINDOW, maxlen=WINDOW)
                        for ch in channels}
        xs = list(range(WINDOW))

        self.fig, axes_raw = plt.subplots(n, 1,
                                           figsize=(10, max(3, 2 * n)),
                                           sharex=True, tight_layout=True)
        self.fig.patch.set_facecolor("#1e1e1e")
        self.axes  = [axes_raw] if n == 1 else list(axes_raw)
        self.lines = []

        for i, (ax, ch) in enumerate(zip(self.axes, channels)):
            ax.set_facecolor("#2b2b2b")
            ax.set_ylabel(ch, color="#cccccc", fontsize=8)
            ax.tick_params(colors="#888888", labelsize=7)
            for sp in ax.spines.values():
                sp.set_edgecolor("#444444")
            ln, = ax.plot(xs, list(self.buffers[ch]),
                          color=colours[i % len(colours)], linewidth=1.2)
            self.lines.append(ln)

        self.axes[-1].set_xlabel("Sample", color="#cccccc", fontsize=8)
        self.fig.suptitle(meta["label"], color="white", fontsize=10)

        canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas_widget = canvas

        if self.source_var.get() == "serial":
            self.anim = animation.FuncAnimation(
                self.fig, self._animate, interval=INTERVAL,
                blit=False, cache_frame_data=False
            )
            canvas.draw()

    # ── Animation callback ─────────────────────────────────────────────────────
    def _animate(self, _frame):
        case_id = self.case_var.get()
        meta    = CASES[case_id]
        updated = False
        last_parsed = None

        while not self.data_queue.empty():
            item = self.data_queue.get_nowait()

            if item[0] == "ERROR":
                self.status_var.set(f"Serial error: {item[1]}")
                self._stop()
                break

            if item[0] == "STATUS":
                msg = item[1]
                self.status_var.set(msg)
                if "BINARY" in msg.upper():
                    self.format_var.set("BINARY")
                elif "TEXT" in msg.upper():
                    self.format_var.set("TEXT")
                continue

            if item[0] == "RAW":
                self._append_raw_line(item[1])
                continue

            if item[0] == "LOGFILE":
                self.logfile_var.set(os.path.basename(item[1]))
                continue

            # ("DATA", cid, values, fmt)
            _, cid, values, fmt = item

            # Calibration/config cases are logged to status only
            if cid in NON_PLOT_CASES:
                self.status_var.set(
                    f"Cal/config case {cid} received ({fmt.upper()})")
                continue

            # If we see data for a different plottable case, schedule a UI rebuild
            if cid != case_id:
                if cid in CASES:
                    if self._pending_auto_case != cid:
                        self._pending_auto_case = cid
                        try:
                            self.after(0, lambda c=cid: self._apply_pending_auto_case(c))
                        except Exception:
                            self._apply_pending_auto_case(cid)
                    continue
                else:
                    continue

            try:
                # parsed becomes a list of numeric values
                if fmt == "text":
                    parsed = meta["parse_text"](values)
                else:
                    parsed = [float(v) for v in values]

                # Angle unit convert for case 1 and 2 according to toggle:
                if case_id == 1:
                    # case 1 input is heading in degrees
                    if not self.show_degrees.get():
                        try:
                            parsed[0] = math.radians(parsed[0])
                        except Exception:
                            pass
                elif case_id == 2:
                    # case 2 input is radians
                    if self.show_degrees.get():
                        try:
                            parsed = [math.degrees(float(v)) for v in parsed]
                        except Exception:
                            parsed = [float(v) for v in parsed]

                # For case 3 and case 4 compute derived magnitude from scaled values (indices 3,4,5)
                if case_id == 3 or case_id == 4:
                    try:
                        sx = float(parsed[3])
                        sy = float(parsed[4])
                        sz = float(parsed[5])
                        mag = math.sqrt(sx * sx + sy * sy + sz * sz)
                    except Exception:
                        mag = 0.0
                    parsed = list(parsed) + [mag]

                # Determine channels list including derived channel for case 3 or 4
                channels = list(meta["channels"])
                if case_id == 3:
                    channels = channels + ["Acc magnitude (scaled)"]
                elif case_id == 4:
                    channels = channels + ["Mag magnitude (scaled)"]

                # Adjust channel names for angle toggle in runtime (keeps PV labels consistent)
                if case_id == 1:
                    if self.show_degrees.get():
                        channels[0] = "Heading (deg)"
                    else:
                        channels[0] = "Heading (rad)"
                elif case_id == 2:
                    if self.show_degrees.get():
                        channels = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"]
                    else:
                        channels = ["Roll (rad)", "Pitch (rad)", "Yaw (rad)"]

                # Ensure buffers exist and append values
                for ch, val in zip(channels, parsed):
                    if ch not in self.buffers:
                        self.buffers[ch] = deque([0.0] * WINDOW, maxlen=WINDOW)
                    self.buffers[ch].append(val)

                last_parsed = parsed
                updated = True
            except Exception:
                pass

        if updated:
            # Use channels matching current case (include derived if case 3/4)
            channels = list(meta["channels"])
            if case_id == 3:
                channels = channels + ["Acc magnitude (scaled)"]
            elif case_id == 4:
                channels = channels + ["Mag magnitude (scaled)"]
            if case_id == 1:
                if self.show_degrees.get():
                    channels[0] = "Heading (deg)"
                else:
                    channels[0] = "Heading (rad)"
            elif case_id == 2:
                if self.show_degrees.get():
                    channels = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"]
                else:
                    channels = ["Roll (rad)", "Pitch (rad)", "Yaw (rad)"]

            for ln, ch in zip(self.lines, channels):
                try:
                    ln.set_ydata(list(self.buffers[ch]))
                except Exception:
                    pass
            for ax in self.axes:
                try:
                    ax.relim()
                    ax.autoscale_view(scalex=False)
                except Exception:
                    pass
            if last_parsed is not None:
                self._update_pv(last_parsed)

        return self.lines

    # ── Start / Stop ───────────────────────────────────────────────────────────
    def _start(self):
        self.running = True
        self.format_var.set("--")
        self.logfile_var.set("")
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self._rebuild_plot()

        if self.source_var.get() == "serial":
            self._start_serial()
        else:
            self._load_file()

    def _start_serial(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("No port", "Select a serial port first.")
            self._stop()
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = BAUDRATE
        self.serial_stop.clear()
        self.serial_thread = SerialReader(
            port, baud, self.data_queue, self.serial_stop)
        self.serial_thread.start()
        self.status_var.set(f"Detecting format on {port} @ {baud} baud...")

    def _load_file(self):
        """Auto-detect binary vs text, load entire file, render static plot."""
        path = self.file_path.get()
        if not path or not os.path.isfile(path):
            messagebox.showerror("No file", "Select a valid log file first.")
            self._stop()
            return

        with open(path, "rb") as f:
            first = f.read(1)
        is_binary = bool(first and first[0] == TCM_BIN_SOF)
        fmt_label = "BINARY" if is_binary else "TEXT"
        self.format_var.set(fmt_label)

        case_id  = self.case_var.get()
        meta     = CASES[case_id]
        channels = list(meta["channels"])
        if case_id == 3:
            channels = channels + ["Acc magnitude (scaled)"]
        elif case_id == 4:
            channels = channels + ["Mag magnitude (scaled)"]
        # respect angle toggle for static loads
        if case_id == 1:
            if self.show_degrees.get():
                channels[0] = "Heading (deg)"
            else:
                channels[0] = "Heading (rad)"
        elif case_id == 2:
            if self.show_degrees.get():
                channels = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"]
            else:
                channels = ["Roll (rad)", "Pitch (rad)", "Yaw (rad)"]

        data     = {ch: [] for ch in channels}
        skipped  = 0

        if is_binary:
            reader = BinaryFrameReader()
            with open(path, "rb") as f:
                while True:
                    b = f.read(1)
                    if not b:
                        break
                    reader.push(b[0])
                    for cid, payload in reader.pop_frames():
                        if cid != case_id:
                            skipped += 1
                            continue
                        try:
                            parsed = CASES[cid]["parse_bin"](payload)
                            # angle conversion for static file
                            if cid == 1 and not self.show_degrees.get():
                                try:
                                    parsed[0] = math.radians(parsed[0])
                                except Exception:
                                    pass
                            if cid == 2 and self.show_degrees.get():
                                try:
                                    parsed = [math.degrees(float(v)) for v in parsed]
                                except Exception:
                                    parsed = [float(v) for v in parsed]

                            if cid == 3 or cid == 4:
                                try:
                                    sx, sy, sz = float(parsed[3]), float(parsed[4]), float(parsed[5])
                                    mag = math.sqrt(sx*sx + sy*sy + sz*sz)
                                except Exception:
                                    mag = 0.0
                                parsed = list(parsed) + [mag]

                            for ch, val in zip(channels, parsed):
                                data[ch].append(float(val))
                        except struct.error:
                            pass
        else:
            with open(path, "r", errors="replace") as f:
                for line in f:
                    result = parse_text_line(line)
                    if not result:
                        continue
                    cid, values = result
                    if cid != case_id:
                        skipped += 1
                        continue
                    try:
                        parsed = CASES[cid]["parse_text"](values)
                        # angle conversion for static file
                        if cid == 1 and not self.show_degrees.get():
                            try:
                                parsed[0] = math.radians(parsed[0])
                            except Exception:
                                pass
                        if cid == 2 and self.show_degrees.get():
                            try:
                                parsed = [math.degrees(float(v)) for v in parsed]
                            except Exception:
                                parsed = [float(v) for v in parsed]

                        if cid == 3 or cid == 4:
                            try:
                                sx, sy, sz = float(parsed[3]), float(parsed[4]), float(parsed[5])
                                mag = math.sqrt(sx*sx + sy*sy + sz*sz)
                            except Exception:
                                mag = 0.0
                            parsed = list(parsed) + [mag]

                        for ch, val in zip(channels, parsed):
                            data[ch].append(float(val))
                    except Exception:
                        pass

        n_samples = len(data[channels[0]]) if channels else 0
        if n_samples == 0:
            messagebox.showwarning(
                "No data",
                f"No case-{case_id} frames found in the file.")
            self._stop()
            return

        xs = list(range(n_samples))
        for ax, ln, ch in zip(self.axes, self.lines, channels):
            ln.set_xdata(xs)
            ln.set_ydata(data[ch])
            ax.set_xlim(0, n_samples - 1)
            ax.relim()
            ax.autoscale_view()

        self.canvas_widget.draw()

        # Show last sample as present values
        last = [data[ch][-1] for ch in channels]
        self._update_pv(last)

        self.status_var.set(
            f"[{fmt_label}] {n_samples} samples loaded  "
            f"({skipped} frames from other cases skipped)"
        )
        self._stop(keep_plot=True)

    def _stop(self, keep_plot=False):
        self.running = False
        self.serial_stop.set()
        if self.anim:
            try:
                self.anim.event_source.stop()
            except Exception:
                pass
            self.anim = None
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        if not keep_plot:
            self.status_var.set("Stopped.")

    def _save_png(self):
        if not self.fig:
            messagebox.showinfo("Nothing to save", "Start a capture first.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG", "*.png"), ("All files", "*.*")]
        )
        if path:
            self.fig.savefig(path, dpi=150,
                             facecolor=self.fig.get_facecolor())
            self.status_var.set(f"Saved -> {path}")

    def _on_close(self):
        self._stop()
        self._close_raw_window()
        self._close_send_window()
        self.destroy()

    def _apply_pending_auto_case(self, cid: int):
        """
        Apply an automatic case switch that was scheduled from the animation
        callback.  Run on the main/UI thread to avoid interfering with the
        running animation timer.
        """
        try:
            self._suppress_case_send = True
            keys_sorted = sorted(CASES)
            if cid in keys_sorted:
                idx = keys_sorted.index(cid)
                # update combobox and internal state without sending to device
                try:
                    self.case_cb.current(idx)
                except Exception:
                    pass
                self.case_var.set(cid)
                # rebuild plot for the new case
                self._rebuild_plot()
                self.status_var.set(f"Auto-switched to case {cid}")
        finally:
            self._suppress_case_send = False
            self._pending_auto_case = None

# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = TCMPlotter()
    app.mainloop()