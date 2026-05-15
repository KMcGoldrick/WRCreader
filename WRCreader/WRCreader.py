"""
tcm_plotter.py
--------------
Reads and plots TCM sensor data from either a live serial (RS-485/UART)
port or a CSV log file.

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

Usage:
  pip install pyserial matplotlib
  python tcm_plotter.py
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import tkinter.scrolledtext as scrolledtext
import threading
import queue
import struct
import os

import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

# ── Constants ─────────────────────────────────────────────────────────────────
WINDOW      = 200       # samples shown in scrolling plot
BAUDRATE    = 115200
INTERVAL    = 50        # animation refresh ms
TCM_BIN_SOF = 0xAA
RAW_WIN_MAX_LINES = 2000

# ── Case metadata ──────────────────────────────────────────────────────────────
#
# parse_text(values: list[str]) -> list[number]
# parse_bin(payload: bytes)     -> list[number]
#
# Binary payload layouts (little-endian, matching tcmDataBinary):
#   case 1: fff           (headingDeg, north, east)
#   case 2: fff           (rollRad, pitchRad, yawRad)
#   case 3: hhh fff       (raw x/y/z int16, scaled x/y/z float)
#   case 4: hhh fff       (raw x/y/z int16, scaled x/y/z float)
#   case 5: HfHf          (raw temp uint16, scaled temp float,
#                          raw batt uint16, scaled batt float)

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
    """
    Sniffs the first SNIFF bytes of the stream.
    Binary signature: 0xAA followed by a byte in 0..11.
    If no binary signature found within SNIFF bytes, assumes text.
    """
    SNIFF = 8

    def __init__(self):
        self._buf   = bytearray()
        self.format = None      # "binary" or "text"

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
    """
    State-machine parser for:
      0xAA | case (1 byte) | length (1 byte) | payload (length bytes)
    Call push(byte) byte-by-byte; collect completed frames with pop_frames().
    """
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
    """Return (case_id, [str values]) or None."""
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


# ── Serial reader thread ───────────────────────────────────────────────────────
class SerialReader(threading.Thread):
    """
    Reads raw bytes, auto-detects format, dispatches to queue as:
      ("DATA",   case_id, values, fmt)   -- values already parsed for binary
      ("STATUS", message)
      ("ERROR",  message)
      ("RAW",    raw_string)             -- new: exact raw representation for UI
    """
    def __init__(self, port, baud, q: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.port       = port
        self.baud       = baud
        self.q          = q
        self.stop_event = stop_event

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                detector   = FormatDetector()
                bin_reader = BinaryFrameReader()
                text_buf   = bytearray()

                self.q.put(("STATUS", "Detecting frame format..."))

                while not self.stop_event.is_set():
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue

                    for byte in chunk:
                        if not detector.detected():
                            detector.feed(byte)
                            if detector.detected():
                                self.q.put(("STATUS",
                                    f"Format detected: {detector.format.upper()}"))

                        if not detector.detected():
                            continue

                        if detector.format == "binary":
                            bin_reader.push(byte)
                            for cid, payload in bin_reader.pop_frames():
                                # publish a RAW representation for the UI (hex payload)
                                try:
                                    raw_str = f"BIN,{cid},{payload.hex()}"
                                    self.q.put(("RAW", raw_str))
                                except Exception:
                                    pass
                                self._dispatch_binary(cid, payload)
                        else:
                            text_buf.append(byte)
                            if byte == ord('\n'):
                                line = text_buf.decode("ascii", errors="replace")
                                text_buf.clear()
                                # publish raw text line for UI
                                try:
                                    self.q.put(("RAW", line))
                                except Exception:
                                    pass
                                result = parse_text_line(line)
                                if result:
                                    cid, values = result
                                    self.q.put(("DATA", cid, values, "text"))

        except serial.SerialException as e:
            self.q.put(("ERROR", str(e)))

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
            pass    # malformed payload, skip silently


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

        self.serial_stop   = threading.Event()
        self.serial_thread = None
        self.data_queue    = queue.Queue()
        self.anim          = None
        self.buffers       = {}
        self.fig           = None
        self.axes          = []
        self.lines         = []
        self.canvas_widget = None
        self.running       = False

        # Raw window state
        self.raw_win   = None
        self.raw_text  = None
        self.raw_autoscroll = tk.BooleanVar(value=True)

        self._build_controls()
        self._refresh_ports()

    # ── UI ─────────────────────────────────────────────────────────────────────
    def _build_controls(self):
        ctrl = ttk.Frame(self, padding=8)
        ctrl.pack(side=tk.TOP, fill=tk.X)

        # Source radios
        ttk.Label(ctrl, text="Source:").grid(row=0, column=0, sticky=tk.W)
        ttk.Radiobutton(ctrl, text="Serial Port", variable=self.source_var,
                        value="serial", command=self._on_source_change
                        ).grid(row=0, column=1, sticky=tk.W)
        ttk.Radiobutton(ctrl, text="Log File", variable=self.source_var,
                        value="file", command=self._on_source_change
                        ).grid(row=0, column=2, sticky=tk.W)

        # Format indicator
        ttk.Label(ctrl, text="   Format:").grid(row=0, column=3, sticky=tk.W)
        ttk.Label(ctrl, textvariable=self.format_var,
                  foreground="#0070c0",
                  font=("TkDefaultFont", 9, "bold")
                  ).grid(row=0, column=4, sticky=tk.W)

        # Port row
        self.port_frame = ttk.Frame(ctrl)
        self.port_frame.grid(row=1, column=0, columnspan=6, sticky=tk.W, pady=2)
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
        self.file_frame.grid(row=2, column=0, columnspan=6, sticky=tk.W, pady=2)
        ttk.Label(self.file_frame, text="File:").pack(side=tk.LEFT)
        ttk.Entry(self.file_frame, textvariable=self.file_path, width=40
                  ).pack(side=tk.LEFT, padx=4)
        ttk.Button(self.file_frame, text="Browse...",
                   command=self._browse).pack(side=tk.LEFT)
        self.file_frame.grid_remove()

        # Case selector
        ttk.Label(ctrl, text="Case:").grid(row=3, column=0, sticky=tk.W, pady=(6, 2))
        case_opts = [CASES[k]["label"] for k in sorted(CASES)]
        self.case_cb = ttk.Combobox(ctrl, values=case_opts, state="readonly", width=28)
        self.case_cb.current(0)
        self.case_cb.grid(row=3, column=1, columnspan=3, sticky=tk.W, pady=(6, 2))
        self.case_cb.bind("<<ComboboxSelected>>", self._on_case_change)

        # Buttons
        btn_frame = ttk.Frame(ctrl)
        btn_frame.grid(row=4, column=0, columnspan=6, sticky=tk.W, pady=6)
        self.start_btn = ttk.Button(btn_frame, text="Start", command=self._start)
        self.start_btn.pack(side=tk.LEFT, padx=(0, 6))
        self.stop_btn = ttk.Button(btn_frame, text="Stop",
                                   command=self._stop, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btn_frame, text="Save PNG...",
                   command=self._save_png).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="Raw...", command=self._open_raw_window).pack(side=tk.LEFT, padx=(6,0))

        # Status bar
        ttk.Label(self, textvariable=self.status_var,
                  relief=tk.SUNKEN, anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X)

        # Plot area
        self.plot_frame = ttk.Frame(self)
        self.plot_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _on_source_change(self):
        if self.source_var.get() == "serial":
            self.port_frame.grid()
            self.file_frame.grid_remove()
        else:
            self.port_frame.grid_remove()
            self.file_frame.grid()
        self.format_var.set("--")

    def _on_case_change(self, _=None):
        idx = self.case_cb.current()
        self.case_var.set(sorted(CASES)[idx])
        if self.running:
            self._rebuild_plot()

    def _browse(self):
        path = filedialog.askopenfilename(
            title="Open TCM log file",
            filetypes=[("Log files", "*.csv *.txt *.log *.bin"),
                       ("All files", "*.*")]
        )
        if path:
            self.file_path.set(path)

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
        top_row.pack(fill=tk.X, pady=(0,6))
        ttk.Checkbutton(top_row, text="Autoscroll", variable=self.raw_autoscroll).pack(side=tk.LEFT)

        clear_btn = ttk.Button(top_row, text="Clear", command=self._clear_raw)
        clear_btn.pack(side=tk.RIGHT)

        self.raw_text = scrolledtext.ScrolledText(frm, wrap=tk.NONE, state=tk.DISABLED, height=20)
        try:
            self.raw_text.configure(background="#0f0f0f", foreground="#e6e6e6", insertbackground="#e6e6e6")
        except Exception:
            pass
        self.raw_text.pack(fill=tk.BOTH, expand=True)

    def _close_raw_window(self):
        if self.raw_win:
            try:
                self.raw_win.destroy()
            except Exception:
                pass
        self.raw_win = None
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
            self.raw_text.insert(tk.END, line if line.endswith("\n") else line + "\n")
            # trim old lines if necessary
            try:
                total_lines = int(self.raw_text.index("end-1c").split(".")[0])
                if total_lines > RAW_WIN_MAX_LINES:
                    delete_to = total_lines - RAW_WIN_MAX_LINES
                    self.raw_text.delete("1.0", f"{delete_to}.0")
            except Exception:
                pass
            if self.raw_autoscroll.get():
                self.raw_text.see(tk.END)
        finally:
            self.raw_text.configure(state=tk.DISABLED)

    # ── Plot builder ───────────────────────────────────────────────────────────
    def _rebuild_plot(self):
        if self.anim:
            self.anim.event_source.stop()
            self.anim = None
        if self.canvas_widget:
            self.canvas_widget.get_tk_widget().destroy()
            plt.close(self.fig)

        case_id  = self.case_var.get()
        meta     = CASES[case_id]
        channels = meta["channels"]
        n        = len(channels)
        colours  = plt.rcParams["axes.prop_cycle"].by_key()["color"]

        self.buffers = {ch: deque([0.0] * WINDOW, maxlen=WINDOW) for ch in channels}
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
            # use blit=False to avoid backend resize issues in some environments
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

            # ("DATA", cid, values, fmt)
            _, cid, values, fmt = item

            if cid in NON_PLOT_CASES:
                self.status_var.set(
                    f"Cal/config case {cid} received ({fmt.upper()})")
                continue

            if cid != case_id:
                continue

            try:
                if fmt == "text":
                    parsed = meta["parse_text"](values)
                else:
                    parsed = [float(v) for v in values]

                for ch, val in zip(meta["channels"], parsed):
                    self.buffers[ch].append(val)
                updated = True
            except Exception:
                pass

        if updated:
            for ln, ch in zip(self.lines, meta["channels"]):
                ln.set_ydata(list(self.buffers[ch]))
            for ax in self.axes:
                ax.relim()
                ax.autoscale_view(scalex=False)

        return self.lines

    # ── Start / Stop ───────────────────────────────────────────────────────────
    def _start(self):
        self.running = True
        self.format_var.set("--")
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

        # Peek at first byte to decide format
        with open(path, "rb") as f:
            first = f.read(1)
        is_binary = bool(first and first[0] == TCM_BIN_SOF)
        fmt_label = "BINARY" if is_binary else "TEXT"
        self.format_var.set(fmt_label)

        case_id  = self.case_var.get()
        meta     = CASES[case_id]
        channels = meta["channels"]
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
                            parsed = meta["parse_bin"](payload)
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
                        parsed = meta["parse_text"](values)
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
        self.status_var.set(
            f"[{fmt_label}] {n_samples} samples loaded  "
            f"({skipped} frames from other cases skipped)"
        )
        self._stop(keep_plot=True)

    def _stop(self, keep_plot=False):
        self.running = False
        self.serial_stop.set()
        if self.anim:
            self.anim.event_source.stop()
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
        self.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = TCMPlotter()
    app.mainloop()