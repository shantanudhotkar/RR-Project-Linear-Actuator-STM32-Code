"""
X/Z stage GUI  -  pip install pyserial   (tkinter ships with Python)

UART PROTOCOL (115200 8N1, '\n' terminated, ASCII)  -- firmware must match this

PC -> STM32
  PC,<seq>        PC heartbeat, every 200 ms (STM32 stops + logs position if it stops coming)
  X<mm>           X absolute move, e.g.  X20.870
  Z0 / Z1         Z to home position / Z to fixed extended position
  HX / HZ         home axis: retract until counts stop, that position = 0 (also clears WARN/FAULT)
  S               stop everything
  LOG?            read the position saved in flash after a heartbeat loss

STM32 -> PC
  HB,<seq>,<xs>,<x_um>,<zs>,<z_um>      heartbeat every 500 ms, state: I idle, H homing, M moving, S settle, W warn, E fault
  OK,<cmd...>  /  NAK,<reason>          reply to every command
  DONE,<axis>,<target_um>,<final_um>,<err_um>,<brake_counts>
  HOMED,<axis>,<zero_err_um>
  WARN,<axis>,OVERLIMIT,<final_um>,<target_um>,<err_um>    overshoot > 0.5 mm -> please home
  STALL,<axis>,<pos_um>
  FAULT,<axis>,<reason>,<pos_um>
  INFO,<text...>
  LOG,<valid>,<x_um>,<z_um>
"""
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

BAUD = 115200
PC_HB_PERIOD_MS = 200
STM_HB_TIMEOUT = 2.0
X_MAX_MM = 100.0            # set to the real X travel

STATE_NAME = {"I": "IDLE", "H": "HOMING", "M": "MOVING", "S": "SETTLING",
              "W": "WARNING - HOME", "E": "FAULT", "?": "?"}


class App:
    def __init__(self, root):
        self.root = root
        root.title("X / Z Stage Controller")
        self.ser = None
        self.rx_q = queue.Queue()
        self.reader_run = False
        self.pc_seq = 0
        self.last_stm_hb = 0.0
        self.warn_open = False
        self.pending_return = {}          # axis -> target_um to go back to after re-homing

        self.var_port = tk.StringVar()
        self.var_x_mm = tk.StringVar(value="5.000")
        self.var_alive = tk.StringVar(value="NOT CONNECTED")
        self.var_state = {"X": tk.StringVar(value="?"), "Z": tk.StringVar(value="?")}
        self.var_pos = {"X": tk.StringVar(value="--.---"), "Z": tk.StringVar(value="--.---")}
        self.var_last = {"X": tk.StringVar(value="-"), "Z": tk.StringVar(value="-")}

        self._build_ui()
        self.refresh_ports()
        self.root.after(30, self.poll)
        self.root.after(PC_HB_PERIOD_MS, self.pc_heartbeat)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------ UI
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")
        ttk.Label(top, text="Port:").pack(side="left")
        self.cb_port = ttk.Combobox(top, textvariable=self.var_port, width=12, state="readonly")
        self.cb_port.pack(side="left", padx=4)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.btn_conn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.btn_conn.pack(side="left", padx=6)
        self.lbl_alive = tk.Label(top, textvariable=self.var_alive, width=22, fg="white", bg="gray40")
        self.lbl_alive.pack(side="right")

        mid = ttk.Frame(self.root, padding=6)
        mid.pack(fill="x")

        fx = ttk.LabelFrame(mid, text="X axis (any position)", padding=8)
        fx.grid(row=0, column=0, padx=6, sticky="nsew")
        ttk.Label(fx, text="Target (mm):").grid(row=0, column=0, sticky="w")
        ttk.Entry(fx, textvariable=self.var_x_mm, width=10).grid(row=0, column=1, padx=4)
        ttk.Button(fx, text="Go to", command=self.x_go).grid(row=0, column=2, padx=4)
        ttk.Button(fx, text="Home X", command=lambda: self.send("HX")).grid(row=0, column=3, padx=4)
        ttk.Label(fx, text="Position (mm):").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Label(fx, textvariable=self.var_pos["X"], font=("Consolas", 16, "bold")).grid(row=1, column=1, columnspan=2, sticky="w")
        ttk.Label(fx, text="State:").grid(row=2, column=0, sticky="w")
        ttk.Label(fx, textvariable=self.var_state["X"]).grid(row=2, column=1, columnspan=2, sticky="w")
        ttk.Label(fx, text="Last move:").grid(row=3, column=0, sticky="w")
        ttk.Label(fx, textvariable=self.var_last["X"]).grid(row=3, column=1, columnspan=3, sticky="w")

        fz = ttk.LabelFrame(mid, text="Z axis (fixed 0 / 1)", padding=8)
        fz.grid(row=0, column=1, padx=6, sticky="nsew")
        ttk.Button(fz, text="Z = 0  (home pos)", command=lambda: self.send("Z0")).grid(row=0, column=0, padx=4)
        ttk.Button(fz, text="Z = 1  (extended)", command=lambda: self.send("Z1")).grid(row=0, column=1, padx=4)
        ttk.Button(fz, text="Home Z", command=lambda: self.send("HZ")).grid(row=0, column=2, padx=4)
        ttk.Label(fz, text="Position (mm):").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Label(fz, textvariable=self.var_pos["Z"], font=("Consolas", 16, "bold")).grid(row=1, column=1, sticky="w")
        ttk.Label(fz, text="State:").grid(row=2, column=0, sticky="w")
        ttk.Label(fz, textvariable=self.var_state["Z"]).grid(row=2, column=1, sticky="w")
        ttk.Label(fz, text="Last move:").grid(row=3, column=0, sticky="w")
        ttk.Label(fz, textvariable=self.var_last["Z"]).grid(row=3, column=1, columnspan=2, sticky="w")

        bot = ttk.Frame(self.root, padding=6)
        bot.pack(fill="x")
        tk.Button(bot, text="STOP", bg="red3", fg="white", font=("Arial", 14, "bold"),
                  command=lambda: self.send("S"), width=10).pack(side="left", padx=6)
        ttk.Button(bot, text="Home All", command=self.home_all).pack(side="left", padx=6)
        ttk.Button(bot, text="Read stop log", command=lambda: self.send("LOG?")).pack(side="left", padx=6)

        self.txt = tk.Text(self.root, height=14, width=90, state="disabled", font=("Consolas", 9))
        self.txt.pack(fill="both", expand=True, padx=6, pady=6)

    def log(self, s):
        self.txt.configure(state="normal")
        self.txt.insert("end", time.strftime("%H:%M:%S ") + s + "\n")
        self.txt.see("end")
        self.txt.configure(state="disabled")

    # ------------------------------------------------------------ serial
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_port["values"] = ports
        if ports and self.var_port.get() not in ports:
            self.var_port.set(ports[0])

    def toggle_connect(self):
        if self.ser:
            self.disconnect()
        else:
            try:
                self.ser = serial.Serial(self.var_port.get(), BAUD, timeout=0.1)
            except (serial.SerialException, OSError) as e:
                messagebox.showerror("Serial", str(e))
                self.ser = None
                return
            self.pc_seq = 0
            self.last_stm_hb = 0.0
            self.reader_run = True
            threading.Thread(target=self.reader, daemon=True).start()
            self.btn_conn.configure(text="Disconnect")
            self.log(f"connected to {self.var_port.get()}")

    def disconnect(self):
        self.reader_run = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.btn_conn.configure(text="Connect")
        self.var_alive.set("NOT CONNECTED")
        self.lbl_alive.configure(bg="gray40")
        self.log("disconnected (STM32 will stop on heartbeat loss)")

    def reader(self):
        buf = b""
        ser = self.ser
        while self.reader_run and ser:
            try:
                buf += ser.read(64)
            except (serial.SerialException, OSError):
                self.rx_q.put("!!SERIAL_ERROR")
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode(errors="replace").strip()
                if line:
                    self.rx_q.put(line)

    def send(self, text, quiet=False):
        if not self.ser:
            self.log("not connected")
            return False
        try:
            self.ser.write((text + "\n").encode())
        except (serial.SerialException, OSError) as e:
            self.log(f"write error: {e}")
            self.disconnect()
            return False
        if not quiet:
            self.log(f"> {text}")
        return True

    def pc_heartbeat(self):
        self.root.after(PC_HB_PERIOD_MS, self.pc_heartbeat)
        if self.ser:
            self.pc_seq += 1
            self.send(f"PC,{self.pc_seq}", quiet=True)

    # ------------------------------------------------------------ commands
    def x_go(self):
        try:
            mm = float(self.var_x_mm.get().replace(",", "."))
        except ValueError:
            messagebox.showerror("X", "Enter a number in mm")
            return
        if mm < 0 or mm > X_MAX_MM:
            messagebox.showerror("X", f"Target must be 0 .. {X_MAX_MM} mm")
            return
        self.pending_return.pop("X", None)
        self.send(f"X{mm:.3f}")

    def home_all(self):
        self.send("HX")
        self.send("HZ")

    # ------------------------------------------------------------ receive handling
    def poll(self):
        self.root.after(30, self.poll)
        alive = self.ser is not None and (time.time() - self.last_stm_hb) < STM_HB_TIMEOUT
        if self.ser:
            self.var_alive.set("STM32 ALIVE" if alive else "NO STM32 HEARTBEAT")
            self.lbl_alive.configure(bg="green4" if alive else "red3")
        try:
            while True:
                line = self.rx_q.get_nowait()
                if line == "!!SERIAL_ERROR":
                    self.log("serial error")
                    self.disconnect()
                    return
                self.handle(line)
        except queue.Empty:
            pass

    def handle(self, line):
        f = line.split(",")
        tag = f[0]
        try:
            if tag == "HB":
                self.last_stm_hb = time.time()
                for ax, si, pi in (("X", 2, 3), ("Z", 4, 5)):
                    self.var_state[ax].set(STATE_NAME.get(f[si], f[si]))
                    self.var_pos[ax].set(f"{int(f[pi]) / 1000.0:.3f}")
                return
            self.log("< " + line)
            if tag == "DONE":
                ax = f[1]
                self.var_last[ax].set(f"target {int(f[2])/1000:.3f}  final {int(f[3])/1000:.3f}  "
                                      f"err {int(f[4])} um  brake {f[5]}")
            elif tag == "HOMED":
                ax = f[1]
                self.var_last[ax].set(f"homed, zero error {int(f[2])} um")
                if ax in self.pending_return:
                    tgt_um = self.pending_return.pop(ax)
                    self.log(f"re-homed {ax}, returning to previous target")
                    if ax == "X":
                        self.send(f"X{tgt_um / 1000.0:.3f}")
                    else:
                        self.send("Z1" if tgt_um > 0 else "Z0")
            elif tag == "WARN":
                self.on_warn(f[1], int(f[3]), int(f[4]), int(f[5]))
            elif tag == "STALL":
                self.log(f"!! STALL on {f[1]} at {int(f[2])/1000:.3f} mm")
            elif tag == "FAULT":
                self.log(f"!! FAULT on {f[1]}: {f[2]} at {int(f[3])/1000:.3f} mm")
            elif tag == "LOG":
                if f[1] == "1":
                    self.log(f"stop log: X = {int(f[2])/1000:.3f} mm, Z = {int(f[3])/1000:.3f} mm")
                else:
                    self.log("stop log: empty")
        except (IndexError, ValueError):
            self.log("[bad line] " + line)

    def on_warn(self, axis, final_um, target_um, err_um):
        if self.warn_open:
            return
        self.warn_open = True
        try:
            ok = messagebox.askyesno(
                "Overshoot limit exceeded",
                f"{axis} axis overshot by {err_um / 1000:.3f} mm (limit 0.5 mm).\n"
                f"Target {target_um / 1000:.3f} mm, reached {final_um / 1000:.3f} mm.\n\n"
                "Please do homing.\nRe-home now and return to the target?")
        finally:
            self.warn_open = False
        if ok:
            self.pending_return[axis] = target_um
            self.send("H" + axis)
        else:
            self.log(f"{axis}: warning ignored, no homing done")

    def on_close(self):
        self.reader_run = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()