#!/usr/bin/env python3
import argparse
import queue
import threading
import tkinter as tk
from tkinter import ttk

import serial


IMPORTANT_KEYS = ("EVT", "STAT", "WARN", "ERR", "BUS")


def serial_reader(ser, q):
    while True:
        try:
            raw = ser.readline()
        except Exception as exc:
            q.put(f"[serial error] {exc}")
            return
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        q.put(line)


def extract_important(line):
    for key in IMPORTANT_KEYS:
        idx = line.find(key)
        if idx >= 0:
            return line[idx:]
    return None


class MotorGui:
    def __init__(self, root, ser):
        self.root = root
        self.ser = ser
        self.q = queue.Queue()

        self.mode_var = tk.StringVar(value="assist")
        self.on_var = tk.StringVar(value="OFF")
        self.status_var = tk.StringVar(value="Connecting...")

        self._build_ui()
        threading.Thread(target=serial_reader, args=(self.ser, self.q), daemon=True).start()
        self.send("status")
        self.poll_logs()

    def _build_ui(self):
        self.root.title("v1 Motor Control")
        self.root.geometry("700x480")

        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Power").grid(row=0, column=0, sticky=tk.W, padx=(0, 8))
        ttk.Button(top, text="ON", command=self.on).grid(row=0, column=1, padx=4)
        ttk.Button(top, text="OFF", command=self.off).grid(row=0, column=2, padx=4)
        ttk.Label(top, textvariable=self.on_var).grid(row=0, column=3, sticky=tk.W, padx=(8, 24))

        ttk.Label(top, text="Mode").grid(row=0, column=4, sticky=tk.W, padx=(0, 8))
        ttk.Radiobutton(
            top,
            text="Transparent",
            value="transparent",
            variable=self.mode_var,
            command=self.set_mode,
        ).grid(row=0, column=5, padx=4)
        ttk.Radiobutton(
            top,
            text="Assist",
            value="assist",
            variable=self.mode_var,
            command=self.set_mode,
        ).grid(row=0, column=6, padx=4)

        ttk.Button(top, text="ZERO", command=lambda: self.send("zero")).grid(row=0, column=7, padx=(16, 4))
        ttk.Button(top, text="STATUS", command=lambda: self.send("status")).grid(row=0, column=8, padx=4)

        mid = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        mid.pack(fill=tk.X)
        ttk.Label(mid, textvariable=self.status_var).pack(anchor=tk.W)

        log_frame = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log = tk.Text(log_frame, height=20, wrap=tk.WORD, state=tk.DISABLED)
        self.log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log.yview)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.log.configure(yscrollcommand=sb.set)

    def append_log(self, msg):
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, msg + "\n")
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def send(self, cmd):
        try:
            self.ser.write((cmd + "\n").encode())
        except Exception as exc:
            self.append_log(f"[send error] {exc}")

    def on(self):
        self.send("on")

    def off(self):
        self.send("off")

    def set_mode(self):
        self.send(f"mode {self.mode_var.get()}")

    def _update_from_line(self, line):
        if line.startswith("EVT on"):
            self.on_var.set("ON")
        elif line.startswith("EVT off"):
            self.on_var.set("OFF")
        elif line.startswith("EVT mode=assist"):
            self.mode_var.set("assist")
        elif line.startswith("EVT mode=transparent"):
            self.mode_var.set("transparent")
        elif line.startswith("STAT "):
            self.status_var.set(line)

    def poll_logs(self):
        while not self.q.empty():
            line = self.q.get_nowait()
            important = extract_important(line)
            if important is None:
                continue
            self._update_from_line(important)
            self.append_log(important)
        self.root.after(50, self.poll_logs)


def main():
    ap = argparse.ArgumentParser(description="Minimal exoskeleton motor GUI")
    ap.add_argument("--port", default="/dev/cu.usbmodem101")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    root = tk.Tk()
    app = MotorGui(root, ser)

    def on_close():
        app.send("off")
        ser.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
