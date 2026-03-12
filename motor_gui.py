#!/usr/bin/env python3
import argparse
import queue
import re
import threading
import tkinter as tk
from tkinter import ttk

import serial


def serial_reader(ser, q):
    while True:
        try:
            raw = ser.readline()
        except Exception:
            return
        if raw:
            q.put(raw.decode(errors="replace").strip())


class MotorGui:
    def __init__(self, root, ser):
        self.root = root
        self.ser = ser
        self.q = queue.Queue()
        self.state = {
            1: {
                "angle": tk.StringVar(value="-- deg"),
                "machine": tk.StringVar(value="idle"),
                "human": tk.StringVar(value="idle"),
                "in_range": True,
            },
            2: {
                "angle": tk.StringVar(value="-- deg"),
                "machine": tk.StringVar(value="idle"),
                "human": tk.StringVar(value="idle"),
                "in_range": True,
            },
        }

        self._build_ui()
        threading.Thread(target=serial_reader, args=(self.ser, self.q), daemon=True).start()
        self.send("status")
        self.poll()

    def _build_ui(self):
        self.root.title("Dual Motor Assist Status")
        self.root.geometry("980x360")
        self.root.configure(bg="#0f172a")

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#0f172a")
        style.configure("Card.TFrame", background="#1e293b")
        style.configure("CardTitle.TLabel", background="#1e293b", foreground="#cbd5e1", font=("Helvetica", 12, "bold"))
        style.configure("CardValue.TLabel", background="#1e293b", foreground="#f8fafc", font=("Helvetica", 20, "bold"))
        style.configure("StatusBar.TLabel", background="#14532d", foreground="#ecfdf5", font=("Helvetica", 12, "bold"), padding=8)

        wrapper = ttk.Frame(self.root, padding=14)
        wrapper.pack(fill=tk.BOTH, expand=True)

        self.status_bar = ttk.Label(wrapper, text="SYSTEM STATUS: ALL IN RANGE", style="StatusBar.TLabel", anchor=tk.CENTER)
        self.status_bar.pack(fill=tk.X, pady=(0, 12))

        ttk.Label(wrapper, text="Motor 1", style="CardTitle.TLabel").pack(anchor=tk.W, pady=(0, 6))
        row1 = ttk.Frame(wrapper)
        row1.pack(fill=tk.BOTH, expand=True, pady=(0, 12))
        self._card(row1, "Angle", self.state[1]["angle"]).pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        self._card(row1, "Machine", self.state[1]["machine"]).pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8)
        self._card(row1, "Human", self.state[1]["human"]).pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))

        ttk.Label(wrapper, text="Motor 2", style="CardTitle.TLabel").pack(anchor=tk.W, pady=(0, 6))
        row2 = ttk.Frame(wrapper)
        row2.pack(fill=tk.BOTH, expand=True)
        self._card(row2, "Angle", self.state[2]["angle"]).pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        self._card(row2, "Machine", self.state[2]["machine"]).pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8)
        self._card(row2, "Human", self.state[2]["human"]).pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))

    def _card(self, parent, title, value_var):
        frame = ttk.Frame(parent, style="Card.TFrame", padding=14)
        ttk.Label(frame, text=title, style="CardTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(frame, textvariable=value_var, style="CardValue.TLabel").pack(anchor=tk.W, pady=(14, 0))
        return frame

    def send(self, cmd):
        try:
            self.ser.write((cmd + "\n").encode())
        except Exception:
            pass

    def _parse_stat(self, line):
        mid = re.search(r"id=([0-9]+)", line)
        if not mid:
            return
        motor_id = int(mid.group(1))
        if motor_id not in self.state:
            return

        rel = re.search(r"rel=([-0-9.]+)deg", line)
        machine_dir = re.search(r"machine_dir=([a-zA-Z]+)", line)
        intent_dir = re.search(r"intent_dir=([-0-9]+)", line)
        in_range = re.search(r"in_range=([0-9]+)", line)

        if rel:
            deg = float(rel.group(1))
            angle_text = f"{deg:.1f} deg"
            if in_range and in_range.group(1) == "0":
                angle_text += " (OUT OF RANGE)"
            self.state[motor_id]["angle"].set(angle_text)
        if in_range:
            self.state[motor_id]["in_range"] = (in_range.group(1) == "1")
            self._update_status_bar()

        if machine_dir:
            d = machine_dir.group(1).lower()
            if d == "forward":
                self.state[motor_id]["machine"].set("forward")
            elif d == "backward":
                self.state[motor_id]["machine"].set("backward")
            else:
                self.state[motor_id]["machine"].set("idle")

        if intent_dir:
            v = int(intent_dir.group(1))
            if v > 0:
                self.state[motor_id]["human"].set("forward")
            elif v < 0:
                self.state[motor_id]["human"].set("backward")
            else:
                self.state[motor_id]["human"].set("idle")

    def _update_status_bar(self):
        m1_ok = self.state[1]["in_range"]
        m2_ok = self.state[2]["in_range"]

        if m1_ok and m2_ok:
            text = "SYSTEM STATUS: ALL IN RANGE"
            bg = "#14532d"
            fg = "#ecfdf5"
        elif (not m1_ok) and (not m2_ok):
            text = "SYSTEM STATUS: BOTH MOTORS OUT OF RANGE"
            bg = "#7f1d1d"
            fg = "#fef2f2"
        elif not m1_ok:
            text = "SYSTEM STATUS: MOTOR 1 OUT OF RANGE"
            bg = "#7f1d1d"
            fg = "#fef2f2"
        else:
            text = "SYSTEM STATUS: MOTOR 2 OUT OF RANGE"
            bg = "#7f1d1d"
            fg = "#fef2f2"

        self.status_bar.configure(text=text, background=bg, foreground=fg)

    def poll(self):
        while not self.q.empty():
            line = self.q.get_nowait()
            if "STAT " in line:
                self._parse_stat(line)
        self.root.after(50, self.poll)


def main():
    ap = argparse.ArgumentParser(description="Minimal motor status UI")
    ap.add_argument("--port", default="/dev/cu.usbmodem101")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    root = tk.Tk()
    MotorGui(root, ser)

    def on_close():
        try:
            ser.close()
        finally:
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
