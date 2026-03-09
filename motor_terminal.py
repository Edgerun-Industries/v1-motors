#!/usr/bin/env python3
import argparse
import queue
import threading

import serial


def reader(ser, q):
    while True:
        try:
            line = ser.readline()
        except Exception as exc:
            q.put(f"[serial error] {exc}")
            return
        if line:
            q.put(line.decode(errors="replace").rstrip())


def main():
    ap = argparse.ArgumentParser(description="Single motor serial console")
    ap.add_argument("--port", default="/dev/cu.usbmodem101")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    q = queue.Queue()
    threading.Thread(target=reader, args=(ser, q), daemon=True).start()

    print(f"Connected: {args.port} @ {args.baud}")
    print("Commands: ping, status, id <n>, scan_ids, enable, spin_once cw, spin_once ccw, stop, disable, help")
    print("Local commands: /raw on, /raw off, /quit")
    print("Type 'quit' to exit.")

    for cmd in ["ping", "status"]:
        ser.write((cmd + "\n").encode())

    show_raw = False

    try:
        while True:
            while not q.empty():
                line = q.get_nowait()
                if (not show_raw) and ("RX_RAW" in line):
                    continue
                print(line)

            cmd = input("> ").strip()
            if not cmd:
                continue
            if cmd in ("quit", "/quit"):
                break
            if cmd == "/raw on":
                show_raw = True
                print("[local] RX_RAW display enabled")
                continue
            if cmd == "/raw off":
                show_raw = False
                print("[local] RX_RAW display hidden")
                continue
            if cmd == "scan":
                ser.write(b"scan_ids\n")
                continue
            ser.write((cmd + "\n").encode())
    finally:
        ser.close()
        print("Closed.")


if __name__ == "__main__":
    main()
