#!/usr/bin/env python3
"""capture_log.py - Auto-detecting STM32 serial logger for Stage 2 calibration & Stage 6 closed-loop HIL.

Usage (auto-detects /dev/cu.usbmodem* and defaults to 03_simulink/data/real):
    python3 04_firmware/experiments/capture_log.py

Or specify port and output directory explicitly:
    python3 04_firmware/experiments/capture_log.py /dev/cu.usbmodemXXXX 03_simulink/data/real
"""
import glob
import os
import sys
import serial  # pip install pyserial

DEFAULT_BAUD = 115200


def auto_detect_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if ports:
        return ports[0]
    sys.exit("Error: No STM32 /dev/cu.usbmodem* port found. Plug in the Nucleo USB cable and retry.")


def default_outdir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "..", "03_simulink", "data", "real"))


def main() -> int:
    port = sys.argv[1] if len(sys.argv) >= 2 else auto_detect_port()
    outdir = sys.argv[2] if len(sys.argv) >= 3 else default_outdir()
    os.makedirs(outdir, exist_ok=True)

    print("=" * 70)
    print("  STM32 MBD Calibration & Closed-Loop Telemetry Logger")
    print(f"  Serial Port : {port} @ {DEFAULT_BAUD} baud")
    print(f"  Output Dir  : {outdir}")
    print("=" * 70)
    print("[*] Connected! Press the Blue Button (B1) on the Nucleo to start.")
    print("[*] Press Ctrl+C at any time to exit.\n")

    f = None
    cur_name = ""
    line_count = 0

    with serial.Serial(port, DEFAULT_BAUD, timeout=1.0) as ser:
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if line.startswith("#BEGIN"):
                    parts = line.split(maxsplit=1)
                    cur_name = parts[1] if len(parts) > 1 else "capture.csv"
                    out_path = os.path.join(outdir, cur_name)
                    f = open(out_path, "w", newline="")
                    line_count = 0
                    print(f"\n>>> [CAPTURING] {cur_name} -> {out_path}")
                elif line.startswith("#END") and f:
                    f.close()
                    f = None
                    print(f"\r>>> [SAVED]     {cur_name} ({line_count} rows)                    ")
                elif line.startswith("#"):
                    print(line)
                elif f:
                    f.write(line + "\n")
                    line_count += 1
                    if line_count % 25 == 0:
                        print(f"\r    Recording {cur_name}: {line_count} rows... ({line})", end="", flush=True)
                else:
                    print(line)
        except KeyboardInterrupt:
            print("\n\n[*] Stopped by user.")
            if f:
                f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
