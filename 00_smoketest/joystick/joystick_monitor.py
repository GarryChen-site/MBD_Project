#!/usr/bin/env python3
"""KY-023 Dual-Axis Joystick Serial Monitor & Acceptance Test.

Goal:
  1. Centre noise: sigma <= 10 counts (~8 mV) at rest
  2. Full travel: X and Y read 0 - 4095 over full motion
  3. Pushbutton SW toggles between 1 (released) and 0 (pressed)

Usage:
  python joystick/joystick_monitor.py [PORT] [BAUD]
  e.g.: python joystick/joystick_monitor.py /dev/cu.usbmodem* 115200
"""
import sys
import time
import glob
import re
import serial

DEFAULT_BAUD = 115200


def auto_detect_port() -> str:
    ports = glob.glob("/dev/cu.usbmodem*")
    if ports:
        return ports[0]
    sys.exit("Error: No STM32 /dev/cu.usbmodem* port found. Please specify manually.")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else auto_detect_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    print("=" * 65)
    print("  KY-023 Joystick Acceptance Test (PC Monitor)")
    print("  Connecting to:", port, "at", baud, "baud")
    print("=" * 65)

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except Exception as e:
        sys.exit(f"Error opening serial port: {e}")

    pattern = re.compile(r"X=\s*(\d+)\s+Y=\s*(\d+)\s+\|\s+SW=(\w+)")

    min_x, max_x = 4095, 0
    min_y, max_y = 4095, 0
    btn_pressed = False

    print("[*] Listening for STM32 joystick output...")
    print("[*] Press Ctrl+C at any time to finish and see travel summary.\n")

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue

            # Pass through boot banners or noise reports directly
            if "[" in line or "=" in line:
                print(line)

            match = pattern.search(line)
            if match:
                x = int(match.group(1))
                y = int(match.group(2))
                sw_str = match.group(3)

                if x < min_x: min_x = x
                if x > max_x: max_x = x
                if y < min_y: min_y = y
                if y > max_y: max_y = y
                if "PRESSED" in sw_str: btn_pressed = True

                # Visual mini-bar
                bx = int(x / 4095.0 * 20)
                by = int(y / 4095.0 * 20)
                bar_x = "[" + "=" * bx + " " * (20 - bx) + "]"
                bar_y = "[" + "=" * by + " " * (20 - by) + "]"

                print(f"\rX:{x:4d} {bar_x} | Y:{y:4d} {bar_y} | SW:{sw_str:7s} | Range X:[{min_x:4d}..{max_x:4d}] Y:[{min_y:4d}..{max_y:4d}]", end="", flush=True)

    except KeyboardInterrupt:
        print("\n\n" + "=" * 65)
        print("  TRAVEL ACCEPTANCE SUMMARY")
        print("=" * 65)
        print(f"X Axis Range : {min_x} .. {max_x} (Expected ~0 .. ~4095)")
        print(f"Y Axis Range : {min_y} .. {max_y} (Expected ~0 .. ~4095)")
        print(f"SW Button    : {'DETECTED PRESSED' if btn_pressed else 'NOT DETECTED'}")
        print("-" * 65)

        pass_x = (min_x <= 150) and (max_x >= 3900)
        pass_y = (min_y <= 150) and (max_y >= 3900)

        print(f"Full Travel X : {'PASS' if pass_x else 'WARN'} (min={min_x}, max={max_x})")
        print(f"Full Travel Y : {'PASS' if pass_y else 'WARN'} (min={min_y}, max={max_y})")
        print(f"Button SW     : {'PASS' if btn_pressed else 'WARN'}")
        print("=" * 65)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
