#!/usr/bin/env python3
"""TIM3 Quadrature Encoder Monitor for N20 Geared Motor.

Procedure:
  1. TIM3 configured in Encoder mode (TI1 and TI2, x4 count).
  2. Prints counter value every 100 ms to serial terminal.
  3. Tape mark placed on output shaft; zero counter.
  4. Turn shaft by hand exactly 1 full turn and read count.
  5. Repeat 3-5 times and take average.

Usage:
  python smoketest/encoder/encoder_monitor.py [PORT] [BAUD]
"""
import sys
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
    print("  N20 Motor TIM3 Encoder Smoketest Monitor")
    print(f"  Port: {port} | Baud: {baud}")
    print("  Goal: 1 turn -> count = N_rev +- 2%, direction sign correct")
    print("=" * 65)

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except Exception as e:
        sys.exit(f"Error opening serial port: {e}")

    pattern = re.compile(r"TIM3_CNT=\s*(\d+)\s+\|\s+Total=([+-]?\d+)\s+\|\s+Dir=([A-Za-z0-9\(\)\+\-\s]+)")

    print("[*] Connected. Streaming 100 ms telemetry...")
    print("[*] Press Blue Button (B1) on Nucleo to Zero / Record trials.")
    print("[*] Press Ctrl+C to exit.\n")

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue

            match = pattern.search(line)
            if match:
                cnt = int(match.group(1))
                total = int(match.group(2))
                dir_str = match.group(3).strip()
                print(f"\r[100ms] TIM3_CNT: {cnt:5d}  |  Total Counts: {total:+7d}  |  Dir: {dir_str:<12s}", end="", flush=True)
            else:
                # Clear the live telemetry line and print the log line completely
                print(f"\r\033[K{line}")

    except KeyboardInterrupt:
        print("\n\n[*] Exiting monitor.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
