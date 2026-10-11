#!/usr/bin/env python3
"""TB6612FNG + N20 Motor Smoketest PC Monitor.

Milestone Targets:
  1. 20 kHz PWM verified on Logic Analyzer (or audible silence)
  2. Duty cycle 0 -> 50% ramp spins motor in expected direction
  3. STBY pin LOW stops motor immediately

Usage:
  python smoketest/motor/motor_monitor.py [PORT] [BAUD]
"""
import sys
import glob
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
    print("  TB6612 + N20 Motor Bring-Up Telemetry Monitor")
    print(f"  Port: {port} | Baud: {baud}")
    print("  Target: 20 kHz PWM, Duty 0->50% spin, STBY shutdown")
    print("=" * 65)

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except Exception as e:
        sys.exit(f"Error opening serial port: {e}")

    print("[*] Connected. Streaming test output...")
    print("[*] Press Blue Button (B1) on Nucleo to re-run test sequence.")
    print("[*] Press Ctrl+C to exit.\n")

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(line)
    except KeyboardInterrupt:
        print("\n\n[*] Exiting monitor.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
