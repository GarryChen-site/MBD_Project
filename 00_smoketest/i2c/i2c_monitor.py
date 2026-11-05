#!/usr/bin/env python3
"""Shared I2C Bus Smoketest PC Telemetry Monitor.

Targets:
  1. I2C Bus Scan detects VL53L0X (0x29) and MMA7660 (0x4C) ACK
  2. MMA7660 reads X, Y, Z acceleration and orientation posture
  3. VL53L0X measures 3 benchmark distances (100, 200, 300 mm) within +/- 10% of a ruler

Usage:
  python smoketest/i2c/i2c_monitor.py [PORT] [BAUD]
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
    print("  Shared I2C Bus Bring-Up Monitor (VL53L0X + MMA7660)")
    print(f"  Port: {port} | Baud: {baud}")
    print("  Targets: 0x29 & 0x4C ACK, 3 Distances within +/- 10%")
    print("=" * 65)

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except Exception as e:
        sys.exit(f"Error opening serial port: {e}")

    print("[*] Connected. Streaming I2C telemetry...")
    print("[*] Press Blue Button (B1) on Nucleo to start/re-run 3-distance test.")
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
