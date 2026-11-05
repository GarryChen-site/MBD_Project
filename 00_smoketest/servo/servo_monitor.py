#!/usr/bin/env python3
"""
SG90 Micro Servo + Concurrent Motor Telemetry Monitor
Reads serial telemetry from STM32F446RE over USART2 at 115200 baud.
"""

import sys
import time
import serial
import serial.tools.list_ports

def find_stm32_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "STM" in port.description or "STLink" in port.description or "usbmodem" in port.device:
            return port.device
    if ports:
        return ports[0].device
    return None

def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else find_stm32_port()
    if not port_name:
        print("[ERROR] No serial port detected! Connect your STM32 Nucleo via USB.")
        sys.exit(1)

    print(f"=======================================================")
    print(f"  SG90 SERVO & MOTOR SMOKETEST TELEMETRY MONITOR       ")
    print(f"  Port: {port_name} | Baud: 115200                   ")
    print(f"=======================================================\n")

    try:
        ser = serial.Serial(port_name, 115200, timeout=1.0)
    except Exception as e:
        print(f"[ERROR] Could not open {port_name}: {e}")
        sys.exit(1)

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='replace')
            if line:
                print(line, end='', flush=True)
    except KeyboardInterrupt:
        print("\n[INFO] Monitor stopped by user.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
