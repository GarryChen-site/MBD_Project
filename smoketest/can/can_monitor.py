#!/usr/bin/env python3
"""PC-side CAN monitor / sender for CANable V2 (500 kbit/s).

Install:   pip install python-can pyserial gs_usb      (macOS: brew install libusb)

Usage:
  python can_monitor.py slcan  /dev/cu.usbmodemXXXX     # CANable with slcan firmware
  python can_monitor.py gs_usb                           # CANable with candleLight firmware

While running:  prints every frame, and sends CMD 0x321 every 2 s.
The STM32 should reply with ACK 0x322 and toggle LD2.
"""
import sys
import time

import can

BITRATE = 500_000
ID_HEARTBEAT, ID_CMD, ID_ACK = 0x123, 0x321, 0x322


def open_bus(kind: str, port: str | None) -> can.BusABC:
    if kind == "slcan":
        if not port:
            sys.exit("slcan needs a serial port, e.g. /dev/cu.usbmodem1101")
        return can.Bus(interface="slcan", channel=port, bitrate=BITRATE)
    if kind == "gs_usb":
        from gs_usb.gs_usb import GsUsb

        devs = GsUsb.scan()
        if not devs:
            sys.exit("No gs_usb (candleLight) device found")
        dev = devs[0]
        return can.Bus(
            interface="gs_usb",
            channel=dev.product,
            index=dev.bus,
            address=dev.address,
            bitrate=BITRATE,
        )
    sys.exit("first argument must be 'slcan' or 'gs_usb'")


def main() -> None:
    kind = sys.argv[1] if len(sys.argv) > 1 else "gs_usb"
    port = sys.argv[2] if len(sys.argv) > 2 else None
    bus = open_bus(kind, port)
    print(f"Connected via {kind} @ {BITRATE} bit/s. Ctrl+C to stop.")

    last_cmd = 0.0
    n = 0
    try:
        while True:
            msg = bus.recv(timeout=0.1)
            if msg is not None:
                print(f"{msg.timestamp:.3f}  id=0x{msg.arbitration_id:03X} "
                      f"dlc={msg.dlc} data={msg.data.hex(' ')}")
            if time.time() - last_cmd > 2.0:
                last_cmd = time.time()
                n = (n + 1) & 0xFF
                bus.send(can.Message(arbitration_id=ID_CMD, is_extended_id=False,
                                     data=[0xA0, n]))
                print(f"--> sent CMD 0x{ID_CMD:03X} [A0 {n:02X}]")
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
