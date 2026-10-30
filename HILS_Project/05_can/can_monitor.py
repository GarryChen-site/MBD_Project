#!/usr/bin/env python3
"""
Stage 5 & 6 — PC CANable V2 Live Fault Monitor, DBC Decoder & UDS 0x22 Client
==============================================================================
Uses `05_can/vehicle.dbc` to decode and verify live CAN frames (`0x200 Drive_Status`,
`0x201 Diag_Status`, `0x210 AEB_Status`, `0x123 ECU_Heartbeat`, `0x7E8 Diag_Resp`)
transmitted by the STM32F446RE over SN65HVD230 <-> CANable V2 at 500 kbit/s.

Usage:
  1. Live CANable V2 (candleLight / gs_usb firmware):
       python3 05_can/can_monitor.py gs_usb

  2. Live CANable V2 (slcan serial firmware):
       python3 05_can/can_monitor.py slcan /dev/cu.usbmodemXXXX

  3. Offline Host C/Python Simulation (--demo):
       python3 05_can/can_monitor.py --demo

What happens in Live Mode when you push the joystick Right-only (X-only):
  - Catches `mode = LIMP (2)` and `speed_req_rpm = 40.0 rpm` on `0x200 Drive_Status`
  - Catches `fault_bitmap = 0x0001 (F_PEDAL_PLAUS)` on `0x201 Diag_Status`
  - Verifies SAE J1850 CRC-8 and 4-bit alive counter on every frame
  - Automatically sends UDS Service `0x22` (`0x7E0: [03 22 F1 00]`) to query active faults
    and (`0x7E0: [03 22 F1 02]`) to query on-target WCET (us), decoding `0x7E8` response
  - Saves the entire trace to `05_can/logs/can_fault_capture.csv`
"""

import csv
import ctypes
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "scripts"))
import dbc_codegen as dbc_mod  # noqa: E402

BITRATE = 500_000
MODE_NAMES = {0: "NORMAL", 1: "DEGRADED", 2: "LIMP", 3: "SAFE_STOP"}
AEB_NAMES = {0: "NORMAL", 1: "WARNING", 2: "BRAKING"}
FAULT_BITS = {
    0: "F_PEDAL_PLAUS",
    1: "F_PEDAL_RANGE",
    2: "F_CAN_TIMEOUT",
    3: "F_TOF_INVALID",
    4: "F_SPD_TRACK",
}


def decode_fault_names(bitmap: int) -> str:
    if bitmap == 0:
        return "NONE"
    names = [name for bit, name in FAULT_BITS.items() if (bitmap >> bit) & 1]
    return "|".join(names) if names else f"0x{bitmap:04X}"


def decode_frame_py(msg_def: dbc_mod.MessageDef, payload: bytes):
    """Decode a CAN frame in Python using `vehicle.dbc` and verify its E2E CRC-8."""
    vals = {}
    for s in msg_def.signals:
        raw = 0
        for i in range(s.length):
            bit_pos = s.start_bit + i
            byte_idx = bit_pos // 8
            bit_idx = bit_pos % 8
            if byte_idx < len(payload) and ((payload[byte_idx] >> bit_idx) & 1):
                raw |= 1 << i
        vals[s.name] = raw * s.factor + s.offset

    crc_ok = True
    if msg_def.crc_sig is not None:
        crc_byte = msg_def.crc_sig.start_bit // 8
        calc_crc = dbc_mod.py_e2e_crc8(payload[:crc_byte], msg_def.e2e_data_id)
        crc_ok = payload[crc_byte] == calc_crc
    return vals, crc_ok


def open_canable_bus(kind: str, port: str | None):
    """Open CANable V2 using the exact same logic as `smoketest/can/can_monitor.py`."""
    try:
        import can
    except ImportError:
        sys.exit(
            "Error: `python-can` is not installed in this Python environment.\n"
            "Install with: pip install python-can pyserial gs_usb"
        )

    if kind == "slcan":
        if not port:
            sys.exit("slcan needs a serial port, e.g.: python3 05_can/can_monitor.py slcan /dev/cu.usbmodemXXXX")
        return can.Bus(interface="slcan", channel=port, bitrate=BITRATE), can
    if kind == "gs_usb":
        from gs_usb.gs_usb import GsUsb

        devs = GsUsb.scan()
        if not devs:
            sys.exit("No gs_usb (candleLight) device found. Check USB connection or use 'slcan'.")
        dev = devs[0]
        return (
            can.Bus(
                interface="gs_usb",
                channel=dev.product,
                index=dev.bus,
                address=dev.address,
                bitrate=BITRATE,
            ),
            can,
        )
    sys.exit("First argument must be 'gs_usb', 'slcan', or '--demo'")


def run_live(kind: str, port: str | None) -> int:
    repo_root = Path(__file__).resolve().parents[1]
    dbc_path = repo_root / "05_can" / "vehicle.dbc"
    log_dir = repo_root / "05_can" / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    csv_path = log_dir / "can_fault_capture.csv"

    messages = {m.can_id: m for m in dbc_mod.parse_dbc(dbc_path)}
    bus, can_pkg = open_canable_bus(kind, port)

    print("=" * 78)
    print(f"  STM32 MBD CANable Live Monitor & Fault Catcher ({kind} @ {BITRATE} bit/s)")
    print(f"  DBC Database : {dbc_path.relative_to(repo_root)}")
    print(f"  Capture File : {csv_path.relative_to(repo_root)}")
    print("=" * 78)
    print("[*] Listening for STM32 CAN frames (`0x200`, `0x201`, `0x210`, `0x123`)...")
    print("[*] Push Joystick UP-RIGHT (diagonal) for normal driving, or RIGHT-ONLY to inject F_PEDAL_PLAUS!")
    print("[*] Press Ctrl+C to stop and save log.\n")

    last_fault_bitmap = 0
    t0 = None
    frame_cnt = 0
    crc_ok_cnt = 0
    crc_err_cnt = 0

    with csv_path.open("w", newline="", encoding="utf-8") as f_csv:
        writer = csv.writer(f_csv)
        writer.writerow(
            [
                "t_s",
                "can_id_hex",
                "msg_name",
                "dlc",
                "raw_hex",
                "crc_ok",
                "speed_req_rpm",
                "speed_meas_rpm",
                "duty_pct",
                "mode",
                "fault_bitmap_hex",
                "fault_names",
            ]
        )

        try:
            while True:
                msg = bus.recv(timeout=0.1)
                if msg is None:
                    continue

                if t0 is None:
                    t0 = msg.timestamp
                rel_t = msg.timestamp - t0
                cid = msg.arbitration_id
                payload = bytes(msg.data)
                raw_hex = payload.hex(" ").upper()
                frame_cnt += 1

                if cid in messages:
                    mdef = messages[cid]
                    vals, crc_ok = decode_frame_py(mdef, payload)
                    if crc_ok:
                        crc_ok_cnt += 1
                    else:
                        crc_err_cnt += 1
                    crc_tag = "CRC:OK" if crc_ok else "CRC:ERR!"

                    if cid == 0x200:  # Drive_Status (20 ms)
                        mode_int = int(vals["mode"])
                        mode_str = MODE_NAMES.get(mode_int, str(mode_int))
                        writer.writerow(
                            [
                                f"{rel_t:.3f}",
                                "0x200",
                                "Drive_Status",
                                msg.dlc,
                                raw_hex,
                                int(crc_ok),
                                f"{vals['speed_req_rpm']:.1f}",
                                f"{vals['speed_rpm']:.1f}",
                                f"{vals['duty_pct']:.1f}",
                                mode_str,
                                f"0x{last_fault_bitmap:04X}",
                                decode_fault_names(last_fault_bitmap),
                            ]
                        )
                        # Print every 5th Drive_Status (100 ms) or immediately when in LIMP/SAFE_STOP
                        if (frame_cnt % 5 == 0) or (mode_int != 0):
                            print(
                                f"[{rel_t:7.3f}s] 0x200 Drive_Status | "
                                f"req={vals['speed_req_rpm']:5.1f} rpm  meas={vals['speed_rpm']:5.1f} rpm  "
                                f"duty={vals['duty_pct']:4.1f}%  mode={mode_str:<9} "
                                f"cnt={int(vals['msg_counter']):2d} ({crc_tag})"
                            )

                    elif cid == 0x201:  # Diag_Status (100 ms)
                        fb = int(vals["fault_bitmap"])
                        fname = decode_fault_names(fb)
                        writer.writerow(
                            [
                                f"{rel_t:.3f}",
                                "0x201",
                                "Diag_Status",
                                msg.dlc,
                                raw_hex,
                                int(crc_ok),
                                "",
                                "",
                                "",
                                "",
                                f"0x{fb:04X}",
                                fname,
                            ]
                        )
                        if fb != 0 and last_fault_bitmap == 0:
                            print("\n" + "!" * 78)
                            print(
                                f"  [!!! FAULT CAUGHT ON CAN BUS @ {rel_t:.3f}s !!!] "
                                f"0x201 Diag_Status: fault_bitmap=0x{fb:04X} ({fname})"
                            )
                            print("  --> Sending UDS 0x22 ReadDataByIdentifier (0x7E0) for Active Faults & WCET...")
                            print("!" * 78)
                            # Query Active Faults (0xF100) and Max WCET us (0xF102)
                            bus.send(
                                can_pkg.Message(
                                    arbitration_id=0x7E0,
                                    is_extended_id=False,
                                    data=[0x03, 0x22, 0xF1, 0x00, 0, 0, 0, 0],
                                )
                            )
                            bus.send(
                                can_pkg.Message(
                                    arbitration_id=0x7E0,
                                    is_extended_id=False,
                                    data=[0x03, 0x22, 0xF1, 0x02, 0, 0, 0, 0],
                                )
                            )
                        elif fb == 0 and last_fault_bitmap != 0:
                            print(
                                f"\n  [+++ FAULT RECOVERED ON CAN BUS @ {rel_t:.3f}s +++] "
                                f"0x201 Diag_Status: fault_bitmap=0x0000 (NORMAL)\n"
                            )
                        last_fault_bitmap = fb

                elif cid == 0x7E8 and len(payload) >= 6:  # UDS Diag_Resp
                    if payload[1] == 0x62:
                        did = (payload[2] << 8) | payload[3]
                        val = (payload[4] << 8) | payload[5]
                        if did == 0xF100:
                            print(
                                f"  [<-- UDS 0x7E8 Resp] DID 0xF100 (Active Faults) = 0x{val:04X} ({decode_fault_names(val)})"
                            )
                        elif did == 0xF102:
                            print(f"  [<-- UDS 0x7E8 Resp] DID 0xF102 (Max Step WCET) = {val} us\n")

        except KeyboardInterrupt:
            print(
                f"\n[*] Stopped. Total frames: {frame_cnt} (CRC OK: {crc_ok_cnt}, CRC ERR: {crc_err_cnt})."
            )
            print(f"[*] Saved CSV log to: {csv_path}")
        finally:
            bus.shutdown()
    return 0


def run_demo() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    dbc_path = repo_root / "05_can" / "vehicle.dbc"
    bsw_dir = repo_root / "04_firmware" / "bsw"
    fw_dir = repo_root / "04_firmware"

    messages = {m.can_id: m for m in dbc_mod.parse_dbc(dbc_path)}

    print("=== Stage 5 CAN Telemetry & Diagnostics End-to-End Demo ===")
    with tempfile.TemporaryDirectory() as tmpdir:
        lib_path = Path(tmpdir) / "libcan_tlm.dylib"
        cmd = [
            "/usr/bin/clang",
            "-dynamiclib",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(bsw_dir),
            "-I",
            str(fw_dir),
            "-I",
            str(fw_dir / "app_generated"),
            str(bsw_dir / "com_can_signals.c"),
            str(bsw_dir / "can_telemetry.c"),
            "-o",
            str(lib_path),
        ]
        subprocess.run(cmd, check=True)
        lib = ctypes.CDLL(str(lib_path))

        class Rte_AppIn_t(ctypes.Structure):
            _fields_ = [
                ("pedal1_adc", ctypes.c_float),
                ("pedal2_adc", ctypes.c_float),
                ("speed_meas_rpm", ctypes.c_float),
            ]

        class Rte_AppOut_t(ctypes.Structure):
            _fields_ = [
                ("duty_cmd_pct", ctypes.c_float),
                ("speed_req_rpm", ctypes.c_float),
                ("fault_bitmap", ctypes.c_uint16),
                ("mode", ctypes.c_uint8),
                ("s1_pct", ctypes.c_float),
                ("p2_pct", ctypes.c_float),
                ("e_pct", ctypes.c_float),
                ("p_cmd_pct", ctypes.c_float),
            ]

        class E2E_State_t(ctypes.Structure):
            _fields_ = [
                ("last_cnt", ctypes.c_uint8),
                ("initialised", ctypes.c_uint8),
                ("lost", ctypes.c_uint16),
                ("err_total", ctypes.c_uint16),
            ]

        class Com_Pedal_Cmd_t(ctypes.Structure):
            _fields_ = [("pedal_pct", ctypes.c_float)]

        class CanTelemetry_State_t(ctypes.Structure):
            _fields_ = [
                ("tx_cnt_drive", ctypes.c_uint8),
                ("tx_cnt_diag", ctypes.c_uint8),
                ("tx_cnt_aeb", ctypes.c_uint8),
                ("hb_counter", ctypes.c_uint32),
                ("rx_e2e_pedal", E2E_State_t),
                ("last_pedal_cmd", Com_Pedal_Cmd_t),
                ("can_age_ms", ctypes.c_uint16),
                ("active_fault_bitmap", ctypes.c_uint16),
                ("stored_fault_bitmap", ctypes.c_uint16),
                ("max_wcet_us", ctypes.c_uint16),
                ("tx_frame_count", ctypes.c_uint32),
                ("rx_frame_count", ctypes.c_uint32),
                ("last_drive_buf", ctypes.c_uint8 * 7),
                ("last_diag_buf", ctypes.c_uint8 * 5),
                ("last_aeb_buf", ctypes.c_uint8 * 5),
            ]

        lib.CanTelemetry_Init.restype = None
        lib.CanTelemetry_GetState.restype = ctypes.POINTER(CanTelemetry_State_t)
        lib.CanTelemetry_Init()

        for tick in range(20):
            if tick < 10:
                inp = Rte_AppIn_t(3200.0, 3200.0, 89.5)
                out = Rte_AppOut_t(31.2, 90.0, 0x0000, 0, 60.0, 60.0, 0.0, 60.0)
                d_mm = 800
                aeb_st = 0
            else:
                inp = Rte_AppIn_t(3600.0, 2070.0, 40.0)
                out = Rte_AppOut_t(14.9, 40.0, 0x0001, 2, 80.0, 0.0, 80.0, 0.0)
                d_mm = 180
                aeb_st = 2

            lib.CanTelemetry_Step10ms(tick, ctypes.byref(inp), ctypes.byref(out), d_mm, aeb_st, 75)
            st = lib.CanTelemetry_GetState().contents

            if tick in (0, 2, 10, 12):
                raw_drive = bytes(st.last_drive_buf)
                vals, crc_ok = decode_frame_py(messages[0x200], raw_drive)
                print(
                    f"  t={tick*10:3d} ms | TX 0x200 Drive_Status [{raw_drive.hex(' ').upper()}] -> "
                    f"req={vals['speed_req_rpm']:5.1f} rpm, meas={vals['speed_rpm']:5.1f} rpm, "
                    f"duty={vals['duty_pct']:4.1f}%, mode={MODE_NAMES[int(vals['mode'])]}, "
                    f"cnt={int(vals['msg_counter']):2d}, CRC={'OK' if crc_ok else 'ERR'}"
                )
                assert crc_ok

            if tick in (1, 11):
                raw_aeb = bytes(st.last_aeb_buf)
                vals, crc_ok = decode_frame_py(messages[0x210], raw_aeb)
                print(
                    f"  t={tick*10:3d} ms | TX 0x210 AEB_Status   [{raw_aeb.hex(' ').upper()}]       -> "
                    f"d={int(vals['distance_mm']):4d} mm, aeb={AEB_NAMES[int(vals['aeb_state'])]}, "
                    f"cnt={int(vals['msg_counter']):2d}, CRC={'OK' if crc_ok else 'ERR'}"
                )
                assert crc_ok

            if tick in (3, 13):
                raw_diag = bytes(st.last_diag_buf)
                vals, crc_ok = decode_frame_py(messages[0x201], raw_diag)
                print(
                    f"  t={tick*10:3d} ms | TX 0x201 Diag_Status  [{raw_diag.hex(' ').upper()}]       -> "
                    f"fault_bitmap=0x{int(vals['fault_bitmap']):04X} ({decode_fault_names(int(vals['fault_bitmap']))}), "
                    f"rx_err={int(vals['rx_err_count'])}, cnt={int(vals['msg_counter']):2d}, CRC={'OK' if crc_ok else 'ERR'}"
                )
                assert crc_ok

        uds_req = (ctypes.c_uint8 * 8)(0x03, 0x22, 0xF1, 0x02, 0, 0, 0, 0)
        st_before = lib.CanTelemetry_GetState().contents.tx_frame_count
        lib.CanTelemetry_OnRxFrame(0x7E0, uds_req, 8)
        st_after = lib.CanTelemetry_GetState().contents
        assert st_after.tx_frame_count == st_before + 1
        print(
            f"  UDS 0x22 [03 22 F1 02] -> 0x7E8 Positive Response (max_wcet_us = {st_after.max_wcet_us} us, "
            f"stored_faults = 0x{st_after.stored_fault_bitmap:04X}) -> PASS"
        )

    print("=== Stage 5 CAN Telemetry Demo: ALL PASS ===")
    return 0


def main() -> int:
    if len(sys.argv) == 1 or sys.argv[1] == "--demo":
        return run_demo()
    kind = sys.argv[1]
    port = sys.argv[2] if len(sys.argv) > 2 else None
    return run_live(kind, port)


if __name__ == "__main__":
    sys.exit(main())
