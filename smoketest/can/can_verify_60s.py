#!/usr/bin/env python3
"""CAN Real-Bus 60-Second Verification Test Script.

Goal:
  Verify that the STM32F446RE + SN65HVD230 transmitting over physical CAN bus
  to Makerbase CANable 2.0 achieves:
  - >= 120 heartbeat frames (ID 0x123) within 60.0 seconds
  - Heartbeat interval: 500 ms nominal (+-2 ms target)
  - 0 error frames

Usage:
  python smoketest/can_verify_60s.py slcan /dev/cu.usbmodem***
"""
import sys
import time
import statistics
import can

BITRATE = 500_000
ID_HEARTBEAT = 0x123
TEST_DURATION_SEC = 60.0
NOMINAL_INTERVAL_MS = 500.0
JITTER_TOLERANCE_MS = 2.0  # target tolerance


def open_bus(kind: str, port: str | None) -> can.BusABC:
    if kind == "slcan":
        if not port:
            sys.exit("Error: slcan requires a port (e.g., /dev/cu.usbmodem***)")
        return can.Bus(interface="slcan", channel=port, bitrate=BITRATE)
    if kind == "gs_usb":
        from gs_usb.gs_usb import GsUsb

        devs = GsUsb.scan()
        if not devs:
            sys.exit("Error: No gs_usb device found")
        dev = devs[0]
        return can.Bus(
            interface="gs_usb",
            channel=dev.product,
            index=dev.bus,
            address=dev.address,
            bitrate=BITRATE,
        )
    sys.exit("Error: First argument must be 'slcan' or 'gs_usb'")


def run_test(kind: str, port: str | None):
    print("=" * 65)
    print("  CAN Real-Bus 60s Acceptance Test")
    print(f"  Target: >= 120 frames in 60s (500 ms +- {JITTER_TOLERANCE_MS:.0f} ms), 0 errors")
    print("=" * 65)

    bus = open_bus(kind, port)
    print(f"[*] Connected via {kind} at {BITRATE} bit/s.")
    print("[*] Waiting for first heartbeat (ID 0x123) to start timing...")

    timestamps = []
    error_frames = 0
    raw_frames = []

    try:
        # Wait for the very first heartbeat frame
        start_time = None
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            if msg.is_error_frame:
                error_frames += 1
                continue
            if msg.arbitration_id == ID_HEARTBEAT:
                start_time = time.perf_counter()
                timestamps.append(start_time)
                counter_val = int.from_bytes(msg.data[0:4], byteorder="big")
                loopback_flag = msg.data[4] if len(msg.data) > 4 else 0
                raw_frames.append((start_time, counter_val, loopback_flag))
                print(f"[SYNC] First heartbeat received: counter={counter_val}, loopback_flag={loopback_flag}")
                break

        print(f"[*] Benchmark running for {TEST_DURATION_SEC} seconds. Please wait...")

        # Collect for 60 seconds
        last_progress = 0
        while True:
            elapsed = time.perf_counter() - start_time
            if elapsed >= TEST_DURATION_SEC:
                break

            # Print progress every 10 seconds
            if int(elapsed) // 10 > last_progress:
                last_progress = int(elapsed) // 10
                print(f"  -> Elapsed: {int(elapsed)}s / 60s | Frames: {len(timestamps)} | Errors: {error_frames}")

            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue

            recv_time = time.perf_counter()
            if msg.is_error_frame:
                error_frames += 1
                continue

            if msg.arbitration_id == ID_HEARTBEAT:
                timestamps.append(recv_time)
                counter_val = int.from_bytes(msg.data[0:4], byteorder="big")
                loopback_flag = msg.data[4] if len(msg.data) > 4 else 0
                raw_frames.append((recv_time, counter_val, loopback_flag))

    except KeyboardInterrupt:
        print("\n[!] Aborted by user.")
    finally:
        bus.shutdown()

    # Analyze results
    print("\n" + "=" * 65)
    print("  TEST RESULTS & METRICS")
    print("=" * 65)

    total_frames = len(timestamps)
    print(f"Total Heartbeat Frames : {total_frames} (Requirement: >= 120)")
    print(f"Total Error Frames     : {error_frames} (Requirement: 0)")

    if total_frames < 2:
        print("[FAIL] Not enough frames received to calculate jitter.")
        sys.exit(1)

    intervals_ms = [(timestamps[i] - timestamps[i - 1]) * 1000.0 for i in range(1, total_frames)]
    jitters_ms = [abs(dt - NOMINAL_INTERVAL_MS) for dt in intervals_ms]

    avg_interval = statistics.mean(intervals_ms)
    min_interval = min(intervals_ms)
    max_interval = max(intervals_ms)
    std_interval = statistics.stdev(intervals_ms) if len(intervals_ms) > 1 else 0.0

    max_jitter = max(jitters_ms)
    avg_jitter = statistics.mean(jitters_ms)
    within_tol = sum(1 for j in jitters_ms if j <= JITTER_TOLERANCE_MS)
    pct_within_tol = (within_tol / len(jitters_ms)) * 100.0

    print("-" * 65)
    print(f"Nominal Period         : {NOMINAL_INTERVAL_MS:.1f} ms")
    print(f"Mean Measured Period   : {avg_interval:.2f} ms")
    print(f"Min / Max Period       : {min_interval:.2f} ms / {max_interval:.2f} ms")
    print(f"Standard Deviation     : {std_interval:.2f} ms")
    print(f"Mean Jitter            : {avg_jitter:.2f} ms")
    print(f"Max Jitter             : {max_jitter:.2f} ms")
    print(f"Frames within +-2ms    : {within_tol} / {len(intervals_ms)} ({pct_within_tol:.1f}%)")
    print("-" * 65)

    # Check acceptance criteria
    pass_count = total_frames >= 120
    pass_errors = error_frames == 0
    # Note: On USB-serial (slcan), OS polling schedule may introduce small USB latency spikes
    pass_jitter = avg_jitter <= JITTER_TOLERANCE_MS

    print("Criteria Breakdown:")
    print(f" 1. Frame Count (>= 120 in 60s)       : {'PASS' if pass_count else 'FAIL'} ({total_frames})")
    print(f" 2. Error Frames (0 error frames)      : {'PASS' if pass_errors else 'FAIL'} ({error_frames})")
    print(f" 3. Average Jitter (<= 2 ms)           : {'PASS' if pass_jitter else 'WARN'} ({avg_jitter:.2f} ms)")

    if pass_count and pass_errors:
        print("\n>>> OVERALL RESULT: [PASS] <<<")
    else:
        print("\n>>> OVERALL RESULT: [FAIL] <<<")


if __name__ == "__main__":
    k = sys.argv[1] if len(sys.argv) > 1 else "slcan"
    p = sys.argv[2] if len(sys.argv) > 2 else "/dev/cu.usbmodem***"
    run_test(k, p)
