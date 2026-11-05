# 05 CAN — DBC Single Source of Truth, Code Generator & Bus Monitor

This directory implements **Stage 5** of the V-Model: the single-source-of-truth CAN database ([`vehicle.dbc`](vehicle.dbc)), the automated Python-to-C99 BSW code generator and E2E verification harness ([`scripts/dbc_codegen.py`](scripts/dbc_codegen.py)), and the host CAN bus monitor/stimulus tool ([`can_monitor.py`](can_monitor.py)).

---

## 1. Directory Layout

```text
05_can/
├── README.md                  # CAN bus specification, message table & usage commands
├── vehicle.dbc                # Single-source-of-truth CAN database (CAN 2.0A, 500 kbit/s, Intel byte order)
├── can_monitor.py             # Host CAN bus monitor, decoder, and fault-injection tool
└── scripts/
    └── dbc_codegen.py         # Parses `vehicle.dbc`, generates `04_firmware/bsw/com_can_signals.{c,h}`,
                               # and runs the 1,000-frame C-vs-Python E2E CRC-8 self-test (`--selftest`)
```

---

## 2. Production Message Matrix (`vehicle.dbc`)

All frames use standard 11-bit identifiers (`CAN 2.0A`) at **500 kbit/s** (`BRP = 2, TS1 = 13, TS2 = 2` → 16 tq/bit, 87.5 % sample point at 16 MHz `PCLK1`) and little-endian (`@1+`) signal layout:

| CAN ID | Frame Name | Sender → Receiver | Period | DLC | Signals (`vehicle.dbc`) | E2E Protection |
| :---: | :--- | :---: | :---: | :-: | :--- | :--- |
| `0x100` | `Pedal_Cmd` | `PC` → `ECU` | `20 ms` | 4 | `pedal_pct` (`0..100 %`, 0.5 %/LSB), `ctrl_source` (`0=LOCAL, 1=BENCH`), `msg_counter` (4b), `msg_crc` (8b) | CRC-8 SAE J1850 + 4-bit counter |
| `0x123` | `ECU_Heartbeat` | `ECU` → `PC` | `500 ms` | 5 | `hb_counter` (32b), `loopback_flag` (8b) | Rolling 32-bit counter |
| `0x200` | `Drive_Status` | `ECU` → `PC` | `20 ms` | 7 | `speed_rpm` (`0..200 rpm`, 0.01/LSB), `duty_pct` (`0..100 %`, 0.5/LSB), `mode` (4b), `speed_req_rpm` (`0..200 rpm`, 0.1/LSB), `msg_counter` (4b), `msg_crc` (8b) | CRC-8 SAE J1850 + 4-bit counter |
| `0x201` | `Diag_Status` | `ECU` → `PC` | `100 ms` | 5 | `fault_bitmap` (16b), `rx_err_count` (8b), `msg_counter` (4b), `msg_crc` (8b) | CRC-8 SAE J1850 + 4-bit counter |
| `0x210` | `AEB_Status` | `ECU` → `PC` | `50 ms` | 5 | `distance_mm` (`0..2000 mm`), `aeb_state` (4b), `tof_valid` (1b), `msg_counter` (4b), `msg_crc` (8b) | CRC-8 SAE J1850 + 4-bit counter |
| `0x7E0 / 0x7E8` | `UDS_Diag` | `PC` ↔ `ECU` | Event | 8 | Minimal ISO 14229 UDS Service `0x22 ReadDataByIdentifier` (`0xF100`..`0xF103` in [`can_telemetry.c`](../04_firmware/bsw/can_telemetry.c)) | ISO-TP single-frame (`PCI = 0x03 / 0x05`) |

---

## 3. Code Generation & Verification Commands

Run from the repository root:

```bash
# Regenerate 04_firmware/bsw/com_can_signals.{c,h} from vehicle.dbc and run the C/Python E2E self-test:
python3 05_can/scripts/dbc_codegen.py --selftest
```

**Verified Self-Test Checks (`ALL PASS`):**
- `A5-1`: `E2E_Crc8("123456789") == 0x4B` (standard SAE J1850 check value).
- `A5-2`: **1,000 / 1,000** random frames across `Pedal_Cmd`, `Drive_Status`, `Diag_Status`, `AEB_Status`, and `ECU_Heartbeat` match bit-for-bit between generated C and Python.
- `A5-5` & `A5-6`: Fault-injection checks verify that corrupted CRC (`E2E_ERR_CRC`), repeated alive counter (`E2E_ERR_REPEAT`), and skipped alive counter (`E2E_ERR_SEQUENCE`) are deterministically rejected and increment `err_total`.
