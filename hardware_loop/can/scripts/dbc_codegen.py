#!/usr/bin/env python3
"""
Stage 5 — DBC-to-C Code Generator & E2E Consistency Self-Test
==============================================================
Parses `05_can/vehicle.dbc` (single source of truth, CAN-008) and generates:
  - `04_firmware/bsw/com_can_signals.h`
  - `04_firmware/bsw/com_can_signals.c`

Includes:
  - Intel (`@1+`) arbitrary bit-field pack/unpack (`Com_PackBits`, `Com_UnpackBits`)
  - SAE J1850 CRC-8 (`0x1D`, init `0xFF`, xorout `0xFF`) + 4-bit alive counter E2E protection
  - Typed C pack/unpack functions for every message in `vehicle.dbc`
  - Host C vs Python self-test (`--selftest`) verifying Criteria A5-1, A5-2, A5-5, A5-6

Usage:
    python3 05_can/scripts/dbc_codegen.py --selftest
"""

import argparse
import ctypes
import math
import random
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional


@dataclass
class SignalDef:
    name: str
    start_bit: int
    length: int
    is_little_endian: bool
    is_signed: bool
    factor: float
    offset: float
    minimum: float
    maximum: float
    unit: str
    receiver: str


@dataclass
class MessageDef:
    can_id: int
    name: str
    dlc: int
    transmitter: str
    cycle_ms: int = 0
    signals: List[SignalDef] = field(default_factory=list)

    @property
    def e2e_data_id(self) -> int:
        return self.can_id & 0xFF

    @property
    def crc_sig(self) -> Optional[SignalDef]:
        for s in self.signals:
            if s.name == "msg_crc":
                return s
        return None

    @property
    def cnt_sig(self) -> Optional[SignalDef]:
        for s in self.signals:
            if s.name == "msg_counter":
                return s
        return None


def parse_dbc(dbc_path: Path) -> List[MessageDef]:
    """Parse standard Vector DBC messages, signals, and cycle times."""
    text = dbc_path.read_text(encoding="utf-8")
    bo_re = re.compile(r"^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\w+)")
    sg_re = re.compile(
        r"^\s+SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*"
        r"\(([^,]+),([^)]+)\)\s*\[([^|]+)\|([^\]]+)\]\s*\"([^\"]*)\"\s*(\S+)"
    )
    ba_cycle_re = re.compile(r'^BA_\s+"GenMsgCycleTime"\s+BO_\s+(\d+)\s+(\d+)\s*;')

    messages: List[MessageDef] = []
    by_id: Dict[int, MessageDef] = {}
    current_msg: Optional[MessageDef] = None

    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        m_bo = bo_re.match(line)
        if m_bo:
            can_id = int(m_bo.group(1))
            name = m_bo.group(2)
            dlc = int(m_bo.group(3))
            tx = m_bo.group(4)
            current_msg = MessageDef(can_id=can_id, name=name, dlc=dlc, transmitter=tx)
            messages.append(current_msg)
            by_id[can_id] = current_msg
            continue

        m_sg = sg_re.match(line)
        if m_sg and current_msg is not None:
            sig = SignalDef(
                name=m_sg.group(1),
                start_bit=int(m_sg.group(2)),
                length=int(m_sg.group(3)),
                is_little_endian=(m_sg.group(4) == "1"),
                is_signed=(m_sg.group(5) == "-"),
                factor=float(m_sg.group(6)),
                offset=float(m_sg.group(7)),
                minimum=float(m_sg.group(8)),
                maximum=float(m_sg.group(9)),
                unit=m_sg.group(10),
                receiver=m_sg.group(11),
            )
            current_msg.signals.append(sig)
            continue

        m_ba = ba_cycle_re.match(line)
        if m_ba:
            cid = int(m_ba.group(1))
            cyc = int(m_ba.group(2))
            if cid in by_id:
                by_id[cid].cycle_ms = cyc

    return messages


# ---------------------------------------------------------------------------
# Reference Python E2E + Intel Bit Packing (for --selftest verification)
# ---------------------------------------------------------------------------
def py_crc8_raw(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc ^ 0xFF


def py_e2e_crc8(data: bytes, data_id: int) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1D) & 0xFF) if (crc & 0x80) else ((crc << 1) & 0xFF)
    crc ^= data_id & 0xFF
    for _ in range(8):
        crc = (((crc << 1) ^ 0x1D) & 0xFF) if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc ^ 0xFF


def py_pack_bits(buf: bytearray, start_bit: int, length: int, raw: int) -> None:
    mask = (1 << length) - 1
    val = raw & mask
    for i in range(length):
        bit_pos = start_bit + i
        byte_idx = bit_pos // 8
        bit_idx = bit_pos % 8
        if (val >> i) & 1:
            buf[byte_idx] |= 1 << bit_idx
        else:
            buf[byte_idx] &= ~(1 << bit_idx)


def py_encode_message(msg: MessageDef, sig_values: Dict[str, float], counter: int) -> bytes:
    buf = bytearray(msg.dlc)
    for s in msg.signals:
        if s.name in ("msg_counter", "msg_crc"):
            continue
        phys = sig_values[s.name]
        phys_clamped = max(s.minimum, min(s.maximum, phys))
        raw = int(math.floor((phys_clamped - s.offset) / s.factor + 0.5))
        max_raw = (1 << s.length) - 1
        raw = max(0, min(max_raw, raw))
        py_pack_bits(buf, s.start_bit, s.length, raw)

    cnt_sig = msg.cnt_sig
    crc_sig = msg.crc_sig
    if cnt_sig is not None:
        py_pack_bits(buf, cnt_sig.start_bit, cnt_sig.length, counter & 0x0F)
    if crc_sig is not None:
        crc_byte = crc_sig.start_bit // 8
        crc = py_e2e_crc8(bytes(buf[:crc_byte]), msg.e2e_data_id)
        buf[crc_byte] = crc
    return bytes(buf)


# ---------------------------------------------------------------------------
# C Code Generation
# ---------------------------------------------------------------------------
def c_type_for_signal(s: SignalDef) -> str:
    if s.factor != 1.0 or s.offset != 0.0:
        return "float"
    if s.length <= 8:
        return "int8_t" if s.is_signed else "uint8_t"
    if s.length <= 16:
        return "int16_t" if s.is_signed else "uint16_t"
    return "int32_t" if s.is_signed else "uint32_t"


def generate_c_files(messages: List[MessageDef], out_dir: Path, node: str = "ECU") -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    h_path = out_dir / "com_can_signals.h"
    c_path = out_dir / "com_can_signals.c"

    # ---- Build Header (.h) ----
    h_lines = [
        "/**",
        " * @file com_can_signals.h",
        " * @brief Auto-generated CAN signal pack/unpack & E2E protection from `05_can/vehicle.dbc`.",
        " * @note  DO NOT EDIT MANUALLY — regenerate via `python3 05_can/scripts/dbc_codegen.py`.",
        " */",
        "#ifndef COM_CAN_SIGNALS_H",
        "#define COM_CAN_SIGNALS_H",
        "",
        "#include <stdint.h>",
        "#include <stddef.h>",
        "",
        "#ifdef __cplusplus",
        'extern "C" {',
        "#endif",
        "",
        "/* ========================================================================= */",
        "/* 1. CAN Message IDs & Metadata                                             */",
        "/* ========================================================================= */",
    ]

    for m in messages:
        macro_upper = m.name.upper()
        h_lines.append(f"#define CAN_ID_{macro_upper:<18} (0x{m.can_id:03X}U) /* {m.can_id} */")
        h_lines.append(f"#define CAN_DLC_{macro_upper:<17} ({m.dlc}U)")
        h_lines.append(f"#define CAN_CYCLE_{macro_upper:<15} ({m.cycle_ms}U)")

    h_lines.extend(
        [
            "",
            "typedef enum {",
        ]
    )
    for m in messages:
        h_lines.append(f"    COM_MSG_{m.name.upper()},")
    h_lines.extend(
        [
            "    COM_MSG_COUNT",
            "} Com_MsgId_t;",
            "",
            "typedef struct {",
            "    uint16_t can_id;",
            "    uint8_t  dlc;",
            "    uint16_t cycle_ms;",
            "    uint8_t  is_tx;",
            "    uint8_t  e2e_data_id;",
            "    int8_t   crc_byte;",
            "    int8_t   cnt_start_bit;",
            "} Com_MsgCfg_t;",
            "",
            "extern const Com_MsgCfg_t Com_MsgCfg[COM_MSG_COUNT];",
            "",
            "/* ========================================================================= */",
            "/* 2. E2E Protection (CRC-8 SAE J1850 + 4-bit Alive Counter)                 */",
            "/* ========================================================================= */",
            "typedef enum {",
            "    E2E_OK = 0,",
            "    E2E_ERR_CRC,",
            "    E2E_ERR_REPEAT,",
            "    E2E_ERR_SEQUENCE,",
            "    E2E_ERR_NO_NEW",
            "} E2E_Status_t;",
            "",
            "typedef struct {",
            "    uint8_t  last_cnt;",
            "    uint8_t  initialised;",
            "    uint16_t lost;",
            "    uint16_t err_total;",
            "} E2E_State_t;",
            "",
            "uint8_t      E2E_Crc8_Raw(const uint8_t *data, uint8_t len);",
            "uint8_t      E2E_Crc8(const uint8_t *data, uint8_t len, uint8_t data_id);",
            "void         E2E_Protect(uint8_t *msg, uint8_t dlc, uint8_t crc_byte_idx,",
            "                         uint8_t cnt_start_bit, uint8_t data_id, uint8_t *counter);",
            "E2E_Status_t E2E_Check(const uint8_t *msg, uint8_t dlc, uint8_t crc_byte_idx,",
            "                       uint8_t cnt_start_bit, uint8_t data_id, E2E_State_t *st);",
            "",
            "/* ========================================================================= */",
            "/* 3. Low-Level Intel (@1+) Bit Field Pack / Unpack                          */",
            "/* ========================================================================= */",
            "void     Com_PackBits(uint8_t *buf, uint8_t start_bit, uint8_t len, uint32_t raw);",
            "uint32_t Com_UnpackBits(const uint8_t *buf, uint8_t start_bit, uint8_t len);",
            "int32_t  Com_SignExtend(uint32_t raw, uint8_t len);",
            "",
            "/* ========================================================================= */",
            "/* 4. Typed Message Payloads & Pack/Unpack API                               */",
            "/* ========================================================================= */",
        ]
    )

    for m in messages:
        h_lines.append(f"typedef struct {{")
        for s in m.signals:
            if s.name in ("msg_counter", "msg_crc"):
                continue
            ctype = c_type_for_signal(s)
            unit_comment = f" /* [{s.minimum}..{s.maximum}] {s.unit} */" if s.unit or s.maximum else ""
            h_lines.append(f"    {ctype:<10} {s.name};{unit_comment}")
        h_lines.append(f"}} Com_{m.name}_t;")
        h_lines.append("")
        if m.crc_sig is not None:
            h_lines.append(
                f"void         Com_Pack_{m.name}(const Com_{m.name}_t *in, uint8_t *out_buf, uint8_t *alive_cnt);"
            )
            h_lines.append(
                f"E2E_Status_t Com_Unpack_{m.name}(const uint8_t *in_buf, Com_{m.name}_t *out, E2E_State_t *e2e_state);"
            )
        else:
            h_lines.append(f"void         Com_Pack_{m.name}(const Com_{m.name}_t *in, uint8_t *out_buf);")
            h_lines.append(f"void         Com_Unpack_{m.name}(const uint8_t *in_buf, Com_{m.name}_t *out);")
        h_lines.append("")

    h_lines.extend(
        [
            "#ifdef __cplusplus",
            "}",
            "#endif",
            "",
            "#endif /* COM_CAN_SIGNALS_H */",
            "",
        ]
    )
    h_path.write_text("\n".join(h_lines), encoding="utf-8")

    # ---- Build Implementation (.c) ----
    c_lines = [
        "/**",
        " * @file com_can_signals.c",
        " * @brief Auto-generated CAN signal pack/unpack & E2E protection from `05_can/vehicle.dbc`.",
        " */",
        '#include "com_can_signals.h"',
        "#include <string.h>",
        "",
        "const Com_MsgCfg_t Com_MsgCfg[COM_MSG_COUNT] = {",
    ]
    for m in messages:
        is_tx = 1 if m.transmitter == node else 0
        crc_byte = (m.crc_sig.start_bit // 8) if m.crc_sig else -1
        cnt_bit = m.cnt_sig.start_bit if m.cnt_sig else -1
        c_lines.append(
            f"    {{ 0x{m.can_id:03X}U, {m.dlc}U, {m.cycle_ms}U, {is_tx}U, 0x{m.e2e_data_id:02X}U, {crc_byte}, {cnt_bit} }}, /* {m.name} */"
        )
    c_lines.extend(
        [
            "};",
            "",
            "static uint32_t clamp_and_quantize_u32(float phys, float factor, float offset, float min_v, float max_v, uint8_t len)",
            "{",
            "    float clamped = phys;",
            "    if (clamped < min_v) { clamped = min_v; }",
            "    if (clamped > max_v) { clamped = max_v; }",
            "    float scaled = (clamped - offset) / factor;",
            "    uint32_t raw = (scaled >= 0.0F) ? (uint32_t)(scaled + 0.5F) : 0U;",
            "    uint32_t max_raw = (len >= 32U) ? 0xFFFFFFFFUL : ((1UL << len) - 1UL);",
            "    if (raw > max_raw) { raw = max_raw; }",
            "    return raw;",
            "}",
            "",
            "void Com_PackBits(uint8_t *buf, uint8_t start_bit, uint8_t len, uint32_t raw)",
            "{",
            "    for (uint8_t i = 0U; i < len; i++) {",
            "        uint8_t bit_pos = (uint8_t)(start_bit + i);",
            "        uint8_t byte_idx = (uint8_t)(bit_pos >> 3U);",
            "        uint8_t bit_idx  = (uint8_t)(bit_pos & 0x07U);",
            "        if (((raw >> i) & 1UL) != 0UL) {",
            "            buf[byte_idx] |= (uint8_t)(1U << bit_idx);",
            "        } else {",
            "            buf[byte_idx] &= (uint8_t)~(1U << bit_idx);",
            "        }",
            "    }",
            "}",
            "",
            "uint32_t Com_UnpackBits(const uint8_t *buf, uint8_t start_bit, uint8_t len)",
            "{",
            "    uint32_t raw = 0UL;",
            "    for (uint8_t i = 0U; i < len; i++) {",
            "        uint8_t bit_pos = (uint8_t)(start_bit + i);",
            "        uint8_t byte_idx = (uint8_t)(bit_pos >> 3U);",
            "        uint8_t bit_idx  = (uint8_t)(bit_pos & 0x07U);",
            "        if (((buf[byte_idx] >> bit_idx) & 1U) != 0U) {",
            "            raw |= (1UL << i);",
            "        }",
            "    }",
            "    return raw;",
            "}",
            "",
            "int32_t Com_SignExtend(uint32_t raw, uint8_t len)",
            "{",
            "    if ((len == 0U) || (len >= 32U)) {",
            "        return (int32_t)raw;",
            "    }",
            "    uint32_t sign_bit = 1UL << (len - 1U);",
            "    if ((raw & sign_bit) != 0UL) {",
            "        uint32_t mask = ~((1UL << len) - 1UL);",
            "        raw |= mask;",
            "    }",
            "    return (int32_t)raw;",
            "}",
            "",
            "uint8_t E2E_Crc8_Raw(const uint8_t *data, uint8_t len)",
            "{",
            "    uint8_t crc = 0xFFU;",
            "    for (uint8_t i = 0U; i < len; i++) {",
            "        crc ^= data[i];",
            "        for (uint8_t b = 0U; b < 8U; b++) {",
            "            crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x1DU) : (uint8_t)(crc << 1U);",
            "        }",
            "    }",
            "    return (uint8_t)(crc ^ 0xFFU);",
            "}",
            "",
            "uint8_t E2E_Crc8(const uint8_t *data, uint8_t len, uint8_t data_id)",
            "{",
            "    uint8_t crc = 0xFFU;",
            "    for (uint8_t i = 0U; i < len; i++) {",
            "        crc ^= data[i];",
            "        for (uint8_t b = 0U; b < 8U; b++) {",
            "            crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x1DU) : (uint8_t)(crc << 1U);",
            "        }",
            "    }",
            "    crc ^= data_id;",
            "    for (uint8_t b = 0U; b < 8U; b++) {",
            "        crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x1DU) : (uint8_t)(crc << 1U);",
            "    }",
            "    return (uint8_t)(crc ^ 0xFFU);",
            "}",
            "",
            "void E2E_Protect(uint8_t *msg, uint8_t dlc, uint8_t crc_byte_idx,",
            "                 uint8_t cnt_start_bit, uint8_t data_id, uint8_t *counter)",
            "{",
            "    (void)dlc;",
            "    uint8_t cnt = (uint8_t)((*counter) & 0x0FU);",
            "    Com_PackBits(msg, cnt_start_bit, 4U, (uint32_t)cnt);",
            "    msg[crc_byte_idx] = E2E_Crc8(msg, crc_byte_idx, data_id);",
            "    *counter = (uint8_t)((cnt + 1U) & 0x0FU);",
            "}",
            "",
            "E2E_Status_t E2E_Check(const uint8_t *msg, uint8_t dlc, uint8_t crc_byte_idx,",
            "                       uint8_t cnt_start_bit, uint8_t data_id, E2E_State_t *st)",
            "{",
            "    (void)dlc;",
            "    uint8_t expected_crc = E2E_Crc8(msg, crc_byte_idx, data_id);",
            "    if (msg[crc_byte_idx] != expected_crc) {",
            "        st->err_total++;",
            "        return E2E_ERR_CRC;",
            "    }",
            "    uint8_t cnt = (uint8_t)(Com_UnpackBits(msg, cnt_start_bit, 4U) & 0x0FU);",
            "    if (st->initialised == 0U) {",
            "        st->initialised = 1U;",
            "        st->last_cnt = cnt;",
            "        return E2E_OK;",
            "    }",
            "    uint8_t delta = (uint8_t)((cnt - st->last_cnt) & 0x0FU);",
            "    if (delta == 0U) {",
            "        st->err_total++;",
            "        return E2E_ERR_REPEAT;",
            "    }",
            "    st->last_cnt = cnt;",
            "    if (delta == 1U) {",
            "        return E2E_OK;",
            "    }",
            "    if (delta == 2U) {",
            "        st->lost++;",
            "        return E2E_OK;",
            "    }",
            "    st->err_total++;",
            "    return E2E_ERR_SEQUENCE;",
            "}",
            "",
        ]
    )

    for m in messages:
        crc_sig = m.crc_sig
        cnt_sig = m.cnt_sig
        if crc_sig is not None and cnt_sig is not None:
            crc_byte = crc_sig.start_bit // 8
            c_lines.append(
                f"void Com_Pack_{m.name}(const Com_{m.name}_t *in, uint8_t *out_buf, uint8_t *alive_cnt)"
            )
            c_lines.append("{")
            c_lines.append(f"    memset(out_buf, 0, {m.dlc}U);")
            for s in m.signals:
                if s.name in ("msg_counter", "msg_crc"):
                    continue
                ctype = c_type_for_signal(s)
                if ctype == "float":
                    c_lines.append(
                        f"    Com_PackBits(out_buf, {s.start_bit}U, {s.length}U, "
                        f"clamp_and_quantize_u32(in->{s.name}, {s.factor}F, {s.offset}F, {s.minimum}F, {s.maximum}F, {s.length}U));"
                    )
                else:
                    c_lines.append(
                        f"    Com_PackBits(out_buf, {s.start_bit}U, {s.length}U, (uint32_t)in->{s.name});"
                    )
            c_lines.append(
                f"    E2E_Protect(out_buf, {m.dlc}U, {crc_byte}U, {cnt_sig.start_bit}U, 0x{m.e2e_data_id:02X}U, alive_cnt);"
            )
            c_lines.append("}")
            c_lines.append("")

            c_lines.append(
                f"E2E_Status_t Com_Unpack_{m.name}(const uint8_t *in_buf, Com_{m.name}_t *out, E2E_State_t *e2e_state)"
            )
            c_lines.append("{")
            c_lines.append(
                f"    E2E_Status_t st = E2E_Check(in_buf, {m.dlc}U, {crc_byte}U, {cnt_sig.start_bit}U, 0x{m.e2e_data_id:02X}U, e2e_state);"
            )
            c_lines.append("    if (st != E2E_OK) {")
            c_lines.append("        return st;")
            c_lines.append("    }")
            for s in m.signals:
                if s.name in ("msg_counter", "msg_crc"):
                    continue
                ctype = c_type_for_signal(s)
                if ctype == "float":
                    c_lines.append(
                        f"    out->{s.name} = (float)Com_UnpackBits(in_buf, {s.start_bit}U, {s.length}U) * {s.factor}F + {s.offset}F;"
                    )
                else:
                    c_lines.append(
                        f"    out->{s.name} = ({ctype})Com_UnpackBits(in_buf, {s.start_bit}U, {s.length}U);"
                    )
            c_lines.append("    return E2E_OK;")
            c_lines.append("}")
            c_lines.append("")
        else:
            c_lines.append(f"void Com_Pack_{m.name}(const Com_{m.name}_t *in, uint8_t *out_buf)")
            c_lines.append("{")
            c_lines.append(f"    memset(out_buf, 0, {m.dlc}U);")
            for s in m.signals:
                ctype = c_type_for_signal(s)
                c_lines.append(
                    f"    Com_PackBits(out_buf, {s.start_bit}U, {s.length}U, (uint32_t)in->{s.name});"
                )
            c_lines.append("}")
            c_lines.append("")
            c_lines.append(f"void Com_Unpack_{m.name}(const uint8_t *in_buf, Com_{m.name}_t *out)")
            c_lines.append("{")
            for s in m.signals:
                ctype = c_type_for_signal(s)
                c_lines.append(
                    f"    out->{s.name} = ({ctype})Com_UnpackBits(in_buf, {s.start_bit}U, {s.length}U);"
                )
            c_lines.append("}")
            c_lines.append("")

    c_path.write_text("\n".join(c_lines), encoding="utf-8")


# ---------------------------------------------------------------------------
# Host C vs Python Self-Test (Criteria A5-1, A5-2, A5-5, A5-6)
# ---------------------------------------------------------------------------
def run_selftest(messages: List[MessageDef], out_dir: Path) -> int:
    print("=== Stage 5 DBC Codegen & E2E Consistency Self-Test ===")
    c_path = out_dir / "com_can_signals.c"

    with tempfile.TemporaryDirectory() as tmpdir:
        lib_path = Path(tmpdir) / "libcom_can.dylib"
        cmd = [
            "/usr/bin/clang",
            "-dynamiclib",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(out_dir),
            str(c_path),
            "-o",
            str(lib_path),
        ]
        subprocess.run(cmd, check=True)
        lib = ctypes.CDLL(str(lib_path))

        # 1. Criterion A5-1a: Standard SAE J1850 check value "123456789" -> 0x4B
        lib.E2E_Crc8_Raw.argtypes = [ctypes.c_char_p, ctypes.c_uint8]
        lib.E2E_Crc8_Raw.restype = ctypes.c_uint8
        c_check = lib.E2E_Crc8_Raw(b"123456789", 9)
        py_check = py_crc8_raw(b"123456789")
        assert c_check == 0x4B and py_check == 0x4B, f"CRC check failed: C=0x{c_check:02X}, Py=0x{py_check:02X}"
        print(f"[A5-1a] SAE J1850 CRC-8 check(\"123456789\") = 0x{c_check:02X} (expected 0x4B) -> PASS")

        # 2. Criterion A5-1b & A5-2: 1,000 random frames across all messages (C vs Python byte-identical + unpack round-trip)
        class Com_Pedal_Cmd_t(ctypes.Structure):
            _fields_ = [("pedal_pct", ctypes.c_float)]

        class Com_Drive_Status_t(ctypes.Structure):
            _fields_ = [
                ("speed_rpm", ctypes.c_float),
                ("duty_pct", ctypes.c_float),
                ("mode", ctypes.c_uint8),
                ("speed_req_rpm", ctypes.c_float),
            ]

        class Com_Diag_Status_t(ctypes.Structure):
            _fields_ = [
                ("fault_bitmap", ctypes.c_uint16),
                ("rx_err_count", ctypes.c_uint8),
            ]

        class Com_AEB_Status_t(ctypes.Structure):
            _fields_ = [
                ("distance_mm", ctypes.c_uint16),
                ("aeb_state", ctypes.c_uint8),
                ("tof_valid", ctypes.c_uint8),
            ]

        class E2E_State_t(ctypes.Structure):
            _fields_ = [
                ("last_cnt", ctypes.c_uint8),
                ("initialised", ctypes.c_uint8),
                ("lost", ctypes.c_uint16),
                ("err_total", ctypes.c_uint16),
            ]

        msg_by_name = {m.name: m for m in messages}
        rng = random.Random(42)

        cnt_pedal = ctypes.c_uint8(0)
        cnt_drive = ctypes.c_uint8(0)
        cnt_diag = ctypes.c_uint8(0)
        cnt_aeb = ctypes.c_uint8(0)

        st_pedal = E2E_State_t(0, 0, 0, 0)
        st_drive = E2E_State_t(0, 0, 0, 0)
        st_diag = E2E_State_t(0, 0, 0, 0)
        st_aeb = E2E_State_t(0, 0, 0, 0)

        for i in range(250):
            # 0x100 Pedal_Cmd
            pct = round(rng.uniform(0.0, 100.0), 2)
            py_bytes = py_encode_message(msg_by_name["Pedal_Cmd"], {"pedal_pct": pct}, cnt_pedal.value)
            c_in = Com_Pedal_Cmd_t(pct)
            c_buf = (ctypes.c_uint8 * 4)()
            lib.Com_Pack_Pedal_Cmd(ctypes.byref(c_in), c_buf, ctypes.byref(cnt_pedal))
            assert bytes(c_buf) == py_bytes, f"Pedal_Cmd mismatch at {i}: C={bytes(c_buf).hex()} Py={py_bytes.hex()}"
            c_out = Com_Pedal_Cmd_t()
            rc = lib.Com_Unpack_Pedal_Cmd(c_buf, ctypes.byref(c_out), ctypes.byref(st_pedal))
            assert rc == 0 and abs(c_out.pedal_pct - pct) <= 0.011

            # 0x200 Drive_Status
            spd = round(rng.uniform(0.0, 160.0), 1)
            duty = round(rng.randint(0, 120) * 0.5, 1)
            mode = rng.randint(0, 3)
            sreq = round(rng.uniform(0.0, 150.0), 1)
            py_bytes = py_encode_message(
                msg_by_name["Drive_Status"],
                {"speed_rpm": spd, "duty_pct": duty, "mode": mode, "speed_req_rpm": sreq},
                cnt_drive.value,
            )
            c_in_d = Com_Drive_Status_t(spd, duty, mode, sreq)
            c_buf_d = (ctypes.c_uint8 * 7)()
            lib.Com_Pack_Drive_Status(ctypes.byref(c_in_d), c_buf_d, ctypes.byref(cnt_drive))
            assert bytes(c_buf_d) == py_bytes
            c_out_d = Com_Drive_Status_t()
            rc = lib.Com_Unpack_Drive_Status(c_buf_d, ctypes.byref(c_out_d), ctypes.byref(st_drive))
            assert rc == 0 and abs(c_out_d.speed_rpm - spd) <= 0.11 and c_out_d.mode == mode

            # 0x201 Diag_Status
            fb = rng.randint(0, 0xFFFF)
            rxe = rng.randint(0, 255)
            py_bytes = py_encode_message(
                msg_by_name["Diag_Status"], {"fault_bitmap": fb, "rx_err_count": rxe}, cnt_diag.value
            )
            c_in_dg = Com_Diag_Status_t(fb, rxe)
            c_buf_dg = (ctypes.c_uint8 * 5)()
            lib.Com_Pack_Diag_Status(ctypes.byref(c_in_dg), c_buf_dg, ctypes.byref(cnt_diag))
            assert bytes(c_buf_dg) == py_bytes
            c_out_dg = Com_Diag_Status_t()
            rc = lib.Com_Unpack_Diag_Status(c_buf_dg, ctypes.byref(c_out_dg), ctypes.byref(st_diag))
            assert rc == 0 and c_out_dg.fault_bitmap == fb and c_out_dg.rx_err_count == rxe

            # 0x210 AEB_Status
            dist = rng.randint(0, 2000)
            aeb_st = rng.randint(0, 2)
            tof_v = rng.randint(0, 1)
            py_bytes = py_encode_message(
                msg_by_name["AEB_Status"],
                {"distance_mm": dist, "aeb_state": aeb_st, "tof_valid": tof_v},
                cnt_aeb.value,
            )
            c_in_a = Com_AEB_Status_t(dist, aeb_st, tof_v)
            c_buf_a = (ctypes.c_uint8 * 5)()
            lib.Com_Pack_AEB_Status(ctypes.byref(c_in_a), c_buf_a, ctypes.byref(cnt_aeb))
            assert bytes(c_buf_a) == py_bytes
            c_out_a = Com_AEB_Status_t()
            rc = lib.Com_Unpack_AEB_Status(c_buf_a, ctypes.byref(c_out_a), ctypes.byref(st_aeb))
            assert rc == 0 and c_out_a.distance_mm == dist and c_out_a.aeb_state == aeb_st

        print("[A5-1b & A5-2] 1000/1000 random frames Python vs C byte-identical & round-trip unpack -> PASS")

        # 3. Criterion A5-5: 10 frames with corrupted CRC -> all 10 rejected, err_total increases by 10
        err_before = st_pedal.err_total
        last_good_pct = c_out.pedal_pct
        for _ in range(10):
            c_in = Com_Pedal_Cmd_t(99.9)
            c_buf = (ctypes.c_uint8 * 4)()
            lib.Com_Pack_Pedal_Cmd(ctypes.byref(c_in), c_buf, ctypes.byref(cnt_pedal))
            c_buf[3] ^= 0x55  # Corrupt CRC byte
            rc = lib.Com_Unpack_Pedal_Cmd(c_buf, ctypes.byref(c_out), ctypes.byref(st_pedal))
            assert rc == 1  # E2E_ERR_CRC
        assert (st_pedal.err_total - err_before) == 10
        assert abs(c_out.pedal_pct - last_good_pct) < 1e-5
        print("[A5-5] 10/10 corrupted-CRC frames discarded, err_total += 10, output unchanged -> PASS")

        # 4. Criterion A5-6: Counter repeat (delta=0) and sequence jump (delta>=3)
        st_seq = E2E_State_t(0, 0, 0, 0)
        cnt_seq = ctypes.c_uint8(0)
        c_buf = (ctypes.c_uint8 * 4)()
        lib.Com_Pack_Pedal_Cmd(ctypes.byref(c_in), c_buf, ctypes.byref(cnt_seq))
        assert lib.Com_Unpack_Pedal_Cmd(c_buf, ctypes.byref(c_out), ctypes.byref(st_seq)) == 0  # init frame OK
        # Repeat same frame (delta == 0 -> E2E_ERR_REPEAT == 2)
        assert lib.Com_Unpack_Pedal_Cmd(c_buf, ctypes.byref(c_out), ctypes.byref(st_seq)) == 2
        # Jump counter by +5 (delta == 5 -> E2E_ERR_SEQUENCE == 3)
        cnt_seq.value = 5
        lib.Com_Pack_Pedal_Cmd(ctypes.byref(c_in), c_buf, ctypes.byref(cnt_seq))
        assert lib.Com_Unpack_Pedal_Cmd(c_buf, ctypes.byref(c_out), ctypes.byref(st_seq)) == 3
        print("[A5-6] E2E alive counter repeat (E2E_ERR_REPEAT) & jump (E2E_ERR_SEQUENCE) verified -> PASS")

    print("=== Stage 5 DBC & E2E Self-Test: ALL PASS ===")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Stage 5 DBC-to-C Code Generator")
    parser.add_argument("--dbc", default="05_can/vehicle.dbc", help="Path to vehicle.dbc")
    parser.add_argument("--node", default="ECU", help="ECU node name in DBC")
    parser.add_argument("--out-dir", default="04_firmware/bsw", help="Output directory for generated C files")
    parser.add_argument("--selftest", action="store_true", help="Compile generated C code and run E2E self-tests")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    dbc_path = repo_root / args.dbc
    out_dir = repo_root / args.out_dir

    messages = parse_dbc(dbc_path)
    generate_c_files(messages, out_dir, node=args.node)
    print(
        f"[dbc_codegen] Parsed {len(messages)} messages from {dbc_path.relative_to(repo_root)} "
        f"-> generated {out_dir.relative_to(repo_root)}/com_can_signals.{{h,c}}"
    )

    if args.selftest:
        return run_selftest(messages, out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
