# 06 Test — Automated Verification Reports & Traceability Gate

This directory contains the automated requirement traceability verifier ([`hil/trace_check.py`](hil/trace_check.py)) and the generated verification reports and plots for **Stages 3, 4, 6, and 7**.

---

## 1. Directory Layout

```text
06_test/
├── README.md                                  # Verification overview & test inventory
├── hil/
│   └── trace_check.py                         # Stage 7 automated traceability matrix generator & file-existence gate
└── reports/
    ├── stage4/
    │   └── b2b_report.md                      # Stage 4 MIL-vs-SIL C MEX equivalence report (5 scenarios, 7,555 ticks)
    └── stage6/
        ├── hw_vs_mil_hil_step_test.png        # Stage 6 closed-loop Hardware vs MIL overlay (autonomous 60 % step, 96.1 % fit)
        ├── hw_vs_mil_hil_closed_loop.png      # Stage 6 closed-loop Hardware vs MIL overlay (live joystick + fault injection, 96.6 % fit)
        └── hw_vs_mil_overlay.png              # Latest generated overlay plot
```

---

## 2. Verification Summary Across V-Model Levels

| Test Level | Runner Script | Scope | Result |
| :--- | :--- | :--- | :---: |
| **Stage 2: Plant & Sensor ID** | [`03_simulink/stage2/run_stage2.m`](../03_simulink/stage2/run_stage2.m) | `A2-1`..`A2-8`: Static map ($R^2=0.9999$), FOPDT step fit (**95.2 %**), Simulink plant fit (**89.7 %**), joystick `sep = 5.82` (0 FA) | **ALL PASS** |
| **Stage 3: MIL Test Suite** | [`03_simulink/tests/run_stage3_tests.m`](../03_simulink/tests/run_stage3_tests.m) | **19 automated test cases** (`TC-THR-001`..`008`, `A3-9`, `TC-SPD-001`..`004`, `TC-DIA-001`, `TC-SAF-001`, `TC-AEB-001`..`003`) | **19 / 19 PASS** |
| **Stage 4: SIL Back-to-Back** | [`03_simulink/stage4/s4_03_run_sil_b2b.m`](../03_simulink/stage4/s4_03_run_sil_b2b.m) | **5 composite scenarios** (`7,555` ticks total): `ctrl_codegen.slx` vs compiled `ctrl_sil_mex` ([`b2b_report.md`](reports/stage4/b2b_report.md)) | **5 / 5 PASS** (max diff $3.05\times 10^{-5}$) |
| **Stage 5: CAN & E2E Self-Test** | [`05_can/scripts/dbc_codegen.py`](../05_can/scripts/dbc_codegen.py) | SAE J1850 CRC-8 (`0x4B`) + **1,000-frame** C/Python fuzz test + CRC/counter fault injection | **ALL PASS** |
| **Stage 6: Real-Hardware HIL** | [`03_simulink/stage4/s6_compare_hw_vs_mil.m`](../03_simulink/stage4/s6_compare_hw_vs_mil.m) | `A6-1`..`A6-5` on `hil_step_test.csv` (**96.1 %** fit, **75 µs** WCET) & `hil_closed_loop.csv` (**96.6 %** fit, **77 µs** WCET) | **ALL PASS** |
| **Stage 7: Traceability Gate** | [`06_test/hil/trace_check.py`](hil/trace_check.py) | Verifies 100 % requirement-to-design-to-test mapping & on-disk file existence (`01_requirements/traceability.md`) | **47 / 47 PASS** (`0` violations) |
