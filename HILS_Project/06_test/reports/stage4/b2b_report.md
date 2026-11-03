# Stage 4 — Back-to-Back (MIL vs SIL) Equivalence Report

- **Model**: `03_simulink/controller/ctrl_codegen.slx`
- **Generated C Source**: `04_firmware/app_generated/ctrl_codegen.c`
- **Tolerance**: exact (`0`) on `fault_bitmap` / `mode`; `<= 1e-4` on `single` float signals
- **Overall Status**: **ALL PASS**

## Summary by Scenario

| Scenario | Ticks | Max Float Diff | Max Int/Enum Diff | Result |
| :--- | :---: | :---: | :---: | :---: |
| `S1_Normal_Diag_Ramp` | 401 | `2.29e-05` | `0` | **PASS** |
| `S2_XOnly_Fault_Recovery` | 451 | `0.00e+00` | `0` | **PASS** |
| `S3_Raw_Range_Fault` | 201 | `0.00e+00` | `0` | **PASS** |
| `S4_AntiWindup_Saturation` | 501 | `3.05e-05` | `0` | **PASS** |
| `S5_Random_60s_Stress` | 6001 | `2.48e-05` | `0` | **PASS** |
