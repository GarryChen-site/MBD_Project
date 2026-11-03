# 07 Docs — Engineering Report, Reproduction Guide & Interview Notes

This directory contains the final **Stage 7** release documentation for the project.

| Document | Description |
| :--- | :--- |
| [`report.md`](report.md) | **Master Engineering Report:** Complete quantitative summary across Stages 1–7, HARA-lite safety time chains, empirical system ID equations, IMC $\lambda$-tuning derivation, SIL/HIL back-to-back results, and 5 real engineering defects (`DEF-01`..`DEF-05`) |
| [`reproduce.md`](reproduce.md) | **Clean-Checkout Reproduction Runbook:** Step-by-step commands to reproduce all MATLAB/Simulink models, C code, SIL/HIL reports, CAN DBC self-tests, and traceability gates in under 5 minutes (offline) or 30 minutes (with physical STM32 hardware) |
| [`summary_ja.md`](summary_ja.md) | **日本語プロジェクト概要書 (Japanese Executive Summary):** Concise portfolio summary for Japanese OEM/Tier-1 resumes (職務経歴書) and technical interviews |
| [`interview_notes.md`](interview_notes.md) | **Technical Q&A Defense Notes:** Detailed answers to 8 core MBD/control/embedded interview questions grounded in the exact numbers and defects of this project |
| [`lessons_learned.md`](lessons_learned.md) | **Engineering Retrospective:** Key takeaways from the two-pass walking-skeleton strategy, programmatic Simulink modeling, and hardware bring-up |

> [!TIP]
> For stage-by-stage mathematical derivations and textbook-style guides bridging classical mechanics, differential equations, linear algebra, MATLAB/Simulink, and STM32 firmware, see [`stages/`](../stages/):
> - [`stages/BRIDGE_GUIDE_physics_to_code_to_hardware.md`](../stages/BRIDGE_GUIDE_physics_to_code_to_hardware.md)
> - [`stages/stage_2_guide/README.md`](../stages/stage_2_guide/README.md) (System Identification & Sensor Modeling)
> - [`stages/stage_3_guide/README.md`](../stages/stage_3_guide/README.md) (Feedback Control Design & MIL Verification)
> - [`stages/INTERVIEW_GUIDE_mbd_transition.md`](../stages/INTERVIEW_GUIDE_mbd_transition.md)
