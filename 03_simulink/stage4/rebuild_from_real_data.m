function out = rebuild_from_real_data()
%REBUILD_FROM_REAL_DATA  Pass 2 one-command pipeline after capturing 03_simulink/data/real/*.csv:
%   1. Identifies real motor & joystick parameters (run_stage2('real'))
%   2. Rebuilds & validates motor_plant.slx against real step response
%   3. Rebuilds speed_ctrl.slx & runs all 19 Stage 3 MIL tests
%   4. Regenerates 04_firmware/app_generated/ctrl_codegen.c & runs Stage 4 SIL back-to-back check
root = setup_paths();
realDir = fullfile(root, 'data', 'real');
assert(exist(realDir, 'dir') == 7, 'Directory %s not found. Capture real CSVs first using capture_log.py.', realDir);

fprintf('\n=== [1/4] Stage 2 Identification on Real Hardware Data ===\n');
out.s2 = run_stage2('real');
s2_05_build_plant();
out.s2_val = s2_06_validate_plant(realDir);

fprintf('\n=== [2/4] Stage 3 Rebuild Controller & MIL Test Suite ===\n');
s3_01_build_speed_ctrl();
out.s3_if = s3_02_check_interface();
out.s3 = run_stage3_tests();

fprintf('\n=== [3/4] Stage 4 Regenerate C Code & SIL Back-to-Back Verification ===\n');
out.s4 = run_stage4();

fprintf('\n=== [4/4] Done! Updated C code is in 04_firmware/app_generated/ ===\n');
end
