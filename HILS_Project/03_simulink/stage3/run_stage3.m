function run_stage3()
%RUN_STAGE3  One command for Stage 3: design PI, build the Simulink model, check it, run all MIL tests.
%   Prerequisite: run_stage2('synthetic') (or 'real'). Order:
%     s3_00_design_pi -> s3_01_build_speed_ctrl -> s3_02_check_interface -> run_stage3_tests
setup_paths();
s3_00_design_pi();
s3_01_build_speed_ctrl();
s3_02_check_interface();
run_stage3_tests();
end
