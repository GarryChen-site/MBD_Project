function mdl = s4_01_build_codegen_model()
%S4_01_BUILD_CODEGEN_MODEL  Build standalone controller model controller/ctrl_codegen.slx for Stage 4 C codegen.
%   Contains the exact same Controller algorithm built by build_controller_subsystem,
%   with single/uint16/uint8 root ports and FixedStepDiscrete solver at P_Ts (10 ms).
root = setup_paths();
par = init_params();
load_params_to_base(par);

mdl = 'ctrl_codegen';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);

% Populate root model with the exact Controller blocks
build_controller_subsystem(mdl);

% Enforce explicit single data type and 10 ms sample time on root Inports
inNames = {'pedal1_adc','pedal2_adc','speed_meas_rpm'};
for i = 1:numel(inNames)
    set_param([mdl '/' inNames{i}], 'OutDataTypeStr', 'single', 'SampleTime', 'P_Ts', 'Interpolate', 'off');
end

% Apply Embedded Coder STM32 configuration
ert_stm32(mdl);
set_param(mdl, 'StopTime', '5', 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
          'SaveTime', 'on', 'TimeSaveName', 'tout', 'SaveFormat', 'Dataset', ...
          'ReturnWorkspaceOutputs', 'on');

outPath = fullfile(root, 'controller', [mdl '.slx']);
save_system(mdl, outPath);
fprintf('[S4-01] built standalone codegen model %s\n', outPath);
end
