function mdl = s3_01_build_speed_ctrl()
%S3_01_BUILD_SPEED_CTRL  Programmatically build controller/speed_ctrl.slx (closed loop MIL).
%
%  scn_adcX/scn_adcY (From Workspace) -> Controller -> Plant -> FbDelay(Unit Delay) -> Controller/3
%  Top-level Outports log: duty_cmd_pct speed_req_rpm fault_bitmap mode s1_pct p2_pct e_pct
%  p_cmd_pct speed_meas_rpm  (Dataset format, read by run_mil).
%  Needs Stage-2 parameters (init_params). Re-run whenever the block structure changes.
root = setup_paths();  par = init_params();  load_params_to_base(par);
mdl = 'speed_ctrl';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','ode4','FixedStep','P_Ts/10','StopTime','5', ...
    'SaveOutput','on','OutputSaveName','yout','SaveTime','on','TimeSaveName','tout', ...
    'SaveFormat','Dataset','ReturnWorkspaceOutputs','on','ParameterPrecisionLossMsg','none');
assignin('base','scn_adcX', timeseries(par.pedal.cX*ones(501,1), (0:500)'*par.Ts));
assignin('base','scn_adcY', timeseries(par.pedal.cY*ones(501,1), (0:500)'*par.Ts));

fw = @(n,v) add_block('simulink/Sources/From Workspace',[mdl '/' n],'VariableName',v, ...
        'SampleTime','P_Ts','Interpolate','off','OutputAfterFinalValue','Holding final value');
fw('adcX_in','scn_adcX');  fw('adcY_in','scn_adcY');

add_block('built-in/Subsystem',[mdl '/Controller']);
build_controller_subsystem([mdl '/Controller']);
set_param([mdl '/Controller'],'TreatAsAtomicUnit','on');   % Stage 4 generates code from this block
add_block('built-in/Subsystem',[mdl '/Plant']);
build_plant_subsystem([mdl '/Plant']);
add_block('simulink/Discrete/Unit Delay',[mdl '/FbDelay'],'SampleTime','P_Ts','InitialCondition','0');

add_line(mdl,'adcX_in/1','Controller/1','autorouting','on');
add_line(mdl,'adcY_in/1','Controller/2','autorouting','on');
add_line(mdl,'FbDelay/1','Controller/3','autorouting','on');   % unit delay breaks the algebraic loop
add_line(mdl,'Controller/1','Plant/1','autorouting','on');
add_line(mdl,'Plant/1','FbDelay/1','autorouting','on');

outs = {'duty_cmd_pct','speed_req_rpm','fault_bitmap','mode','s1_pct','p2_pct','e_pct','p_cmd_pct'};
for i = 1:numel(outs)
    add_block('simulink/Sinks/Out1',[mdl '/' outs{i}],'SampleTime','P_Ts');
    add_line(mdl,sprintf('Controller/%d',i),[outs{i} '/1'],'autorouting','on');
end
add_block('simulink/Sinks/Out1',[mdl '/speed_meas_rpm'],'SampleTime','P_Ts');
add_line(mdl,'Plant/1','speed_meas_rpm/1','autorouting','on');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, fullfile(root,'controller','speed_ctrl.slx'));
fprintf('[S3-01] built %s (lambda = %.4f s)\n', fullfile(root,'controller','speed_ctrl.slx'), evalin('base','P_lambda'));
end
