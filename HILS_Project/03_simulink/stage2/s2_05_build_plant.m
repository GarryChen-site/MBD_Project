function mdl = s2_05_build_plant()
%S2_05_BUILD_PLANT  Programmatically build plant/motor_plant.slx (test harness around the Plant subsystem).
%   FromWorkspace(plant_duty_ts) -> Plant -> ToWorkspace(plant_out)
%   Run after run_stage2 so init_params() holds identified K, tau, td, dz, Nrev.
root = setup_paths();  par = init_params();  load_params_to_base(par);
mdl = 'motor_plant';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','ode4','FixedStep','P_Ts/10','StopTime','3');
assignin('base','plant_duty_ts', timeseries(zeros(301,1), (0:300)'*par.Ts));   % placeholder input

add_block('simulink/Sources/From Workspace',[mdl '/duty_in'], 'VariableName','plant_duty_ts', ...
          'SampleTime','P_Ts','Interpolate','off','OutputAfterFinalValue','Holding final value');
add_block('built-in/Subsystem',[mdl '/Plant']);
build_plant_subsystem([mdl '/Plant']);
add_block('simulink/Sinks/To Workspace',[mdl '/log_speed'],'VariableName','plant_out','SaveFormat','Timeseries');
add_line(mdl,'duty_in/1','Plant/1','autorouting','on');
add_line(mdl,'Plant/1','log_speed/1','autorouting','on');
Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, fullfile(root,'plant','motor_plant.slx'));
fprintf('[S2-05] built %s\n', fullfile(root,'plant','motor_plant.slx'));
end
