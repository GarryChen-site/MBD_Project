function r = run_mil(tsX, tsY, stopTime, over)
%RUN_MIL  Run speed_ctrl.slx closed loop with the given raw ADC timeseries.
%   over : struct of base-workspace variable overrides, e.g. struct('P_Rup',1e5,'PL_K',3.0)
%   r    : struct with fields t and one column per logged signal (double).
root = setup_paths();
if ~isfile(fullfile(root,'controller','speed_ctrl.slx')), s3_01_build_speed_ctrl(); end
if ~bdIsLoaded('speed_ctrl'), load_system(fullfile(root,'controller','speed_ctrl.slx')); end
par = init_params();
load_params_to_base(par);
mdl = 'speed_ctrl';
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(stopTime));
if numel(tsX.Time) > 1, tsX.Time = [tsX.Time(1); tsX.Time(2:end) - 1e-7]; end
if numel(tsY.Time) > 1, tsY.Time = [tsY.Time(1); tsY.Time(2:end) - 1e-7]; end
in = in.setVariable('scn_adcX', tsX);
in = in.setVariable('scn_adcY', tsY);
if nargin >= 4 && ~isempty(over)
    fn = fieldnames(over);
    for i = 1:numel(fn), in = in.setVariable(fn{i}, over.(fn{i})); end
end
out = sim(in);
names = {'duty_cmd_pct','speed_req_rpm','fault_bitmap','mode','s1_pct','p2_pct','e_pct','p_cmd_pct','speed_meas_rpm'};
tGrid = (0:round(stopTime/par.Ts))' * par.Ts;
r = struct('t', tGrid);
for i = 1:numel(names)
    el = out.yout{i};
    tEl = double(el.Values.Time(:));
    dEl = double(el.Values.Data(:));
    if numel(tEl) == numel(tGrid) && max(abs(tEl - tGrid)) < 1e-6
        r.(names{i}) = dEl;
    else
        r.(names{i}) = interp1(tEl, dEl, tGrid, 'nearest', 'extrap');
    end
end
end
