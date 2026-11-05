function r = s2_06_validate_plant(dataDir)
%S2_06_VALIDATE_PLANT  Simulate motor_plant.slx with the held-out duty profile (step_25_r2) and
%   compare to the measured rpm: NRMSE fit must be >= 85 % (model incl. encoder quantisation).
%   Run s2_05_build_plant first. Encoder phase differs run to run, so expect a slightly lower
%   fit than the pure-math fit in s2_02.
root = setup_paths();  par = init_params();  load_params_to_base(par);
if nargin < 1, dataDir = fullfile(root,'data','synthetic'); end
mdl = 'motor_plant';
if ~isfile(fullfile(root,'plant','motor_plant.slx')), s2_05_build_plant(); end
load_system(fullfile(root,'plant','motor_plant.slx'));

[t, duty, dN] = stage2_lib('load_step_log', fullfile(dataDir,'step_25_r2.csv'));
ymeas = stage2_lib('counts_to_rpm', dN, par.Nrev, par.Ts);
assignin('base','PL_td', par.td);
assignin('base','plant_duty_ts', timeseries(duty, t));
set_param(mdl,'StopTime', num2str(t(end)));
out = sim(mdl);
ts  = out.get('plant_out');
ysim = interp1(ts.Time, double(ts.Data(:)), t, 'previous', 'extrap');
fit = 100*(1 - norm(ymeas-ysim)/norm(ymeas-mean(ymeas)));
peakErr = abs(max(ymeas)-max(ysim))/max(ysim)*100;
f = figure('Visible','off'); plot(t,ymeas,'.',t,ysim,'-'); grid on; legend('measured','Simulink');
title(sprintf('motor\\_plant vs measured, fit %.1f %%',fit)); saveas(f, fullfile(dataDir,'plant_validation.png')); close(f);
r = struct('fit',fit,'peakErr',peakErr,'pass',fit >= 85);
fprintf('[S2-06] Simulink plant vs measured: fit = %.1f %% (need >= 85), peak error %.1f %% -> %s\n', ...
        fit, peakErr, ternary(r.pass,'PASS','FAIL'));
end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
