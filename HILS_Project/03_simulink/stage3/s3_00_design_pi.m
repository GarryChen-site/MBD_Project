function r = s3_00_design_pi()
%S3_00_DESIGN_PI  Lambda-tune the speed PI from Stage-2 K, tau and verify by hand (Stage 3 sections 3.2-3.3).
%   Prints Kp, Ki, Kb, lambda and, if Control System Toolbox is available, the phase margin
%   and step metrics of the delayed continuous loop (acceptance A3-2: PM >= 60 deg).
par = init_params();
Td = loop_delay(par.Ts, par.Mavg);
lambda = choose_lambda(par.tau, Td);
[Kp, Ki, Kb, info] = tune_pi_lambda(par.K, par.tau, lambda, par.Ts, par.TtFactor);
fprintf('[S3-00] K=%.3f tau=%.4f  Td=%.4f -> lambda=%.4f s | Kp=%.4f Ki=%.4f Kb=%.3f | expected tr=%.3f ts=%.3f\n', ...
        par.K, par.tau, Td, lambda, Kp, Ki, Kb, info.tr_expected, info.ts_expected);
r = struct('Kp',Kp,'Ki',Ki,'Kb',Kb,'lambda',lambda,'Td',Td,'info',info);
if license('test','Control_Toolbox') && ~isempty(which('margin'))
    G = tf(par.K, [par.tau 1], 'InputDelay', Td);
    C = pid(Kp, Ki);
    L = C*G;  T = feedback(L, 1);
    [~, Pm] = margin(L);
    S = stepinfo(T);
    r.PM = Pm;  r.step = S;  r.pass = Pm >= 60;
    fprintf('[S3-00] phase margin = %.1f deg (need >= 60)   stepinfo: tr=%.3f s, overshoot=%.2f %% -> %s\n', ...
            Pm, S.RiseTime, S.Overshoot, ternary(r.pass,'PASS','FAIL'));
else
    fprintf('[S3-00] Control System Toolbox not found: margin check skipped.\n');
end
end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
