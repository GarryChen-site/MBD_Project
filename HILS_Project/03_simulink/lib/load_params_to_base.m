function load_params_to_base(par)
%LOAD_PARAMS_TO_BASE  Publish parameters as base-workspace variables used by the Simulink blocks.
%   Controller (P_*) and plant (PL_*) are separate so tests can perturb the plant only.
if nargin < 1, par = init_params(); end
Td     = loop_delay(par.Ts, par.Mavg);
lambda = choose_lambda(par.tau, Td);
[Kp, Ki, Kb] = tune_pi_lambda(par.K, par.tau, lambda, par.Ts, par.TtFactor);
a = @(n,v) assignin('base', n, v);
% controller
a('P_Ts', par.Ts);          a('P_K', par.K);            a('P_dz', par.dz);
a('P_Kp', Kp);              a('P_Ki', Ki);              a('P_Kb', Kb);
a('P_Dmax', par.Dmax);      a('P_omega_max', par.omega_max);
a('P_alpha', exp(-par.Ts/par.tau_f));
a('P_Rup', par.Rup);        a('P_Rdn', par.Rdn);
a('P_cal', cal_to_vec(par.pedal));
a('P_fm',  single([par.pedal.Dpl par.pedal.Npl par.pedal.Nrange par.pedal.Nclear par.omega_max par.omega_limp]));
% plant (what the real motor is; in closed-loop FbDelay already models 1*Ts sample delay)
a('PL_K', par.K);           a('PL_tau', par.tau);       a('PL_td', max(par.td - par.Ts, 0));
a('PL_dz', par.dz);         a('PL_Nrev', par.Nrev);
a('P_lambda', lambda);
end
