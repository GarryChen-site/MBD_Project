function par = init_params()
%INIT_PARAMS  Single source of truth for all P-xxx parameters (PRODUCT_SPEC.md section 4).
%   Defaults are PLACEHOLDERS marked "% TBD" until Stage 2 produces measured values.
%   If data/params_identified.mat exists (written by stage2/s2_05_save_params.m),
%   its fields overwrite the defaults, so nothing else needs editing.
%
%   par.src = 'default' | 'identified'  tells you which values you are using.

% ---- design constants -------------------------------------------------------
par.Ts      = 0.010;        % P-001 control period [s]
par.Vbat    = 6.0;          % P-003 [V]                         % TBD measure M4
par.Vrated  = 3.0;          % P-004 [V]
par.Dmax    = 100*par.Vrated/par.Vbat;   % P-005 duty cap [%]  (=50)
par.Nrev    = 4200;         % P-006 counts per output rev (x4)   % TBD measure T0-6
par.fpwm    = 20e3;         % P-015

% ---- plant (Stage 2 fills these) -------------------------------------------
par.K       = 4.5;          % P-007 [rpm/%]                      % TBD identify
par.tau     = 0.12;         % P-008 [s]                          % TBD identify
par.td      = 0.0;          % dead time [s] (integer multiple of Ts) % TBD identify
par.dz      = 8;            % P-009 duty dead zone [%]           % TBD identify

% ---- speed controller (Stage 3 derives these from K,tau,Td) ----------------
par.Mavg    = 1;            % speed-estimator window in samples
par.TtFactor= 1.0;          % anti-windup Tt = TtFactor * tau
par.omega_limp = 30;        % P-012 [rpm]                        % TBD HARA-lite
par.Rup     = 400;          % pedal rate limit up [%/s]
par.Rdn     = 600;          % pedal rate limit down [%/s]
par.tau_f   = 0.05;         % pedal low-pass time constant [s]

% ---- pedal conditioning (D-004), Stage 2 joystick calibration fills these --
par.pedal.mode = 'REVERSE'; % P-019 'REVERSE' | 'HALF'
par.pedal.cX   = 2048;      par.pedal.cY   = 2048;      % TBD centre counts
par.pedal.dbX  = 60;        par.pedal.dbY  = 60;        % P-018 TBD dead band counts
par.pedal.maxX = 3900;      par.pedal.maxY = 3900;      % TBD diagonal maxima
par.pedal.alo  = 100;       par.pedal.ahi  = 3995;      % raw range-check band (~2.5 % margin)
par.pedal.Dpl  = 15;        % P-013 plausibility threshold [%]   % TBD choose_dpl
par.pedal.Npl  = 5;         % P-013 debounce ticks
par.pedal.Nrange = 5;       % range-fault debounce ticks
par.pedal.Nclear = 100;     % ticks of clean signal before leaving DEGRADED (1 s)

par.src = 'default';

% ---- overlay measured values -------------------------------------------------
f = fullfile(fileparts(mfilename('fullpath')),'data','params_identified.mat');
if isfile(f)
    S = load(f);                 % contains struct 'ident' with any of the fields above
    fn = fieldnames(S.ident);
    for i = 1:numel(fn)
        if isstruct(S.ident.(fn{i}))
            sf = fieldnames(S.ident.(fn{i}));
            for j = 1:numel(sf), par.(fn{i}).(sf{j}) = S.ident.(fn{i}).(sf{j}); end
        else
            par.(fn{i}) = S.ident.(fn{i});
        end
    end
    par.src = 'identified';
end

% ---- derived ---------------------------------------------------------------
par.omega_max = par.K * (par.Dmax - par.dz);   % P-011 [rpm]
end
