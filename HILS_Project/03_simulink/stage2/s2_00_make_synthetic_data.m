function s2_00_make_synthetic_data(outDir, seed)
%S2_00_MAKE_SYNTHETIC_DATA  Fabricate Stage-2 datasets with the EXACT file formats the real
%   firmware loggers produce, so every later script can be run and verified BEFORE hardware
%   exists. SYNTHETIC DATA PROVES THE SCRIPTS, NOT THE MOTOR. Replace by real logs in data/real/.
%
%   Files written into outDir (default data/synthetic):
%     static_D05.csv ... static_D50.csv   t_ms,duty_permille,dN    (one hold per duty)
%     step_15_r1.csv ... step_35_r3.csv   t_ms,duty_permille,dN    (3 amplitudes x 3 repeats)
%     joy_rest.csv, joy_diag_full.csv, joy_diag_valid.csv, joy_xonly.csv   t_ms,adcX,adcY
%     truth.mat  (the hidden "true" parameters, for checking only)
if nargin < 1, outDir = fullfile(fileparts(fileparts(mfilename('fullpath'))),'data','synthetic'); end
if nargin < 2, seed = 1; end
if ~exist(outDir,'dir'), mkdir(outDir); end
rng(seed);
par = init_params();
truth = struct('K',4.6,'tau',0.115,'nd',1,'dz',7.5,'Nrev',par.Nrev);
Ts = par.Ts;

% ---------------- motor runs -------------------------------------------------
for D = 5:5:50
    write_run(fullfile(outDir, sprintf('static_D%02d.csv',D)), truth, Ts, 0.2, 2.5, 0.0, D);
end
for D = [15 25 35]
    for r = 1:3
        tr = truth;  tr.K = truth.K*(1 + 0.02*randn);    % run-to-run spread
        write_run(fullfile(outDir, sprintf('step_%02d_r%d.csv',D,r)), tr, Ts, 0.5, 2.0, 1.0, D);
    end
end

% ---------------- joystick sessions ------------------------------------------
cX = 2051; cY = 2039; sg = 3.0;           % hidden truth
maxX = 3880; maxY = 3835;                 % diagonal full-push readings
fs = 100;
% rest: 10 s
n = 10*fs;  write_joy(fullfile(outDir,'joy_rest.csv'), cX+sg*randn(n,1), cY+sg*randn(n,1));
% diagonal full pushes (10x), peak u = 1
[X,Y] = pushes(10, ones(10,1), true, cX,cY,maxX,maxY,sg,fs);   write_joy(fullfile(outDir,'joy_diag_full.csv'),X,Y);
% valid diagonal pushes (50x): peaks 0.4..1, durations slow/normal/fast, with X-Y skew
[X,Y] = pushes(50, 0.4+0.6*rand(50,1), true, cX,cY,maxX,maxY,sg,fs); write_joy(fullfile(outDir,'joy_diag_valid.csv'),X,Y);
% X only (fault demo, 10x), peak u = 0.6 .. 0.7
[X,Y] = pushes(10, 0.6+0.1*rand(10,1), false, cX,cY,maxX,maxY,sg,fs); write_joy(fullfile(outDir,'joy_xonly.csv'),X,Y);

save(fullfile(outDir,'truth.mat'),'truth','cX','cY','maxX','maxY','sg');
fprintf('Synthetic data written to %s\n', outDir);
end

function write_run(path, tr, Ts, tpre, tstep, tpost, Dstep)
% Simulate the true plant at 1 ms and emit the 10 ms log exactly like firmware would.
dt = 1e-3;  N = round((tpre+tstep+tpost)/Ts);
a = exp(-dt/tr.tau);  nd = round(tr.nd*Ts/dt);
t_ms = (0:N-1)'*10;  duty_pm = zeros(N,1);  dN = zeros(N,1);
omega = 0; pos = rand();                         % rpm, revolutions (random phase)
buf = zeros(nd+1,1);                             % delay line in 1 ms steps
for k = 1:N
    tk = (k-1)*Ts;
    D = Dstep * (tk >= tpre && tk < tpre+tstep);
    duty_pm(k) = round(D*10);
    posStart = pos*tr.Nrev;
    for m = 1:round(Ts/dt)
        ueff = max(duty_pm(k)/10 - tr.dz, 0);
        buf = [ueff; buf(1:end-1)];
        omega = a*omega + tr.K*(1-a)*buf(end);
        omega_inst = omega + (omega > 0.5)*2.5*randn;    % brush/gearbox jitter breaks deterministic floor() beat
        pos = pos + omega_inst/60*dt;
    end
    dN(k) = floor(pos*tr.Nrev) - floor(posStart);
end
writetable(table(t_ms,duty_pm,dN,'VariableNames',{'t_ms','duty_permille','dN'}), path);
end

function [X,Y] = pushes(np, peaks, diagonal, cX,cY,maxX,maxY,sg,fs)
X = []; Y = [];
for i = 1:np
    dur = [2.0 1.0 0.4]; T = dur(mod(i-1,3)+1);          % slow / normal / fast
    nUp = round(T*fs); nHold = round(0.2*fs); nRest = round(0.8*fs);
    up = 0.5*(1-cos(pi*(0:nUp-1)'/nUp));
    u = [zeros(nRest,1); up; ones(nHold,1); flipud(up); zeros(nRest,1)] * peaks(i);
    skew = round(fs*0.03*rand());                            % 0..30 ms later on Y
    gy = 1 + 0.05*(2*rand-1);                                % +-5 % gain mismatch
    uy = [zeros(skew,1); u(1:end-skew)];
    x = cX + u *(maxX-cX) + sg*randn(size(u));
    if diagonal
        y = cY + gy*uy*(maxY-cY) + sg*randn(size(u));
    else
        y = cY + 0.02*u*(maxX-cX) + sg*randn(size(u));       % 2 % mechanical leakage into Y
    end
    X = [X; x]; Y = [Y; y]; %#ok<AGROW>
end
X = round(min(max(X,0),4095)); Y = round(min(max(Y,0),4095));
end

function write_joy(path, X, Y)
t_ms = (0:numel(X)-1)'*10;  adcX = round(X); adcY = round(Y);
writetable(table(t_ms,adcX,adcY), path);
end
