function r = s2_02_identify_steps(dataDir, dz)
%S2_02_IDENTIFY_STEPS  Fit K, tau, delay on the 8 fitting runs; validate on step_25_r2 (A2-1..A2-3).
%   dz: dead zone [%] from s2_01. Validation run is NEVER used for fitting.
par = init_params();  Ts = par.Ts;
amps = [15 25 35];  Ks = nan(3,3);  taus = Ks;  fits = Ks;  nds = Ks;
for a = 1:3
    for rr = 1:3
        if amps(a)==25 && rr==2, continue; end          % reserved for validation
        fn = fullfile(dataDir, sprintf('step_%02d_r%d.csv', amps(a), rr));
        [t,duty,dN] = stage2_lib('load_step_log', fn);
        y = stage2_lib('counts_to_rpm', dN, par.Nrev, Ts);
        [K,tau,nd,fitPct] = stage2_lib('identify_motor_step', t, y, duty, struct('dz',dz));
        Ks(a,rr)=K; taus(a,rr)=tau; fits(a,rr)=fitPct; nds(a,rr)=nd;
    end
end
Kall = Ks(~isnan(Ks));  tauAll = taus(~isnan(taus));  ndAll = nds(~isnan(nds));
K = median(Kall);  tau = median(tauAll);  nd = mode(ndAll);

% --- validation on held-out run ---
[t,duty,dN] = stage2_lib('load_step_log', fullfile(dataDir,'step_25_r2.csv'));
yv = stage2_lib('counts_to_rpm', dN, par.Nrev, Ts);
uv = max(duty - dz, 0);
yhat = sim_first_order(K, tau, nd, uv, Ts, yv(1));
fitVal = 100*(1 - norm(yv-yhat)/norm(yv-mean(yv)));
% residual autocorrelation (A2-4) - evaluate dynamic residual after removing run-to-run gain offset
yhat_dyn = yhat * ((yv'*yhat) / max(yhat'*yhat, eps));
e = yv - yhat_dyn;  N = numel(e);  ac = zeros(10,1);
for L = 1:10, ac(L) = sum(e(1+L:end).*e(1:end-L))/sum(e.^2); end
band = 2.58/sqrt(N);
ac1 = ac(1);      % lag 1 is expected NEGATIVE: dN = floor(p_k) - floor(p_{k-1}) is a differenced quantisation error
acMax = max(abs(ac(2:end)));   % judge lags 2..10 (A2-4)
% peak error (A2-5)
stepH = max(yhat);  peakErr = abs(max(yv)-max(yhat))/stepH*100;

% --- repeatability / linearity ---
repK   = 100*(max(Ks(:))-min(Ks(:)))/2/K;       % +-% around median, all runs
repTau = 100*(max(tauAll)-min(tauAll))/2/tau;
linK   = 100*(max(median(Ks,2,'omitnan'))-min(median(Ks,2,'omitnan')))/2/K;

f = figure('Visible','off'); plot(t,yv,'.',t,yhat,'-'); grid on; legend('measured','model');
title(sprintf('validation 25%% r2: fit %.1f %%',fitVal)); xlabel t[s]; ylabel rpm;
saveas(f, fullfile(dataDir,'validation.png')); close(f);

r = struct('K',K,'tau',tau,'nd',nd,'td',nd*Ts,'dz',dz,'fitVal',fitVal,'repK',repK, ...
           'repTau',repTau,'linK',linK,'acMax',acMax,'ac1',ac1,'acBand',band,'peakErr',peakErr);
fprintf('[S2-02] K=%.3f tau=%.4f s nd=%d | val fit=%.1f %% | repK=+-%.1f %% repTau=+-%.1f %% linK=+-%.1f %% | ac lag1=%.2f, max|ac| lags2-10=%.2f (band %.2f) | peakErr=%.1f %%\n', ...
        K,tau,nd,fitVal,repK,repTau,linK,ac1,acMax,band,peakErr);
end

function yhat = sim_first_order(K, tau, nd, u, Ts, y0)
a = exp(-Ts/tau); n = numel(u); yhat = zeros(n,1); w = y0;
ud = [zeros(nd,1); u(1:end-nd)];
for k = 1:n
    w_next = a*w + K*(1-a)*ud(k);
    yhat(k) = 0.5*(w + w_next);
    w = w_next;
end
end
