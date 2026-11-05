function varargout = stage2_lib(fn, varargin)
%STAGE2_LIB  Small helper functions for Stage 2 in one file (call as stage2_lib('name', args...)).
%   Names: counts_to_rpm, load_step_log, fit_static_map, identify_motor_step,
%          identify_arx_first_order, sensor_stats, fit_pct
switch fn
    case 'counts_to_rpm',            [varargout{1:nargout}] = counts_to_rpm(varargin{:});
    case 'load_step_log',            [varargout{1:nargout}] = load_step_log(varargin{:});
    case 'fit_static_map',           [varargout{1:nargout}] = fit_static_map(varargin{:});
    case 'identify_motor_step',      [varargout{1:nargout}] = identify_motor_step(varargin{:});
    case 'identify_arx_first_order', [varargout{1:nargout}] = identify_arx_first_order(varargin{:});
    case 'sensor_stats',             [varargout{1:nargout}] = sensor_stats(varargin{:});
    case 'fit_pct',                  [varargout{1:nargout}] = fit_pct(varargin{:});
    otherwise, error('stage2_lib: unknown function "%s"', fn);
end
end

function rpm = counts_to_rpm(dN, Nrev, Tw)
% rpm = 60*dN/(Nrev*Tw)   (Stage 2 section 3.4). SAME formula as in firmware.
rpm = 60 * double(dN) ./ (Nrev * Tw);
end

function [t, duty_pct, dN] = load_step_log(csvPath, ~, ~)
% CSV columns: t_ms,duty_permille,dN  (header line required). Returns t [s], duty [%], dN [counts].
T = readtable(csvPath);
t = (T.t_ms - T.t_ms(1)) / 1000;
duty_pct = T.duty_permille / 10;
dN = T.dN;
end

function [K, dz, R2] = fit_static_map(duty_pct, rpm_ss)
% Linear regression above the dead zone (Stage 2 section 3.3): rpm = K*D - K*dz.
duty_pct = duty_pct(:);  rpm_ss = rpm_ss(:);
use = rpm_ss > 0.1 * max(rpm_ss);              % exclude the flat dead-zone part
p = polyfit(duty_pct(use), rpm_ss(use), 1);
K = p(1);  dz = -p(2) / p(1);
yhat = polyval(p, duty_pct(use));  y = rpm_ss(use);
R2 = 1 - sum((y-yhat).^2) / sum((y-mean(y)).^2);
end

function [K, tau, nd, fitPct, info] = identify_motor_step(t, y_rpm, duty_pct, opts)
% First-order + delay fit (Stage 2 section 3.5, ARX form simulated exactly):
%   y[k+1] = a*y[k] + K*(1-a)*u_eff[k-nd],  a = exp(-Ts/tau),  u_eff = max(D-dz,0)
% Searches nd = 0..opts.ndMax; for each nd minimises the squared error over [K,tau].
% Returns td = nd*Ts in info.td. opts.dz = dead zone [%].
Ts = median(diff(t));
u = max(duty_pct(:) - opts.dz, 0);   y = y_rpm(:);
if ~isfield(opts,'ndMax'), opts.ndMax = 3; end
best = struct('J', inf);
for nd = 0:opts.ndMax
    f = @(th) sum((y - simulate(th, u, Ts, nd, y(1))).^2);
    th0 = [ max(y)/max(max(u),eps), 0.1 ];
    th = fminsearch(@(q) f([abs(q(1)) abs(q(2))+1e-4]), th0, optimset('Display','off','TolX',1e-6,'TolFun',1e-9));
    th = [abs(th(1)) abs(th(2))+1e-4];
    J = f(th);
    if J < best.J, best = struct('J',J,'K',th(1),'tau',th(2),'nd',nd); end
end
K = best.K; tau = best.tau; nd = best.nd;
yhat = simulate([K tau], u, Ts, nd, y(1));
fitPct = fit_pct(y, yhat);
e = y - yhat;
N = numel(e);  ac = zeros(20,1);
for L = 1:20, ac(L) = sum(e(1+L:end).*e(1:end-L)) / sum(e.^2); end
info.yhat = yhat; info.residual = e; info.autocorr = ac;
info.band99 = 2.58/sqrt(N); info.td = nd*Ts; info.Ts = Ts;
end

function yhat = simulate(th, u, Ts, nd, y0)
K = th(1); tau = th(2); a = exp(-Ts/tau);
n = numel(u);  yhat = zeros(n,1);  w = y0;
ud = [zeros(nd,1); u(1:end-nd)];
for k = 1:n
    w_next = a*w + K*(1-a)*ud(k);
    yhat(k) = 0.5*(w + w_next);
    w = w_next;
end
end

function [a, b, tauHat, Khat] = identify_arx_first_order(y, u, Ts, nd)
% Discrete least squares  y[k+1] = a y[k] + b u[k-nd]  (theta = (Phi'Phi)^-1 Phi' Y).
y = y(:); u = u(:);
ud = [zeros(nd,1); u(1:end-nd)];
Phi = [y(1:end-1) ud(1:end-1)];  Y = y(2:end);
th = (Phi'*Phi) \ (Phi'*Y);
a = th(1); b = th(2);
tauHat = -Ts/log(a);  Khat = b/(1-a);
end

function f = fit_pct(y, yhat)
% NRMSE fit [%] = 100*(1 - ||y-yhat|| / ||y-mean(y)||)  (same as MATLAB compare/goodnessOfFit)
f = 100 * (1 - norm(y(:)-yhat(:)) / norm(y(:)-mean(y(:))));
end

function s = sensor_stats(x, label)
% mean, std, min, max, histogram counts and a crude normality hint (|skew|<0.5 and |excess kurt|<1).
x = double(x(:));
s.label = label; s.mean = mean(x); s.std = std(x); s.min = min(x); s.max = max(x);
s.n = numel(x);
[s.hist_counts, s.hist_edges] = histcounts(x, 20);
m = mean(x); sd = std(x);
sk = mean(((x-m)/sd).^3);  ku = mean(((x-m)/sd).^4) - 3;
s.skew = sk; s.exkurt = ku; s.looksNormal = abs(sk) < 0.5 && abs(ku) < 1;
end
