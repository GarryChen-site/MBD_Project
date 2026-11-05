function m = step_metrics(t, y, yref, tol)
%STEP_METRICS  Step response metrics. t, y must START at the step instant.
%   m.rise (10-90 %), m.t10, m.t90, m.settle (tol band, default 2 %), m.overshoot [%], m.sse
%   y0 is taken as y(1); H = yref - y0.
if nargin < 4, tol = 0.02; end
t = t(:); y = y(:);
y0 = y(1);  H = yref - y0;
i10 = find((y - y0)/H >= 0.1, 1, 'first');
i90 = find((y - y0)/H >= 0.9, 1, 'first');
if isempty(i10) || isempty(i90)
    m = struct('rise',NaN,'t10',NaN,'t90',NaN,'settle',NaN,'overshoot',NaN,'sse',NaN); return
end
m.t10 = t(i10);  m.t90 = t(i90);  m.rise = m.t90 - m.t10;
outside = find(abs(y - yref) > tol*abs(H), 1, 'last');
if isempty(outside), m.settle = 0; else, m.settle = t(min(outside+1, numel(t))); end
m.overshoot = max(0, (max(y) - yref)/H*100);
n = max(1, round(0.1*numel(y)));
m.sse = yref - mean(y(end-n+1:end));
end
