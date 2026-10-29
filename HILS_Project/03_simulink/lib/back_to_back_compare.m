function report = back_to_back_compare(y_ref, y_test, absTol, relTol)
%BACK_TO_BACK_COMPARE  Stage 4 section 3 MIL-vs-SIL/PIL equivalence check.
%   y_ref, y_test : structs with signal vectors (duty_cmd_pct, speed_req_rpm, fault_bitmap, mode, ...)
%   Rules:
%     - integer/enum/bitmask signals (fault_bitmap, mode): exact equality (max_abs == 0)
%     - single float signals: |y_ref - y_test| <= max(absTol, relTol * max(|y_ref|))
if nargin < 3 || isempty(absTol), absTol = 1e-4; end
if nargin < 4 || isempty(relTol), relTol = 1e-4; end

sigs = {'duty_cmd_pct','speed_req_rpm','fault_bitmap','mode','s1_pct','p2_pct','e_pct','p_cmd_pct'};
intSigs = {'fault_bitmap','mode'};

rows = cell(numel(sigs), 6);
allPass = true;
for i = 1:numel(sigs)
    s = sigs{i};
    a = double(y_ref.(s)(:));
    b = double(y_test.(s)(:));
    n = min(numel(a), numel(b));
    d = abs(a(1:n) - b(1:n));
    maxAbs = max(d);
    rmsErr = sqrt(mean(d.^2));
    denom  = max(max(abs(a(1:n))), 1e-12);
    maxRel = maxAbs / denom;
    if ismember(s, intSigs)
        ok = (maxAbs == 0);
        fv = find(d > 0, 1, 'first');
    else
        bound = max(absTol, relTol * abs(a(1:n)));
        ok = all(d <= bound);
        fv = find(d > bound, 1, 'first');
    end
    if isempty(fv), fv = 0; end
    rows(i,:) = {s, maxAbs, rmsErr, maxRel, fv, ok};
    allPass = allPass && ok;
end

report = struct('signals', {sigs}, 'rows', {rows}, 'pass', allPass, ...
                'max_abs', max([rows{:,2}]), 'max_rms', max([rows{:,3}]));
end
