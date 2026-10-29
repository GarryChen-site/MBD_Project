function [s1, s2, p2, e] = pedal_cond_ref(adcX, adcY, cal)
%PEDAL_COND_REF  Vectorised golden reference of PedalCond (calls pedal_cond_core per sample).
%   cal: struct .cX .cY .dbX .dbY .maxX .maxY .mode ('REVERSE'|'HALF') [.alo .ahi optional]
v = cal_to_vec(cal);
n = numel(adcX);
s1 = zeros(size(adcX),'single'); s2 = s1; p2 = s1; e = s1;
for k = 1:n
    [s1(k), s2(k), p2(k), e(k)] = pedal_cond_core(adcX(k), adcY(k), v);
end
end
