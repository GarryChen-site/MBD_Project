function [s1, s2, p2, e, rangeBad] = pedal_cond_core(adcX, adcY, cal)
%PEDAL_COND_CORE  Scalar, code-generation-friendly PedalCond (Stage 3 section 3.1.1).
%   cal = [cX cY dbX dbY maxX maxY modeId alo ahi]   (modeId: 1 = REVERSE, 2 = HALF)
%   Used by BOTH pedal_cond_ref (hand check) and the Simulink MATLAB Function block,
%   so acceptance A3-9 compares the model against this single definition.
aX = single(adcX);  aY = single(adcY);
nX = deflection(aX, cal(1), cal(3), cal(5));
nY = deflection(aY, cal(2), cal(4), cal(6));
s1 = single(100) * nX;
if cal(7) < 1.5            % REVERSE: 100 % -> 0 %
    s2 = single(100) * (single(1) - nY);
    p2 = single(100) - s2;
else                       % HALF: 0 % -> 50 %
    s2 = single(50) * nY;
    p2 = min(single(2) * s2, single(100));
end
e = abs(s1 - p2);
% range check on RAW values, before any clamping
rangeBad = (aX < cal(8)) || (aX > cal(9)) || (aY < cal(8)) || (aY > cal(9));
end

function n = deflection(a, c, db, amax)
d = a - single(c);
if d <= single(db)
    n = single(0);
else
    n = (d - single(db)) / (single(amax) - single(c) - single(db));
    n = min(max(n, single(0)), single(1));
end
end
