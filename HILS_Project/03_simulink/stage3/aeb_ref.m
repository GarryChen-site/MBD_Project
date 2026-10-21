function [state, flags] = aeb_ref(d_mm, valid, w_meas_rpm, p)
%AEB_REF  Reference (golden) AEB state machine, one call for a whole 50 ms-tick sequence (Stage 3 section 3.7).
%   state: 0 NORMAL, 1 WARNING, 2 BRAKING (per tick).  flags.tof_invalid: F_TOF_INVALID per tick.
%   p: .dw .db .dh [mm]  .Ndeb .Nrel [ticks]  .Tinv [ticks]  .wstop [rpm]
%   Distance-only version (TTC left as an extension). Braking is NEVER entered from invalid data.
n = numel(d_mm);  state = zeros(n,1);  inv = false(n,1);
s = 0; cW = 0; cB = 0; cR = 0; cInv = 0;
for k = 1:n
    if valid(k), cInv = 0; else, cInv = cInv + 1; end
    inv(k) = cInv >= p.Tinv;
    if valid(k)
        d = d_mm(k);
        cW = (d < p.dw) * (cW + 1);                 % consecutive samples below warn distance
        cB = (d < p.db) * (cB + 1);
        cR = (d > p.dw + p.dh) * (cR + 1);          % consecutive samples above release distance
        switch s
            case 0
                if cW >= p.Ndeb, s = 1; end
            case 1
                if cB >= p.Ndeb, s = 2;
                elseif cR >= p.Nrel, s = 0; end
            case 2
                if cR >= p.Nrel && w_meas_rpm(k) < p.wstop, s = 0; end
        end
    end
    state(k) = s;
end
flags.tof_invalid = inv;
end
