function [c, active] = debounce_step(c, active, cond, Nset)
%DEBOUNCE_STEP  Up/down counter debounce (Stage 3 section 3.5), one tick.
%   c      : counter state (0..Nset)
%   active : fault flag state; set when c >= Nset, cleared only when c == 0
%   cond   : raw fault condition this tick
if cond
    c = min(c + 1, Nset);
else
    c = max(c - 1, 0);
end
if c >= Nset, active = true; end
if c == 0,    active = false; end
end
