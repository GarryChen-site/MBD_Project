function [Dpl, sep] = choose_dpl(e_valid, e_fault)
%CHOOSE_DPL  Plausibility threshold from measured residuals (Stage 3 section 3.1.2).
%   Dpl = mean(e_valid) + 3*std(e_valid), plus margin above observed valid pushes (1.05*max).
%   sep = median(e_fault)/Dpl  must be >= 2 (acceptance A3-8).
e_valid = double(e_valid(:));  e_fault = double(e_fault(:));
Dpl = max([mean(e_valid) + 3*std(e_valid), prctile(e_valid, 99.9), 1.05*max(e_valid)]);
sep = median(e_fault) / Dpl;
end
