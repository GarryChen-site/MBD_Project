function r = s2_01_static_map(dataDir)
%S2_01_STATIC_MAP  Duty -> steady-state rpm and dead-zone fit (Stage 2 section 3.3).
%   Reads static_Dxx.csv (steady state = mean of the last 1 s of the hold).
%   Returns r.K, r.dz, r.R2 and the table. Writes figure data/<dir>/static_map.png
par = init_params();  Ts = par.Ts;
files = dir(fullfile(dataDir,'static_D*.csv'));
assert(~isempty(files), 'No static_D*.csv in %s', dataDir);
D = zeros(numel(files),1);  W = D;
for i = 1:numel(files)
    [t,duty,dN] = stage2_lib('load_step_log', fullfile(dataDir,files(i).name));
    rpm = stage2_lib('counts_to_rpm', dN, par.Nrev, Ts);
    hold_idx = duty == max(duty);
    last = find(hold_idx, 1, 'last');  first = max(last - round(1.0/Ts) + 1, 1);   % last 1 s of hold
    D(i) = max(duty);  W(i) = mean(rpm(first:last));
end
[D, o] = sort(D);  W = W(o);
[K, dz, R2] = stage2_lib('fit_static_map', D, W);
r = struct('duty',D,'rpm_ss',W,'K',K,'dz',dz,'R2',R2);
f = figure('Visible','off');  plot(D,W,'o-',D,K*max(D-dz,0),'--'); grid on
xlabel('duty [%]'); ylabel('rpm_{ss}'); title(sprintf('K=%.3f rpm/%%, dz=%.2f %%, R^2=%.4f',K,dz,R2));
saveas(f, fullfile(dataDir,'static_map.png')); close(f);
fprintf('[S2-01] K = %.3f rpm/%%   dz = %.2f %%   R2 = %.4f\n', K, dz, R2);
end
