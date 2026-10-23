function results = run_stage2(source)
%RUN_STAGE2  Run all Stage-2 analysis steps and print an acceptance table (A2-1 ... A2-8).
%   run_stage2('synthetic')  : generates fake data first, proves the scripts work (default)
%   run_stage2('real')       : reads data/real/*.csv produced by the firmware loggers
%
%   On success writes data/params_identified.mat, which init_params() then picks up.
if nargin < 1, source = 'synthetic'; end
root = setup_paths();
dataDir = fullfile(root,'data',source);
if strcmp(source,'synthetic')
    s2_00_make_synthetic_data(dataDir);
    pf = fullfile(root,'data','params_identified.mat');
    if isfile(pf), delete(pf); end            % don't let stale values influence the run
else
    % Pre-seed measured encoder resolution Nrev = 1426 from smoketest/encoder/encoder_test_log.txt
    ident = struct('Nrev', 1426);
    save(fullfile(root,'data','params_identified.mat'),'ident');
end
st = s2_01_static_map(dataDir);
id = s2_02_identify_steps(dataDir, st.dz);
jc = s2_03_joystick_calibration(dataDir);

% ---- save identified parameters ----
if strcmp(source,'real')
    ident = struct('Nrev',1426,'K',id.K,'tau',id.tau,'td',id.td,'dz',id.dz);
else
    ident = struct('K',id.K,'tau',id.tau,'td',id.td,'dz',id.dz);
end
ident.pedal = jc.cal;
ident.pedal.Dpl = round(jc.out.REVERSE.Dpl*10)/10;
ident.pedal.Npl = 5;
save(fullfile(root,'data','params_identified.mat'),'ident');
fprintf('[S2-04] saved data/params_identified.mat (source: %s)\n', source);
par = init_params();

R = jc.out.REVERSE;  H = jc.out.HALF;
minXonly = ternary(strcmp(source,'synthetic'), 10, 3);
chk = {
 'A2-1 validation fit >= 90 %',                  id.fitVal >= 90,        sprintf('%.1f %%',id.fitVal)
 'A2-2 K, tau repeatable within +-10 %',         id.repK<=10 && id.repTau<=10, sprintf('K +-%.1f %%, tau +-%.1f %%',id.repK,id.repTau)
 'A2-3 K across amplitudes within +-15 %',       id.linK <= 15,          sprintf('+-%.1f %%',id.linK)
 'A2-4 residual autocorr inside 99 % band',      id.acMax <= id.acBand,  sprintf('max %.2f vs band %.2f',id.acMax,id.acBand)
 'A2-5 peak error <= 10 % of step',              id.peakErr <= 10,       sprintf('%.1f %%',id.peakErr)
 'A2-7a REVERSE: sep >= 2 and 0 false alarms',   R.sep>=2 && R.false_alarms==0 && R.xonly_events>=minXonly, sprintf('sep %.2f, FA %d, det %d',R.sep,R.false_alarms,R.xonly_events)
 'A2-7b HALF   : sep >= 2 and 0 false alarms',   H.sep>=2 && H.false_alarms==0 && H.xonly_events>=minXonly, sprintf('sep %.2f, FA %d, det %d',H.sep,H.false_alarms,H.xonly_events)
 'A2-8 init_params uses identified values',      strcmp(par.src,'identified'), par.src
};
fprintf('\n%-48s %-6s %s\n','Criterion','Result','Value');
ok = true;
for i = 1:size(chk,1)
    fprintf('%-48s %-6s %s\n', chk{i,1}, ternary(chk{i,2},'PASS','FAIL'), chk{i,3});
    ok = ok && chk{i,2};
end
fprintf('\nStage 2 (%s data): %s\n', source, ternary(ok,'ALL PASS','SOME FAIL'));
results = struct('static',st,'ident',id,'joy',jc,'ok',ok);
end

function s = ternary(c,a,b)
if c, s = a; else, s = b; end
end
