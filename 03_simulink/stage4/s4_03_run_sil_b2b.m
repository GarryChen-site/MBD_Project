function results = s4_03_run_sil_b2b()
%S4_03_RUN_SIL_B2B  Compile the generated C code (ctrl_codegen.c) into a native MEX SIL binary,
%   run open-loop Back-to-Back (MIL vs SIL) tests across 5 scenarios (including a 60 s random vector),
%   verify A4-1..A4-7, and write 06_test/reports/stage4/b2b_report.md.
root = setup_paths();
repoRoot = fileparts(root);
par = init_params();
load_params_to_base(par);

appGenDir = fullfile(repoRoot, '04_firmware', 'app_generated');
if ~isfile(fullfile(appGenDir, 'ctrl_codegen.c'))
    s4_02_generate_code();
end

% 1. Build native C MEX harness around ctrl_codegen.c
mexName = build_sil_mex(root, appGenDir);

% 2. Build 5 comprehensive open-loop input scenarios [pedal1_adc, pedal2_adc, speed_meas_rpm]
scns = build_scenarios(par);

% 3. Run MIL (ctrl_codegen.slx) and SIL (compiled ctrl_codegen.c) back-to-back on each scenario
mdl = 'ctrl_codegen';
if ~bdIsLoaded(mdl), load_system(fullfile(root, 'controller', [mdl '.slx'])); end

b2b = struct();
allPass = true;
fprintf('\n%-34s %-6s %-12s %-12s %s\n', 'Scenario (MIL vs SIL)', 'Result', 'Max |float|', 'Max |int|', 'Ticks');
for k = 1:numel(scns)
    s = scns(k);
    yMil = run_ctrl_mil(mdl, s.t, s.x, s.y, s.w, par);
    ySil = run_ctrl_sil(mexName, s.x, s.y, s.w);
    rep  = back_to_back_compare(yMil, ySil, 1e-4, 1e-4);
    intDiff = max(max(abs(double(yMil.fault_bitmap) - double(ySil.fault_bitmap))), ...
                  max(abs(double(yMil.mode) - double(ySil.mode))));
    fltDiff = rep.max_abs;
    fprintf('%-34s %-6s %-12.2e %-12d %d\n', s.name, tern(rep.pass,'PASS','FAIL'), fltDiff, intDiff, numel(s.t));
    b2b(k).name = s.name;
    b2b(k).ticks = numel(s.t);
    b2b(k).report = rep;
    b2b(k).intDiff = intDiff;
    b2b(k).fltDiff = fltDiff;
    allPass = allPass && rep.pass;
end

% 4. Write Stage 4 Back-to-Back Markdown Report
repDir = fullfile(repoRoot, '06_test', 'reports', 'stage4');
if ~exist(repDir, 'dir'), mkdir(repDir); end
repPath = fullfile(repDir, 'b2b_report.md');
write_b2b_report(repPath, b2b, allPass);
fprintf('[S4-03] wrote back-to-back equivalence report to %s\n', repPath);

results = struct('scenarios', b2b, 'allPass', allPass, 'reportPath', repPath);
end

function mexName = build_sil_mex(root, appGenDir)
% Generate and compile a lightweight C MEX wrapper calling App_Init / App_Step10ms
% (which wraps ctrl_codegen_initialize / ctrl_codegen_step without rtwtypes/tmwtypes header collisions).
workDir = fullfile(root, 'codegen_work');
if ~exist(workDir, 'dir'), mkdir(workDir); end
fwDir = fileparts(appGenDir);
mexSrc = fullfile(workDir, 'ctrl_sil_mex.c');

cCode = [ ...
'#include "mex.h"', newline, ...
'#include "app_wrapper.h"', newline, newline, ...
'void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {', newline, ...
'    if (nrhs < 3) mexErrMsgTxt("Usage: [duty,req,fb,mode,s1,p2,e,pcmd] = ctrl_sil_mex(x, y, w)");', newline, ...
'    size_t N = mxGetNumberOfElements(prhs[0]);', newline, ...
'    const double *x = mxGetPr(prhs[0]);', newline, ...
'    const double *y = mxGetPr(prhs[1]);', newline, ...
'    const double *w = mxGetPr(prhs[2]);', newline, ...
'    for (int i = 0; i < 8; i++) plhs[i] = mxCreateDoubleMatrix(N, 1, mxREAL);', newline, ...
'    double *duty = mxGetPr(plhs[0]);', newline, ...
'    double *req  = mxGetPr(plhs[1]);', newline, ...
'    double *fb   = mxGetPr(plhs[2]);', newline, ...
'    double *mode = mxGetPr(plhs[3]);', newline, ...
'    double *s1   = mxGetPr(plhs[4]);', newline, ...
'    double *p2   = mxGetPr(plhs[5]);', newline, ...
'    double *e    = mxGetPr(plhs[6]);', newline, ...
'    double *pcmd = mxGetPr(plhs[7]);', newline, ...
'    App_Init();', newline, ...
'    for (size_t k = 0; k < N; k++) {', newline, ...
'        Rte_AppIn_t  in  = { (float)x[k], (float)y[k], (float)w[k] };', newline, ...
'        Rte_AppOut_t out = { 0 };', newline, ...
'        App_Step10ms(&in, &out);', newline, ...
'        duty[k] = (double)out.duty_cmd_pct;', newline, ...
'        req[k]  = (double)out.speed_req_rpm;', newline, ...
'        fb[k]   = (double)out.fault_bitmap;', newline, ...
'        mode[k] = (double)out.mode;', newline, ...
'        s1[k]   = (double)out.s1_pct;', newline, ...
'        p2[k]   = (double)out.p2_pct;', newline, ...
'        e[k]    = (double)out.e_pct;', newline, ...
'        pcmd[k] = (double)out.p_cmd_pct;', newline, ...
'    }', newline, ...
'}', newline];

fid = fopen(mexSrc, 'w'); fwrite(fid, cCode); fclose(fid);

% Collect all generated .c files except main.c / ert_main.c, plus 04_firmware/app_wrapper.c
cList = dir(fullfile(appGenDir, '*.c'));
srcFiles = {mexSrc, fullfile(fwDir, 'app_wrapper.c')};
for i = 1:numel(cList)
    if ~contains(cList(i).name, 'main')
        srcFiles{end+1} = fullfile(appGenDir, cList(i).name); %#ok<AGROW>
    end
end

clear ctrl_sil_mex;
try
    mex('-silent', ['-I' fwDir], ['-I' appGenDir], '-outdir', workDir, srcFiles{:});
catch
    % Fallback on macOS when CommandLineTools clang is available but Xcode GUI license is unaccepted
    clangBin = '/Library/Developer/CommandLineTools/usr/bin/clang';
    sdkPath  = '/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk';
    outFile  = fullfile(workDir, ['ctrl_sil_mex.' mexext]);
    incMatlab = fullfile(matlabroot, 'extern', 'include');
    libMatlab = fullfile(matlabroot, 'bin', computer('arch'));
    srcJoined = strjoin(cellfun(@(p) ['"' p '"'], srcFiles, 'UniformOutput', false), ' ');
    cmd = sprintf('"%s" -isysroot "%s" -O2 -dynamiclib -Wl,-exported_symbol,_mexFunction -I"%s" -I"%s" -I"%s" %s -L"%s" -lmx -lmex -o "%s"', ...
                  clangBin, sdkPath, incMatlab, fwDir, appGenDir, srcJoined, libMatlab, outFile);
    [st, msg] = system(cmd);
    assert(st == 0, 'SIL MEX compilation failed:\n%s', msg);
end
addpath(workDir);
mexName = 'ctrl_sil_mex';
end

function y = run_ctrl_mil(mdl, t, x, y_adc, w, par) %#ok<INUSD>
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(t(end)));
tIn = t(:);
if numel(tIn) > 1
    tIn = [tIn(1); tIn(2:end) - 1e-7; tIn(end) + 1.0];
    xIn = [single(x(:));     single(x(end))];
    yIn = [single(y_adc(:)); single(y_adc(end))];
    wIn = [single(w(:));     single(w(end))];
else
    xIn = single(x(:)); yIn = single(y_adc(:)); wIn = single(w(:));
end
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries(xIn, tIn), 'pedal1_adc');
ds = ds.addElement(timeseries(yIn, tIn), 'pedal2_adc');
ds = ds.addElement(timeseries(wIn, tIn), 'speed_meas_rpm');
in = in.setExternalInput(ds);
out = sim(in);
names = {'duty_cmd_pct','speed_req_rpm','fault_bitmap','mode','s1_pct','p2_pct','e_pct','p_cmd_pct'};
y = struct('t', t(:));
for i = 1:numel(names)
    el = out.yout{i};
    y.(names{i}) = double(el.Values.Data(:));
end
end

function y = run_ctrl_sil(mexName, x, y_adc, w)
[duty, req, fb, mode, s1, p2, e, pcmd] = feval(mexName, double(x(:)), double(y_adc(:)), double(w(:)));
y = struct('duty_cmd_pct',duty, 'speed_req_rpm',req, 'fault_bitmap',fb, 'mode',mode, ...
           's1_pct',s1, 'p2_pct',p2, 'e_pct',e, 'p_cmd_pct',pcmd);
end

function scns = build_scenarios(par)
Ts = par.Ts;
ramp = @(t0,dur,pk) @(t) pk*min(max((t-t0)/dur,0),1);

% Scenario 1: Normal diagonal ramp + speed tracking
[X1, Y1] = scn_pedal('diag', ramp(0.5, 2.0, 0.8), 4.0, par);
r1 = run_mil(X1, Y1, 4.0);
scns(1) = struct('name','S1_Normal_Diag_Ramp', 't',r1.t, 'x',X1.Data, 'y',Y1.Data, 'w',[0; r1.speed_meas_rpm(1:end-1)]);

% Scenario 2: X-only plausibility fault + recovery to NORMAL
[X2, Y2] = scn_pedal('xonly', @(t) 0.6*(t>=0.5 & t<2.0), 4.5, par);
r2 = run_mil(X2, Y2, 4.5);
scns(2) = struct('name','S2_XOnly_Fault_Recovery', 't',r2.t, 'x',X2.Data, 'y',Y2.Data, 'w',[0; r2.speed_meas_rpm(1:end-1)]);

% Scenario 3: Raw ADC wire-break range fault (Y shorted to GND)
[X3, Y3] = scn_pedal('ygnd', @(t) 0.5*(t>=0.5), 2.0, par);
r3 = run_mil(X3, Y3, 2.0);
scns(3) = struct('name','S3_Raw_Range_Fault', 't',r3.t, 'x',X3.Data, 'y',Y3.Data, 'w',[0; r3.speed_meas_rpm(1:end-1)]);

% Scenario 4: Full-throttle saturation & anti-windup step drop
[X4, Y4] = scn_pedal('diag', @(t) (t>=0.5).*(1-0.5*(t>=3.0)), 5.0, par);
r4 = run_mil(X4, Y4, 5.0, struct('PL_K', 0.6*par.K));
scns(4) = struct('name','S4_AntiWindup_Saturation', 't',r4.t, 'x',X4.Data, 'y',Y4.Data, 'w',[0; r4.speed_meas_rpm(1:end-1)]);

% Scenario 5: 60-second (6001 ticks) random stress scenario (Stage 4 section 3 requirement)
rng(42);
t5 = (0:Ts:60)';
N5 = numel(t5);
uRand = filter(ones(20,1)/20, 1, rand(N5,1));
[X5, Y5] = scn_pedal('diag', @(t) interp1(t5, uRand, t, 'linear', 0), 60.0, par);
x5 = X5.Data + round(5*randn(N5,1));
y5 = Y5.Data + round(5*randn(N5,1));
w5 = max(0, par.omega_max * filter(0.08, [1 -0.92], uRand) + 1.5*randn(N5,1));
scns(5) = struct('name','S5_Random_60s_Stress', 't',t5, 'x',x5, 'y',y5, 'w',w5);
end

function write_b2b_report(repPath, b2b, allPass)
fid = fopen(repPath, 'w');
fprintf(fid, '# Stage 4 — Back-to-Back (MIL vs SIL) Equivalence Report\n\n');
fprintf(fid, '- **Model**: `03_simulink/controller/ctrl_codegen.slx`\n');
fprintf(fid, '- **Generated C Source**: `04_firmware/app_generated/ctrl_codegen.c`\n');
fprintf(fid, '- **Tolerance**: exact (`0`) on `fault_bitmap` / `mode`; `<= 1e-4` on `single` float signals\n');
fprintf(fid, '- **Overall Status**: **%s**\n\n', tern(allPass, 'ALL PASS', 'FAIL'));
fprintf(fid, '## Summary by Scenario\n\n');
fprintf(fid, '| Scenario | Ticks | Max Float Diff | Max Int/Enum Diff | Result |\n');
fprintf(fid, '| :--- | :---: | :---: | :---: | :---: |\n');
for k = 1:numel(b2b)
    fprintf(fid, '| `%s` | %d | `%.2e` | `%d` | **%s** |\n', ...
            b2b(k).name, b2b(k).ticks, b2b(k).fltDiff, b2b(k).intDiff, tern(b2b(k).report.pass,'PASS','FAIL'));
end
fclose(fid);
end

function s = tern(c,a,b), if c, s=a; else, s=b; end, end
