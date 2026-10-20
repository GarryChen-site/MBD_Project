function r = s2_03_joystick_calibration(dataDir)
%S2_03_JOYSTICK_CALIBRATION  D-004 calibration + choice of Dpl (THR-008, acceptance A2-7).
%   Inputs  (t_ms,adcX,adcY): joy_rest, joy_diag_full, joy_diag_valid, joy_xonly.
%   Outputs r.cal (struct for pedal_cond_ref), r.Dpl, r.sep, r.false_alarms, r.stats.
%   Runs for BOTH modes REVERSE and HALF.
par = init_params();
rest = readtable(fullfile(dataDir,'joy_rest.csv'));
cX = mean(rest.adcX);  cY = mean(rest.adcY);
sX = stage2_lib('sensor_stats', rest.adcX, 'rest X');  sY = stage2_lib('sensor_stats', rest.adcY, 'rest Y');
db = @(s) ceil(5*s.std + 3);                      % dead band = 5 sigma + 3 counts
dbX = db(sX); dbY = db(sY);

full = readtable(fullfile(dataDir,'joy_diag_full.csv'));
maxX = median(peaks(full.adcX, cX));   maxY = median(peaks(full.adcY, cY));   % DIAGONAL maxima

valid = readtable(fullfile(dataDir,'joy_diag_valid.csv'));
xonly = readtable(fullfile(dataDir,'joy_xonly.csv'));

cal = struct('cX',round(cX),'cY',round(cY),'dbX',dbX,'dbY',dbY,'maxX',round(maxX),'maxY',round(maxY), ...
             'alo',par.pedal.alo,'ahi',par.pedal.ahi);
out = struct();
for mode = {'REVERSE','HALF'}
    m = mode{1};  cal.mode = m;
    [s1,s2,p2,e] = pedal_cond_ref(valid.adcX, valid.adcY, cal);
    [sx1,~,~,ex] = pedal_cond_ref(xonly.adcX, xonly.adcY, cal);
    mv = double(s1) > 5;                                % samples where the pedal is moving
    ev = double(e(mv));
    efault = double(ex(double(sx1) > 50));
    [Dpl, sep] = choose_dpl(ev, efault);
    % false alarms with the chosen Dpl and Npl (debounce reference)
    c = 0; act = false; events = 0; prev = false;
    for k = 1:numel(e)
        [c, act] = debounce_step(c, act, double(e(k)) > Dpl, par.pedal.Npl);
        events = events + (act && ~prev);  prev = act;
    end
    % fault detection: X-only push must set the fault
    c = 0; act = false; detected = 0; prev = false;
    for k = 1:numel(ex)
        [c, act] = debounce_step(c, act, double(ex(k)) > Dpl, par.pedal.Npl);
        detected = detected + (act && ~prev);  prev = act;
    end
    % hand-checkable relationships over the valid diagonal pushes
    if strcmp(m,'REVERSE'), rel = max(abs(double(s1(mv)) + double(s2(mv)) - 100));
    else,                   rel = max(abs(double(s2(mv)) - 0.5*double(s1(mv)))); end
    out.(m) = struct('Dpl',Dpl,'sep',sep,'false_alarms',events,'xonly_events',detected, ...
                     'max_relationship_dev',rel,'e_valid_mean',mean(ev),'e_valid_std',std(ev),'e_fault_med',median(efault));
    fprintf('[S2-03 %-7s] Dpl=%.2f %%  sep=%.2f  false alarms=%d/50 pushes  X-only detected=%d/10  max |relation dev|=%.1f %%\n', ...
            m, Dpl, sep, events, detected, rel);
end
cal.mode = 'REVERSE';
r = struct('cal',cal,'out',out,'stats',struct('X',sX,'Y',sY));
fprintf('[S2-03] centre X=%.1f Y=%.1f | sigma X=%.2f Y=%.2f | db=%d/%d | diag max X=%d Y=%d\n', ...
        cX,cY,sX.std,sY.std,dbX,dbY,cal.maxX,cal.maxY);
end

function pk = peaks(x, c)
% peak of each push: contiguous runs of x above c+300 counts
above = x > c + 300;  d = diff([0; above(:); 0]);
st = find(d==1);  en = find(d==-1)-1;  pk = zeros(numel(st),1);
for i = 1:numel(st), pk(i) = max(x(st(i):en(i))); end
end
