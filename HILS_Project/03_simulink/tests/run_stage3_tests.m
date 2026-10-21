function results = run_stage3_tests(dataDir)
%RUN_STAGE3_TESTS  Automated MIL test suite for Stage 3 (TC-THR/SPD/DIA/SAF/AEB), pass/fail table.
%   Prerequisites: run_stage2('synthetic') (or 'real') so data/params_identified.mat exists,
%   then s3_01_build_speed_ctrl (built automatically on first use).
%   dataDir: folder with joy_diag_valid.csv for the false-alarm replay (default data/synthetic).
root = setup_paths();
if nargin < 1, dataDir = fullfile(root,'data','synthetic'); end
par = init_params();  Ts = par.Ts;
if ~strcmp(par.src,'identified')
    warning('init_params() is using DEFAULT values; run run_stage2 first for meaningful results.');
end
fast = struct('P_alpha', exp(-Ts/0.002), 'P_Rup', 1e5, 'P_Rdn', 1e5);   % test config: no pedal shaping
cal  = par.pedal;  Npl = par.pedal.Npl;  Dpl = par.pedal.Dpl;
R = {};                                                                    % {id, pass, detail}
    function rec(id, ok, msg), R(end+1,:) = {id, logical(ok), msg}; end %#ok<NASGU>
    function run1(id, fn)
        try, [ok,msg] = fn(); rec(id, ok, msg);
        catch ME, rec(id, false, ['ERROR: ' strrep(ME.message, newline, ' ')]); end
    end
ramp = @(t0,dur,pk) @(t) pk*min(max((t-t0)/dur,0),1);

% ---------------------------------------------------------------- TC-THR-001 (pure MATLAB)
run1('TC-THR-001', @() deal_(thr001(par), ''));
    function [ok,msg] = deal_(a,b), ok = a.ok; msg = a.msg; end

% ---------------------------------------------------------------- A3-9  Simulink == reference
run1('A3-9 PedalCond Simulink = ref', @() a39());
    function [ok,msg] = a39()
        [X,Y] = scn_pedal('diag', ramp(0.5,2,1), 3.5, par);
        r = run_mil(X, Y, 3.5);
        n = min(numel(r.t), numel(X.Data));
        [s1,~,p2,e] = pedal_cond_ref(X.Data(1:n), Y.Data(1:n), par.pedal);
        d = max([max(abs(r.s1_pct(1:n)-double(s1))), max(abs(r.p2_pct(1:n)-double(p2))), max(abs(r.e_pct(1:n)-double(e)))]);
        ok = d < 1e-3;  msg = sprintf('max |diff| = %.2e', d);
    end

% ---------------------------------------------------------------- TC-THR-002 / 003
run1('TC-THR-002 X-only fault', @() thr002());
    function [ok,msg] = thr002()
        [X,Y] = scn_pedal('xonly', ramp(1,0.5,0.6), 3, par);
        r = run_mil(X, Y, 3);
        k1 = find(r.e_pct > Dpl, 1);  k2 = find(bitand(r.fault_bitmap,1) > 0, 1);
        nticks = k2 - k1 + 1;
        ok = abs(r.e_pct(end)-60) <= 1 && abs(nticks - Npl) <= 1 && r.mode(end) == 1;
        msg = sprintf('e_end=%.1f, fault after %d ticks (Npl=%d), mode=%d', r.e_pct(end), nticks, Npl, r.mode(end));
    end
run1('TC-THR-003a lower-select X-only', @() thr003a());
    function [ok,msg] = thr003a()
        [X,Y] = scn_pedal('xonly', ramp(1,0.5,0.6), 3, par);
        r = run_mil(X, Y, 3);
        ok = max(r.p_cmd_pct) == 0 && max(r.duty_cmd_pct) < 0.01;
        msg = sprintf('max p_cmd=%.2f  max duty=%.3f', max(r.p_cmd_pct), max(r.duty_cmd_pct));
    end
run1('TC-THR-003b s1=90,p2=60 -> limp', @() thr003b());
    function [ok,msg] = thr003b()
        X = scn_pedal('diag', @(t) 0.9*(t>=1), 5, par);
        [~,Y] = scn_pedal('diag', @(t) 0.6*(t>=1), 5, par);
        r = run_mil(X, Y, 5);
        k2 = find(bitand(r.fault_bitmap,1) > 0, 1);
        lim = par.omega_limp + 0.01;
        ok = ~isempty(k2) && max(r.speed_req_rpm(k2+2:end)) <= lim && abs(r.p_cmd_pct(end)-60) < 1 && r.mode(end)==1;
        msg = sprintf('p_cmd_end=%.1f  max req after fault=%.2f (limp %.0f)', r.p_cmd_pct(end), max(r.speed_req_rpm(k2+2:end)), par.omega_limp);
    end

% ---------------------------------------------------------------- TC-THR-004 range (raw)
run1('TC-THR-004 raw range fault', @() thr004());
    function [ok,msg] = thr004()
        [X,Y] = scn_pedal('ygnd', @(t) 0.5*(t>=0.5), 1.5, par);
        r1 = run_mil(X, Y, 1.5);
        [X2,Y2] = scn_pedal('diag', @(t) 0.5*(t>=0.5), 1.5, par);
        X2.Data(:) = 4095;
        r2 = run_mil(X2, Y2, 1.5);
        k = find(bitand(r1.fault_bitmap,2) > 0, 1);
        ok = bitand(r1.fault_bitmap(end),2)==2 && bitand(r2.fault_bitmap(end),2)==2 && ...
             max(r1.speed_req_rpm(k+2:end)) <= par.omega_limp + 0.01;
        msg = sprintf('Y=0 -> bit1=%d, X=4095 -> bit1=%d', bitand(r1.fault_bitmap(end),2)>0, bitand(r2.fault_bitmap(end),2)>0);
    end

% ---------------------------------------------------------------- TC-THR-005 rate limit
run1('TC-THR-005 rate limiter', @() thr005());
    function [ok,msg] = thr005()
        [X,Y] = scn_pedal('diag', @(t) 1.0*(t>=1), 3, par);
        r = run_mil(X, Y, 3);
        slope = max(diff(r.speed_req_rpm))/Ts;
        lim = par.Rup/100*par.omega_max;
        ok = slope <= lim*1.001;  msg = sprintf('max slope %.1f rpm/s (limit %.1f)', slope, lim);
    end

% ---------------------------------------------------------------- TC-THR-006 no false alarms (replay)
run1('TC-THR-006 replay 50 pushes', @() thr006());
    function [ok,msg] = thr006()
        f = fullfile(dataDir,'joy_diag_valid.csv');
        if ~isfile(f), ok = false; msg = ['missing ' f]; return, end
        T = readtable(f);  t = T.t_ms/1000;
        r = run_mil(timeseries(double(T.adcX),t), timeseries(double(T.adcY),t), t(end));
        b = bitand(r.fault_bitmap,1) > 0;  events = sum(diff([0; b]) == 1);
        ok = events == 0;  msg = sprintf('%d F_PEDAL_PLAUS events in %.0f s of replay', events, t(end));
    end

% ---------------------------------------------------------------- TC-THR-007 both modes
run1('TC-THR-007 REVERSE & HALF', @() thr007());
    function [ok,msg] = thr007()
        res = zeros(1,2);  modes = {'REVERSE','HALF'};
        for i = 1:2
            c = par.pedal;  c.mode = modes{i};
            [X,Y] = scn_pedal('diag', ramp(0.5,2,0.9), 3.5, par);
            r = run_mil(X, Y, 3.5, struct('P_cal', cal_to_vec(c)));
            res(i) = max(r.fault_bitmap) == 0 && abs(r.s1_pct(end)-90) < 1 && abs(r.p2_pct(end)-90) < 1;
        end
        ok = all(res);  msg = sprintf('REVERSE ok=%d, HALF ok=%d (no fault, s1=p2=90)', res(1), res(2));
    end

% ---------------------------------------------------------------- TC-THR-008 Y stuck high
run1('TC-THR-008 Y stuck at max', @() thr008());
    function [ok,msg] = thr008()
        [X,Y] = scn_pedal('yonly', @(t) 1.0*(t>=1), 3, par);
        r = run_mil(X, Y, 3);
        ok = abs(r.p2_pct(end)-100) < 0.5 && r.s1_pct(end) == 0 && abs(r.e_pct(end)-100) < 0.5 && ...
             max(r.p_cmd_pct) == 0 && bitand(r.fault_bitmap(end),1) == 1;
        msg = sprintf('p2=%.1f s1=%.1f e=%.1f p_cmd=%.1f', r.p2_pct(end), r.s1_pct(end), r.e_pct(end), max(r.p_cmd_pct));
    end

% ---------------------------------------------------------------- SPD tests
lam = []; mNom = [];
run1('TC-SPD-001 step response', @() spd001());
    function [ok,msg] = spd001()
        load_params_to_base(par);  lam = evalin('base','P_lambda');
        [X,Y] = scn_pedal('diag', @(t) 0.6*(t>=1), 3, par);
        r = run_mil(X, Y, 3, fast);
        i0 = find(r.t >= 1, 1);
        yref = 0.6*par.omega_max;
        mNom = step_metrics(r.t(i0:end)-1, r.speed_meas_rpm(i0:end), yref, 0.02);
        trExp = 2.2*(lam - loop_delay(Ts, par.Mavg));
        ok = abs(mNom.rise - trExp)/trExp <= 0.20 && mNom.overshoot <= 5 && ...
             mNom.settle <= 5*lam + 0.05 && abs(mNom.sse) <= max(1, 0.01*yref);
        msg = sprintf('tr=%.3f (exp %.3f)  os=%.1f %%  ts=%.3f  sse=%.2f rpm', mNom.rise, trExp, mNom.overshoot, mNom.settle, mNom.sse);
    end
run1('TC-SPD-002 anti-windup', @() spd002());
    function [ok,msg] = spd002()
        [X,Y] = scn_pedal('diag', @(t) (t>=1).*(1-0.5*(t>=5)), 8, par);
        o = fast;  o.PL_K = 0.6*par.K;                       % plant weaker than controller assumes
        r = run_mil(X, Y, 8, o);
        ref = 0.5*par.omega_max;  i5 = find(r.t >= 5, 1);
        irec = find(abs(r.speed_meas_rpm(i5:end)-ref) <= 0.05*ref, 1) + i5 - 1;
        trec = r.t(irec) - 5;
        under = min(r.speed_meas_rpm(irec:end)) / ref;
        ok = trec <= 1.0 && under >= 0.9;
        msg = sprintf('recovery %.2f s after request drop (<=1.0), min/ref after = %.2f', trec, under);
    end
run1('TC-SPD-003 duty cap', @() spd003());
    function [ok,msg] = spd003()
        [X,Y] = scn_pedal('diag', @(t) 1.0*(t>=1), 4, par);
        r = run_mil(X, Y, 4, fast);
        ok = max(r.duty_cmd_pct) <= par.Dmax + 1e-4 && min(r.duty_cmd_pct) >= 0;
        msg = sprintf('max duty %.3f %% (Dmax %.1f)', max(r.duty_cmd_pct), par.Dmax);
    end
run1('TC-SPD-004 plant corners', @() spd004());
    function [ok,msg] = spd004()
        if isempty(mNom), error('TC-SPD-001 must run first'); end
        worstOS = 0; worstTs = 0;
        for kf = [0.9 1.1]
            for tf = [0.8 1.2]
                [X,Y] = scn_pedal('diag', @(t) 0.6*(t>=1), 4, par);
                o = fast; o.PL_K = kf*par.K; o.PL_tau = tf*par.tau;
                r = run_mil(X, Y, 4, o);
                i0 = find(r.t >= 1, 1);
                m = step_metrics(r.t(i0:end)-1, r.speed_meas_rpm(i0:end), 0.6*par.omega_max, 0.02);
                worstOS = max(worstOS, m.overshoot);  worstTs = max(worstTs, m.settle);
            end
        end
        ok = worstOS <= 15 && worstTs <= 2*mNom.settle + 0.05;
        msg = sprintf('worst overshoot %.1f %% (<=15), worst ts %.3f s (<= 2x%.3f)', worstOS, worstTs, mNom.settle);
    end

% ---------------------------------------------------------------- DIA / SAF
run1('TC-DIA-001 debounce', @() dia001());
    function [ok,msg] = dia001()
        T = 2;  [X,Y] = scn_pedal('xonly', @(t) 0*t, T, par);
        hi = round(par.pedal.cX + par.pedal.dbX + 0.6*(par.pedal.maxX - par.pedal.cX - par.pedal.dbX));
        Xg = X;  Xg.Data(101) = hi;                                  % 1-tick glitch
        r1 = run_mil(Xg, Y, T);
        Xs = X;  Xs.Data(101:110) = hi;                              % 10 ticks sustained
        r2 = run_mil(Xs, Y, T);
        k = find(bitand(r2.fault_bitmap,1) > 0, 1);
        ok = max(r1.fault_bitmap) == 0 && ~isempty(k) && (k - 101 + 1) == Npl;
        msg = sprintf('glitch fault=%d; sustained set after %d ticks (Npl=%d)', max(r1.fault_bitmap)>0, k-101+1, Npl);
    end
run1('TC-SAF-001 mode transitions', @() saf001());
    function [ok,msg] = saf001()
        [X,Y] = scn_pedal('xonly', @(t) 0.6*(t>=1 & t<2), 6, par);
        r = run_mil(X, Y, 6);
        kD = find(r.mode == 1, 1);  kN = find(r.mode(kD:end) == 0, 1) + kD - 1;
        ok = r.mode(1) == 0 && ~isempty(kD) && ~isempty(kN) && r.mode(end) == 0;
        msg = sprintf('NORMAL->DEGRADED at %.2f s, back to NORMAL at %.2f s', r.t(kD), r.t(kN));
    end

% ---------------------------------------------------------------- AEB (reference model, MATLAB only)
p = struct('dw',300,'db',150,'dh',50,'Ndeb',3,'Nrel',5,'Tinv',4,'wstop',5);
run1('TC-AEB-001 warn/brake points', @() aeb001());
    function [ok,msg] = aeb001()
        d = (600:-10:100)';  st = aeb_ref(d, true(size(d)), zeros(size(d)), p);
        kw = find(st >= 1, 1);  kb = find(st == 2, 1);
        ok = d(kw) <= p.dw && d(kw) >= p.dw-(p.Ndeb+1)*10 && d(kb) <= p.db && d(kb) >= p.db-(p.Ndeb+1)*10;
        msg = sprintf('WARNING at d=%d (dw=%d), BRAKING at d=%d (db=%d)', d(kw), p.dw, d(kb), p.db);
    end
run1('TC-AEB-002 release, no chatter', @() aeb002());
    function [ok,msg] = aeb002()
        d = [(600:-10:100)'; (110:10:400)'];
        st = aeb_ref(d, true(size(d)), zeros(size(d)), p);
        rng(1);  dn = d + 8*randn(size(d));  stn = aeb_ref(dn, true(size(d)), zeros(size(d)), p);
        n1 = sum(diff(st) ~= 0);  n2 = sum(diff(stn) ~= 0);
        ok = st(end) == 0 && n1 == 3 && n2 == 3;
        msg = sprintf('transitions clean=%d noisy(8mm)=%d (expect 3), final state=%d', n1, n2, st(end));
    end
run1('TC-AEB-003 invalid data', @() aeb003());
    function [ok,msg] = aeb003()
        d = (600:-10:100)';  v = true(size(d));  v(31:end) = false;
        [st, fl] = aeb_ref(d, v, zeros(size(d)), p);
        ok = max(st) < 2 && any(fl.tof_invalid);
        msg = sprintf('max state=%d (must be <2), F_TOF_INVALID raised=%d', max(st), any(fl.tof_invalid));
    end

% ---------------------------------------------------------------- report
fprintf('\n%-34s %-6s %s\n', 'Test', 'Result', 'Detail');
for i = 1:size(R,1)
    fprintf('%-34s %-6s %s\n', R{i,1}, tern(R{i,2},'PASS','FAIL'), R{i,3});
end
allok = all([R{:,2}]);
fprintf('\nStage 3 MIL suite: %d/%d passed -> %s\n', sum([R{:,2}]), size(R,1), tern(allok,'ALL PASS','SOME FAIL'));
results = cell2struct(R, {'id','pass','detail'}, 2);
end

function a = thr001(par)
% Pure MATLAB hand-check of PedalCond relationships (no Simulink needed).
[X,Y] = scn_pedal('diag', @(t) t, 1, par);          % n = t, 0..1
u = X.Time;  ok = true;  msg = '';
for m = {'REVERSE','HALF'}
    c = par.pedal;  c.mode = m{1};
    [s1,s2] = pedal_cond_ref(X.Data, Y.Data, c);
    s1 = double(s1);  s2 = double(s2);
    e1 = max(abs(s1 - 100*u));
    if strcmp(m{1},'REVERSE'), e2 = max(abs(s1 + s2 - 100)); else, e2 = max(abs(s2 - 0.5*s1)); end
    ok = ok && e1 <= 0.5 && e2 <= 0.5;
    msg = sprintf('%s %s: s1 err %.2f, relation err %.2f;', msg, m{1}, e1, e2);
end
z = pedal_cond_ref(par.pedal.cX+par.pedal.dbX, par.pedal.cY+par.pedal.dbY, par.pedal);
ok = ok && z == 0;  msg = [msg ' dead-band edge -> 0'];
a = struct('ok', ok, 'msg', msg);
end

function s = tern(c,a,b), if c, s=a; else, s=b; end, end
