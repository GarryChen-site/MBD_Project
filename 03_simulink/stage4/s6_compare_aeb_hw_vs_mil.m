function res = s6_compare_aeb_hw_vs_mil(csvPath)
%S6_COMPARE_AEB_HW_VS_MIL  AEB Hardware-vs-MIL/Reference Verification.
%   Replays the captured VL53L0X ToF distance (d_mm, tof_valid) and motor speed (speed_meas_rpm)
%   from ClosedLoop_RunAEB (data/real/hil_aeb_test.csv) through the Stage 3 golden reference
%   state machine aeb_ref.m and SIL controller, verifies 100% state-transition agreement,
%   emergency brake PWM cut-off (0%), and SG90 servo brake actuation (0/30/90 deg),
%   and saves 06_test/reports/stage6/hw_vs_mil_hil_aeb_test.png.
%
%   Usage:
%     res = s6_compare_aeb_hw_vs_mil();                               % uses data/real/hil_aeb_test.csv (or synthetic dry-run)
%     res = s6_compare_aeb_hw_vs_mil('data/real/hil_aeb_test.csv');
root = setup_paths();
repoRoot = fileparts(root);
par = init_params();
load_params_to_base(par);

if nargin < 1 || isempty(csvPath)
    realCand = fullfile(root, 'data', 'real', 'hil_aeb_test.csv');
    if isfile(realCand)
        csvPath = realCand;
    else
        csvPath = make_dryrun_aeb_csv(root, par);
    end
elseif ~isfile(csvPath)
    csvPath = fullfile(root, csvPath);
end

T = readtable(csvPath);
t = double(T.t_ms(:)) * 1e-3;
N = numel(t);

% 1. Extract the 50 ms sub-rate ticks (k % 5 == 4 -> 1-based index 5:5:N)
idx50 = (5:5:N)';
pAeb = struct('dw', 300, 'db', 150, 'dh', 50, 'Ndeb', 3, 'Nrel', 5, 'Tinv', 4, 'wstop', 5);
[st50_ref, fl50_ref] = aeb_ref(double(T.d_mm(idx50)), logical(T.tof_valid(idx50)), double(T.speed_meas_rpm(idx50)), pAeb);

% Hold 50 ms state across the 10 ms ticks exactly as ClosedLoop_RunAEB does
aeb_state_ref = zeros(N, 1);
tof_inv_ref   = false(N, 1);
for i = 1:numel(idx50)
    k0 = idx50(i);
    k1 = min(N, k0 + 4);
    aeb_state_ref(k0:k1) = st50_ref(i);
    tof_inv_ref(k0:k1)   = fl50_ref.tof_invalid(i);
end

% Expected SG90 servo angle [deg]
servo_ref = zeros(N, 1);
servo_ref(aeb_state_ref == 1) = 30.0;
servo_ref(aeb_state_ref == 2) = 90.0;

% 2. Verify criteria
stateMismatchTicks = sum(double(T.aeb_state(:)) ~= aeb_state_ref);
servoErrMax        = max(abs(double(T.servo_deg(:)) - servo_ref));
brakingIdx         = (double(T.aeb_state(:)) == 2);
if any(brakingIdx)
    maxDutyDuringBrake = max(double(T.duty_cmd_pct(brakingIdx)));
    modeDuringBrakeOk  = all(double(T.mode(brakingIdx)) == 2);
else
    maxDutyDuringBrake = 0;
    modeDuringBrakeOk  = true;
end
maxWcetUs = max(double(T.wcet_us(:)));

c1 = (stateMismatchTicks == 0);
c2 = (maxDutyDuringBrake == 0.0) && modeDuringBrakeOk;
c3 = (servoErrMax <= 0.5);
c4 = (maxWcetUs <= 2000);

% 3. Plot 4-panel AEB Hardware vs MIL Reference overlay
repDir = fullfile(repoRoot, '06_test', 'reports', 'stage6');
if ~exist(repDir, 'dir'), mkdir(repDir); end
[~, baseName, ~] = fileparts(csvPath);
figPath = fullfile(repDir, ['hw_vs_mil_' baseName '.png']);

f = figure('Visible', 'off', 'Position', [100 100 920 820]);
subplot(4,1,1);
plot(t, T.d_mm, 'k-', 'LineWidth', 1.5); hold on;
yline(pAeb.dw, 'y--', 'Warn (300 mm)', 'LineWidth', 1.2);
yline(pAeb.db, 'r--', 'Brake (150 mm)', 'LineWidth', 1.2);
yline(pAeb.dw + pAeb.dh, 'g--', 'Release (350 mm)', 'LineWidth', 1.2);
grid on; ylabel('ToF Dist [mm]');
title(sprintf('Option 3B AEB Hardware-in-the-Loop: VL53L0X + SG90 + Motor (%s)', baseName), 'Interpreter', 'none');

subplot(4,1,2);
stairs(t, aeb_state_ref, 'b-', 'LineWidth', 1.8); hold on;
stairs(t, T.aeb_state, 'r--', 'LineWidth', 1.3);
yticks([0 1 2]); yticklabels({'0: NORMAL', '1: WARNING', '2: BRAKING'});
ylim([-0.3 2.3]); grid on; ylabel('AEB State');
legend('MIL Reference (aeb\_ref.m)', 'STM32 Hardware', 'Location', 'best');

subplot(4,1,3);
yyaxis left;
plot(t, T.speed_req_rpm, 'k--', 'LineWidth', 1.2); hold on;
plot(t, T.speed_meas_rpm, 'b-', 'LineWidth', 1.4);
ylabel('Speed [rpm]');
yyaxis right;
stairs(t, T.servo_deg, 'r-', 'LineWidth', 1.5);
ylabel('SG90 Servo [deg]'); ylim([-10 105]);
grid on;
legend('Speed Req [rpm]', 'HW Speed [rpm]', 'SG90 Brake Angle [deg]', 'Location', 'best');

subplot(4,1,4);
yyaxis left;
plot(t, T.duty_cmd_pct, 'b-', 'LineWidth', 1.4); ylabel('Motor Duty [%]');
yyaxis right;
plot(t, T.wcet_us, 'm-', 'LineWidth', 1.1); ylabel('WCET [\mus]');
xlabel('Time [s]'); grid on;

saveas(f, figPath);
close(f);

fprintf('\n=== Option 3B AEB Hardware-vs-Reference Verification (%s) ===\n', csvPath);
fprintf('%-52s %-6s %s\n', 'Criterion', 'Result', 'Value');
fprintf('%-52s %-6s %d / %d ticks\n', 'A6-AEB-1 State match vs aeb_ref.m (0 mismatch)', tern(c1,'PASS','FAIL'), stateMismatchTicks, N);
fprintf('%-52s %-6s max duty=%.2f %%, SAFE_STOP=%d\n', 'A6-AEB-2 BRAKING cuts PWM to 0 % & SAFE_STOP(2)', tern(c2,'PASS','FAIL'), maxDutyDuringBrake, modeDuringBrakeOk);
fprintf('%-52s %-6s %.2f deg\n', 'A6-AEB-3 SG90 Servo angle match (0/30/90 deg)', tern(c3,'PASS','FAIL'), servoErrMax);
fprintf('%-52s %-6s %d us (budget 10000 us)\n', 'A6-AEB-4 Step WCET <= 2000 us (< 20 % CPU)', tern(c4,'PASS','FAIL'), round(maxWcetUs));
fprintf('[S6-AEB] Saved 4-panel AEB overlay plot to %s\n', figPath);

res = struct('csvPath', csvPath, 'figPath', figPath, ...
             'stateMismatchTicks', stateMismatchTicks, ...
             'maxDutyDuringBrake', maxDutyDuringBrake, ...
             'servoErrMax', servoErrMax, 'maxWcetUs', maxWcetUs, ...
             'allPass', c1 && c2 && c3 && c4);
end

function csvPath = make_dryrun_aeb_csv(root, par)
% Generate a 10-second synthetic AEB hardware log exercising NORMAL -> WARNING -> BRAKING -> NORMAL
Ts = par.Ts;
t_ms = (0:10:10000)';
N = numel(t_ms);
t = double(t_ms) * 1e-3;

% Constant 60% throttle push from t = 0.5 s to 9.5 s
push = (t >= 0.5 & t <= 9.5);
adcX = uint16(push * 3209 + ~push * 2044);
adcY = uint16(push * 3222 + ~push * 2071);

% Obstacle approaches from 600 mm down to 100 mm (t=2..5.5s), then retreats to 500 mm (t=6..8s)
d_mm = uint16(600 * ones(N, 1));
for k = 1:N
    tk = t(k);
    if tk >= 2.0 && tk < 3.5
        d_mm(k) = uint16(250); % WARNING zone (< 300 mm)
    elseif tk >= 3.5 && tk < 6.5
        d_mm(k) = uint16(110); % BRAKING zone (< 150 mm)
    elseif tk >= 6.5
        d_mm(k) = uint16(480); % RELEASE zone (> 350 mm)
    end
end
tof_valid = ones(N, 1, 'uint8');

pAeb = struct('dw', 300, 'db', 150, 'dh', 50, 'Ndeb', 3, 'Nrel', 5, 'Tinv', 4, 'wstop', 5);
idx50 = (5:5:N)';
aeb_state = zeros(N, 1, 'uint8');
servo_deg = zeros(N, 1);
speed_req_rpm  = zeros(N, 1);
speed_meas_rpm = zeros(N, 1);
duty_cmd_pct   = zeros(N, 1);
fault_bitmap   = zeros(N, 1, 'uint16');
mode           = zeros(N, 1, 'uint8');
wcet_us        = uint32(72 + mod((1:N)', 6));

% Simple closed-loop plant simulation with AEB override
s = 0; cW = 0; cB = 0; cR = 0;
w = 0;
a = exp(-Ts / par.tau);
for k = 1:N
    if mod(k-1, 5) == 4
        dk = double(d_mm(k));
        cW = (dk < pAeb.dw) * (cW + 1);
        cB = (dk < pAeb.db) * (cB + 1);
        cR = (dk > pAeb.dw + pAeb.dh) * (cR + 1);
        switch s
            case 0, if cW >= pAeb.Ndeb, s = 1; end
            case 1, if cB >= pAeb.Ndeb, s = 2; elseif cR >= pAeb.Nrel, s = 0; end
            case 2, if cR >= pAeb.Nrel && w < pAeb.wstop, s = 0; end
        end
    end
    aeb_state(k) = uint8(s);
    if s == 2
        speed_req_rpm(k) = 0;
        duty_cmd_pct(k)  = 0;
        mode(k)          = 2;
        servo_deg(k)     = 90.0;
    else
        req = push(k) * 0.6 * par.omega_max;
        speed_req_rpm(k) = round(req, 2);
        duty_cmd_pct(k)  = round(min(par.Dmax, max(0, req / par.K + par.dz)), 2);
        mode(k)          = 0;
        servo_deg(k)     = (s == 1) * 30.0;
    end
    u_eff = max(0, duty_cmd_pct(k) - par.dz);
    w = a * w + (1 - a) * par.K * u_eff;
    speed_meas_rpm(k) = round(w, 2);
end

outDir = fullfile(root, 'data', 'synthetic');
if ~exist(outDir, 'dir'), mkdir(outDir); end
csvPath = fullfile(outDir, 'hil_aeb_test_dryrun.csv');
T = table(t_ms, adcX, adcY, speed_req_rpm, speed_meas_rpm, duty_cmd_pct, ...
          fault_bitmap, mode, wcet_us, d_mm, tof_valid, aeb_state, servo_deg);
writetable(T, csvPath);
end

function s = tern(c,a,b), if c, s=a; else, s=b; end, end
