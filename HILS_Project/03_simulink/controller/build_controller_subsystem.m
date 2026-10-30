function build_controller_subsystem(sys)
%BUILD_CONTROLLER_SUBSYSTEM  Fill an (empty) subsystem with the Stage-3 Controller (D-004 version).
%
%  In : pedal1_adc, pedal2_adc, speed_meas_rpm
%  Out: duty_cmd_pct, speed_req_rpm, fault_bitmap, mode, s1_pct, p2_pct, e_pct, p_cmd_pct
%
%  adcX,adcY -> single -> PedalCond(MATLAB Fcn, cal=P_cal) -> s1,s2,p2,e,rangeBad
%  e,rangeBad -> FaultMode(MATLAB Fcn, fp=P_fm) -> fault_bitmap, mode, omega_lim
%  min(s1,p2) -> LPF -> RateLimiter -> Gain(omega_max/100) -> min(.,omega_lim) = speed_req_rpm
%  speed_req - speed_meas -> PI with feed-forward and back-calculation anti-windup -> Sat[0,Dmax]
if strcmp(get_param(sys, 'Type'), 'block')
    Simulink.SubSystem.deleteContents(sys);
end
add = @(n,src,varargin) add_block(src,[sys '/' n],varargin{:});
ln  = @(a,b) add_line(sys,a,b,'autorouting','on');
single_ = 'single';
DTC = 'simulink/Signal Attributes/Data Type Conversion';
SUM = 'simulink/Math Operations/Sum';   GAIN = 'simulink/Math Operations/Gain';

% ---------------- ports -------------------------------------------------
add('pedal1_adc','simulink/Sources/In1');
add('pedal2_adc','simulink/Sources/In1');
add('speed_meas_rpm','simulink/Sources/In1');
outs = {'duty_cmd_pct','speed_req_rpm','fault_bitmap','mode','s1_pct','p2_pct','e_pct','p_cmd_pct'};
for i = 1:numel(outs), add(outs{i},'simulink/Sinks/Out1'); end

% ---------------- input conditioning (types) ----------------------------
add('X_single',DTC,'OutDataTypeStr',single_);
add('Y_single',DTC,'OutDataTypeStr',single_);
add('W_single',DTC,'OutDataTypeStr',single_);
add('cal','simulink/Sources/Constant','Value','P_cal','OutDataTypeStr',single_);
add('fp', 'simulink/Sources/Constant','Value','P_fm', 'OutDataTypeStr',single_);
ln('pedal1_adc/1','X_single/1'); ln('pedal2_adc/1','Y_single/1'); ln('speed_meas_rpm/1','W_single/1');

% ---------------- PedalCond (MATLAB Function) ---------------------------
add('PedalCond','simulink/User-Defined Functions/MATLAB Function');
set_mfun([sys '/PedalCond'], [ ...
 'function [s1,s2,p2,e,rangeBad] = PedalCond(adcX, adcY, cal)\n' ...
 '[s1,s2,p2,e,rangeBad] = pedal_cond_core(adcX, adcY, cal);\n']);
ln('X_single/1','PedalCond/1'); ln('Y_single/1','PedalCond/2'); ln('cal/1','PedalCond/3');

% ---------------- FaultMode (MATLAB Function with state) ---------------
add('FaultMode','simulink/User-Defined Functions/MATLAB Function');
set_mfun([sys '/FaultMode'], [ ...
 'function [fault_bitmap, mode, omega_lim] = FaultMode(e, rangeBad, fp)\n' ...
 '%% fp = [Dpl Npl Nrange Nclear omega_max omega_limp]\n' ...
 'persistent cPl aPl cRg aRg cClr modeS\n' ...
 'if isempty(cPl)\n' ...
 '    cPl = single(0); aPl = false; cRg = single(0); aRg = false; cClr = single(0); modeS = uint8(0);\n' ...
 'end\n' ...
 'if e > fp(1), cPl = min(cPl + single(1), fp(2)); else, cPl = max(cPl - single(1), single(0)); end\n' ...
 'if cPl >= fp(2), aPl = true; end\n' ...
 'if cPl <= single(0), aPl = false; end\n' ...
 'if rangeBad, cRg = min(cRg + single(1), fp(3)); else, cRg = max(cRg - single(1), single(0)); end\n' ...
 'if cRg >= fp(3), aRg = true; end\n' ...
 'if cRg <= single(0), aRg = false; end\n' ...
 'if aPl || aRg\n' ...
 '    cClr = single(0); modeS = uint8(1);\n' ...
 'elseif modeS == uint8(1)\n' ...
 '    cClr = cClr + single(1);\n' ...
 '    if cClr >= fp(4), modeS = uint8(0); end\n' ...
 'end\n' ...
 'fault_bitmap = uint16(0);\n' ...
 'if aPl, fault_bitmap = bitor(fault_bitmap, uint16(1)); end\n' ...
 'if aRg, fault_bitmap = bitor(fault_bitmap, uint16(2)); end\n' ...
 'mode = modeS;\n' ...
 'if modeS == uint8(0), omega_lim = fp(5); else, omega_lim = fp(6); end\n']);
ln('PedalCond/4','FaultMode/1'); ln('PedalCond/5','FaultMode/2'); ln('fp/1','FaultMode/3');

% ---------------- pedal request chain ----------------------------------
add('PCmd','simulink/Math Operations/MinMax','Function','min','Inputs','2');
ln('PedalCond/1','PCmd/1'); ln('PedalCond/3','PCmd/2');
add('PedalLPF','simulink/Discrete/Discrete Filter','Numerator','single([1-P_alpha])','Denominator','single([1 -P_alpha])','SampleTime','P_Ts');
add('PedalRateLim','simulink/Discontinuities/Rate Limiter','RisingSlewLimit','single(P_Rup)','FallingSlewLimit','single(-P_Rdn)');
add('ReqScale',GAIN,'Gain','single(P_omega_max/100)','ParamDataTypeStr',single_);
add('ReqLimit','simulink/Math Operations/MinMax','Function','min','Inputs','2');
ln('PCmd/1','PedalLPF/1'); ln('PedalLPF/1','PedalRateLim/1'); ln('PedalRateLim/1','ReqScale/1');
ln('ReqScale/1','ReqLimit/1'); ln('FaultMode/3','ReqLimit/2');

% ---------------- speed PI ---------------------------------------------
add('SpeedErr',SUM,'Inputs','+-');
ln('ReqLimit/1','SpeedErr/1'); ln('W_single/1','SpeedErr/2');
% feed-forward  Dff = dz (when req > 0; linear 1/K is inside IMC PI gains Kp, Ki)
add('FF_gain',GAIN,'Gain','single(0)','ParamDataTypeStr',single_);
add('FF_dz','simulink/Sources/Constant','Value','single(P_dz)','OutDataTypeStr',single_);
add('FF_sum',SUM,'Inputs','++');
add('FF_zero','simulink/Sources/Constant','Value','single(0)','OutDataTypeStr',single_);
add('FF_switch','simulink/Signal Routing/Switch','Criteria','u2 > Threshold','Threshold','single(0)');
ln('ReqLimit/1','FF_gain/1'); ln('FF_gain/1','FF_sum/1'); ln('FF_dz/1','FF_sum/2');
ln('FF_sum/1','FF_switch/1'); ln('ReqLimit/1','FF_switch/2'); ln('FF_zero/1','FF_switch/3');
% P and I terms
add('Kp_gain',GAIN,'Gain','single(P_Kp)','ParamDataTypeStr',single_);
add('KiTs_gain',GAIN,'Gain','single(P_Ki*P_Ts)','ParamDataTypeStr',single_);
ln('SpeedErr/1','Kp_gain/1'); ln('SpeedErr/1','KiTs_gain/1');
add('I_prev','simulink/Discrete/Unit Delay','SampleTime','P_Ts','InitialCondition','single(0)');
add('U_raw',SUM,'Inputs','++++');
ln('FF_switch/1','U_raw/1'); ln('Kp_gain/1','U_raw/2'); ln('I_prev/1','U_raw/3'); ln('KiTs_gain/1','U_raw/4');
add('Sat','simulink/Discontinuities/Saturation','UpperLimit','single(P_Dmax)','LowerLimit','single(0)');
ln('U_raw/1','Sat/1');
% back-calculation:  I[k] = I[k-1] + Ts*( Ki*e + Kb*(u - u_raw) )
add('SatDiff',SUM,'Inputs','+-');
add('Kb_gain',GAIN,'Gain','single(P_Kb)','ParamDataTypeStr',single_);
add('Ki_gain',GAIN,'Gain','single(P_Ki)','ParamDataTypeStr',single_);
add('I_rate',SUM,'Inputs','++');
add('Ts_gain',GAIN,'Gain','single(P_Ts)','ParamDataTypeStr',single_);
add('I_acc',SUM,'Inputs','++');
ln('Sat/1','SatDiff/1'); ln('U_raw/1','SatDiff/2'); ln('SatDiff/1','Kb_gain/1');
ln('SpeedErr/1','Ki_gain/1'); ln('Ki_gain/1','I_rate/1'); ln('Kb_gain/1','I_rate/2');
ln('I_rate/1','Ts_gain/1'); ln('I_prev/1','I_acc/1'); ln('Ts_gain/1','I_acc/2');
ln('I_acc/1','I_prev/1');

% ---------------- outputs ----------------------------------------------
ln('Sat/1','duty_cmd_pct/1');       ln('ReqLimit/1','speed_req_rpm/1');
ln('FaultMode/1','fault_bitmap/1'); ln('FaultMode/2','mode/1');
ln('PedalCond/1','s1_pct/1');       ln('PedalCond/3','p2_pct/1');
ln('PedalCond/4','e_pct/1');        ln('PCmd/1','p_cmd_pct/1');
Simulink.BlockDiagram.arrangeSystem(sys);
end

function set_mfun(blk, script)
% Put source code into a MATLAB Function block. '\n' sequences become newlines, '%%' -> '%'.
code = sprintf(script);
rt = sfroot;
ch = rt.find('-isa','Stateflow.EMChart','Path',blk);
assert(~isempty(ch), 'MATLAB Function block %s not found', blk);
ch.Script = code;
end
