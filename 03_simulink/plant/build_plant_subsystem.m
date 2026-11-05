function build_plant_subsystem(sys)
%BUILD_PLANT_SUBSYSTEM  Fill an (empty) subsystem with the motor + encoder plant.
%   In : duty_pct [%]                 Out: speed_meas_rpm [rpm] (encoder-estimated, 10 ms)
%
%   duty -> ToDouble -> DeadZone(+-PL_dz) -> TF PL_K/(PL_tau s+1) -> TransportDelay(PL_td)
%        -> Gain(PL_Nrev/60) -> Integrator (counts) -> ZOH(Ts) -> floor -> [-UnitDelay] -> dN
%        -> Gain(60/(PL_Nrev*Ts)) -> speed_meas_rpm
%   The last part is the SAME estimator as the firmware (rpm = 60*dN/(Nrev*Tw)).
Simulink.SubSystem.deleteContents(sys);
add = @(n,src,varargin) add_block(src,[sys '/' n],varargin{:});
ln  = @(a,b) add_line(sys,a,b,'autorouting','on');

add('duty_pct',        'simulink/Sources/In1');
add('ToDouble',        'simulink/Signal Attributes/Data Type Conversion','OutDataTypeStr','double');
add('DeadZone',        'simulink/Discontinuities/Dead Zone','LowerValue','-PL_dz','UpperValue','PL_dz');
add('MotorTF',         'simulink/Continuous/Transfer Fcn','Numerator','[PL_K]','Denominator','[PL_tau 1]');
add('Delay',           'simulink/Continuous/Transport Delay','DelayTime','PL_td');
add('ToCountsPerSec',  'simulink/Math Operations/Gain','Gain','PL_Nrev/60');
add('CountsInt',       'simulink/Continuous/Integrator');
add('SampleCounts',    'simulink/Discrete/Zero-Order Hold','SampleTime','P_Ts');
add('FloorCounts',     'simulink/Math Operations/Rounding Function','Operator','floor');
add('PrevCounts',      'simulink/Discrete/Unit Delay','SampleTime','P_Ts','InitialCondition','0');
add('dN',              'simulink/Math Operations/Sum','Inputs','+-');
add('ToRpm',           'simulink/Math Operations/Gain','Gain','60/(PL_Nrev*P_Ts)');
add('speed_meas_rpm',  'simulink/Sinks/Out1');

ln('duty_pct/1','ToDouble/1');       ln('ToDouble/1','DeadZone/1');
ln('DeadZone/1','MotorTF/1');        ln('MotorTF/1','Delay/1');
ln('Delay/1','ToCountsPerSec/1');    ln('ToCountsPerSec/1','CountsInt/1');
ln('CountsInt/1','SampleCounts/1');  ln('SampleCounts/1','FloorCounts/1');
ln('FloorCounts/1','dN/1');          ln('FloorCounts/1','PrevCounts/1');
ln('PrevCounts/1','dN/2');           ln('dN/1','ToRpm/1');
ln('ToRpm/1','speed_meas_rpm/1');
Simulink.BlockDiagram.arrangeSystem(sys);
end
