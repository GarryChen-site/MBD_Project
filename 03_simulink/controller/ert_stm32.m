function ert_stm32(model)
%ERT_STM32  Configure a Simulink model for Embedded Coder C generation targeting ARM Cortex-M4.
%   Applies Stage 4 section 2 settings via script (reproducible, version-controlled).
if ~bdIsLoaded(model), load_system(model); end
cs = getActiveConfigSet(model);

% Use Embedded Coder (ert.tlc) when available; fall back to grt.tlc otherwise
if license('test','RTW_Embedded_Coder')
    set_param(cs, 'SystemTargetFile', 'ert.tlc');
else
    set_param(cs, 'SystemTargetFile', 'grt.tlc');
end

set_try(cs, 'TargetLang', 'C');
set_try(cs, 'SolverType', 'Fixed-step');
set_try(cs, 'Solver', 'FixedStepDiscrete');
set_try(cs, 'FixedStep', 'P_Ts');
set_try(cs, 'GenCodeOnly', 'on');
set_try(cs, 'GenerateReport', 'off');
set_try(cs, 'MatFileLogging', 'off');
set_try(cs, 'SupportNonFinite', 'off');
set_try(cs, 'SupportComplex', 'off');
set_try(cs, 'PurelyIntegerCode', 'off');
set_try(cs, 'CodeInterfacePackaging', 'Nonreusable function');
set_try(cs, 'CombineOutputUpdateFcns', 'on');
set_try(cs, 'ProdHWDeviceType', 'ARM Compatible->ARM Cortex-M');
set_try(cs, 'ParameterPrecisionLossMsg', 'none');
end

function set_try(cs, param, val)
try
    set_param(cs, param, val);
catch
    % Ignore release-specific parameter name differences (ISS-4-01)
end
end
