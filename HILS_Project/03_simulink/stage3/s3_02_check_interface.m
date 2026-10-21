function r = s3_02_check_interface()
%S3_02_CHECK_INTERFACE  A3-5 (port list matches contract) and A3-6 (no double in Controller).
%   Compiles speed_ctrl, inspects every block output inside Controller.
%   Known limitation: the CONTRACT below is the reduced milestone interface (no CAN/AEB ports yet).
root = setup_paths();
mdl = 'speed_ctrl';
if ~isfile(fullfile(root,'controller','speed_ctrl.slx')), s3_01_build_speed_ctrl(); end
load_system(fullfile(root,'controller','speed_ctrl.slx'));
load_params_to_base(init_params());
ctrl = [mdl '/Controller'];

inExp  = {'pedal1_adc','pedal2_adc','speed_meas_rpm'};
outExp = {'duty_cmd_pct','speed_req_rpm','fault_bitmap','mode','s1_pct','p2_pct','e_pct','p_cmd_pct'};
inAct  = get_param(find_system(ctrl,'SearchDepth',1,'BlockType','Inport'),'Name');
outAct = get_param(find_system(ctrl,'SearchDepth',1,'BlockType','Outport'),'Name');
okPorts = isequal(inAct(:)', inExp) && isequal(outAct(:)', outExp);

feval(mdl,[],[],[],'compile');
cleanup = onCleanup(@() feval(mdl,[],[],[],'term'));
blks = find_system(ctrl,'LookUnderMasks','all','Type','Block');
bad = {};
for i = 1:numel(blks)
    if strcmp(get_param(blks{i},'BlockType'),'Inport'), continue; end
    try
        dt = get_param(blks{i},'CompiledPortDataTypes');
    catch, continue
    end
    if any(strcmp(dt.Outport,'double'))
        bad{end+1} = blks{i}; %#ok<AGROW>
    end
end
% input conversion blocks are allowed to take double, but must output single
r = struct('portsOk', okPorts, 'doubleBlocks', {bad}, 'pass', okPorts && isempty(bad));
fprintf('[S3-02] A3-5 ports match contract: %d | A3-6 blocks with double outputs: %d -> %s\n', okPorts, numel(bad), ternary(r.pass,'PASS','FAIL'));
for i = 1:numel(bad), fprintf('        double: %s\n', bad{i}); end
end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
