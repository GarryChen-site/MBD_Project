function v = cal_to_vec(cal)
%CAL_TO_VEC  Pack a pedal calibration struct into the 1x9 single vector used by Simulink.
%   v = [cX cY dbX dbY maxX maxY modeId alo ahi]
if ~isfield(cal,'alo'), cal.alo = 100;  end
if ~isfield(cal,'ahi'), cal.ahi = 3995; end
modeId = 1 + strcmpi(cal.mode,'HALF');
v = single([cal.cX cal.cY cal.dbX cal.dbY cal.maxX cal.maxY modeId cal.alo cal.ahi]);
end
