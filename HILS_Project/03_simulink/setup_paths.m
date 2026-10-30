function root = setup_paths()
%SETUP_PATHS  Add all MBD_HILS_Project MATLAB folders to the path. Run once per session.
%   root = folder containing this file (03_simulink).
root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root,'lib'), fullfile(root,'stage2'), fullfile(root,'stage3'), ...
        fullfile(root,'stage4'), fullfile(root,'plant'), fullfile(root,'controller'), fullfile(root,'tests'));
end
