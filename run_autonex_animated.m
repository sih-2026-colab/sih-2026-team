function report = run_autonex_animated(projectFolder, exportVideo, options)
% Render the shared safe-space planner. The original lane demo is preserved separately.
if nargin<1, projectFolder=fileparts(mfilename('fullpath')); end
if nargin<2, exportVideo=true; end
if nargin<3, options=struct; end
oldPath=path; cleanup=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(projectFolder);
options.visualize=true; options.exportVideo=exportVideo;
if ~isfield(options,'videoFile')
    options.videoFile=fullfile(projectFolder,'autonex_safe_space_animation.mp4');
end
report=run_autonex_simulation(options);
end
