function build_ego_aeb()
%BUILD_EGO_AEB Create simulink/ego_aeb.slx when MATLAB/Simulink is available.
%
% The model is a longitudinal plant + AEB MATLAB Function. After RoadRunner
% is connected, replace the plant with a RoadRunner Scenario block and keep
% the same AEB logic (matlab/control/aeb_logic.m).

    thisDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(thisDir, '..', 'control'));
    addpath(fullfile(thisDir, '..', 'scenarios'));

    repoRoot = fullfile(thisDir, '..', '..');
    outFile = fullfile(repoRoot, 'simulink', 'ego_aeb.slx');
    if ~exist(fullfile(repoRoot, 'simulink'), 'dir')
        mkdir(fullfile(repoRoot, 'simulink'));
    end

    model = 'ego_aeb';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    if exist(outFile, 'file')
        delete(outFile);
    end

    new_system(model);
    open_system(model);

    add_block('simulink/Sources/Clock', [model '/Clock'], 'Position', [30 40 60 70]);
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Fcn', ...
        [model '/AEBPlant'], 'Position', [140 30 320 90]);
    set_param([model '/AEBPlant'], 'fcn', 'ego_aeb_plant');
    add_block('simulink/Sinks/Scope', [model '/Scope'], 'Position', [400 35 430 75]);
    add_line(model, 'Clock/1', 'AEBPlant/1');
    add_line(model, 'AEBPlant/1', 'Scope/1');

    set_param(model, 'StopTime', '4');
    save_system(model, outFile);
    close_system(model);
    fprintf('Wrote %s\n', outFile);
    fprintf('Plant function: matlab/simulink/ego_aeb_plant.m\n');
end
