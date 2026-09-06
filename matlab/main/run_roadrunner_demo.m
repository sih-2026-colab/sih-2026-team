function run_roadrunner_demo()
%RUN_ROADRUNNER_DEMO Launch RoadRunner if installed; otherwise MATLAB AEB preview.
%
% Required: matching MATLAB + RoadRunner + Automated Driving Toolbox.
% Set ROADRUNNER_PROJECT to the folder that contains the .rrproj.

    thisDir = fileparts(mfilename('fullpath'));
    addpath(thisDir);
    addpath(fullfile(thisDir, '..', 'control'));
    addpath(fullfile(thisDir, '..', 'scenarios'));
    addpath(fullfile(thisDir, '..', 'simulink'));

    projectFolder = getenv('ROADRUNNER_PROJECT');
    if isempty(projectFolder)
        projectFolder = fullfile(thisDir, '..', '..', 'roadrunner', 'urban_intersection');
    end

    rrInstalled = exist('roadrunner', 'file') == 2;
    if ~rrInstalled
        warning('RoadRunner MATLAB API not found. Running MATLAB AEB preview instead.');
        preview_aeb();
        return
    end

    try
        rrApp = roadrunner(projectFolder);
        scenarioFile = fullfile(projectFolder, 'urban_intersection.rrscenario');
        if exist(scenarioFile, 'file')
            openScenario(rrApp, scenarioFile);
        end
        rrSim = createSimulation(rrApp);
        set(rrSim, 'PacingRate', 1);
        set(rrSim, 'SimulationCommand', 'Start');
        disp('RoadRunner scenario started. Attach simulink/ego_aeb.slx to the ego actor.');
    catch ME
        warning('RoadRunner launch failed (%s). Falling back to MATLAB preview.', ME.message);
        preview_aeb();
    end
end
