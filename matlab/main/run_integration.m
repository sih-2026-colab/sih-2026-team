function run_integration()
%RUN_INTEGRATION Perception + prediction stubs, then AEB prototype.
    thisDir = fileparts(mfilename('fullpath'));
    addpath(thisDir);
    addpath(fullfile(thisDir, '..', 'perception'));
    addpath(fullfile(thisDir, '..', 'prediction'));
    addpath(fullfile(thisDir, '..', 'planning'));
    addpath(fullfile(thisDir, '..', 'control'));
    addpath(fullfile(thisDir, '..', 'scenarios'));
    addpath(fullfile(thisDir, '..', 'utils'));

    disp('Starting MATLAB integration run...');

    try
        run(fullfile(thisDir, '..', 'perception', 'run_perception.m'));
    catch ME
        warning('Failed to run perception: %s', ME.message);
    end

    try
        run(fullfile(thisDir, '..', 'prediction', 'run_prediction.m'));
    catch ME
        warning('Failed to run prediction: %s', ME.message);
    end

    try
        run(fullfile(thisDir, '..', 'planning', 'run_planner.m'));
    catch ME
        warning('Failed to run planner: %s', ME.message);
    end

    disp('Running AEB prototype (MATLAB fallback if RoadRunner is absent)...');
    run_roadrunner_demo();

    disp('Running control layer (PID longitudinal + AEB override)...');
    run_control();

    disp('MATLAB integration run complete.');
end
