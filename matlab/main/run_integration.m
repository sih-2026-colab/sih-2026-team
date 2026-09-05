% run_integration.m
% Top-level integration script to run perception and planning in sequence.
% Expects 'input_objects.mat' to exist (written by Python) and writes
% 'planning_output.mat' at the end.

function run_integration()
    disp('Starting MATLAB integration run...');

    % Run perception (assumes run_perception.m is on path or in subfolder)
    try
        run('../perception/run_perception.m');
    catch ME
        warning('Failed to run perception: %s', ME.message);
    end

    % Run planner
    try
        run('../planning/run_planner.m');
    catch ME
        warning('Failed to run planner: %s', ME.message);
    end

    disp('MATLAB integration run complete. Check planning_output.mat');
end
