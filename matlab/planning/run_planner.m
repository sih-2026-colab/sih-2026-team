% run_planner.m
% Starter script to load perception detections from a MAT-file,
% attempt to run a Simulink planner model, and save planned trajectories.

% Input: 'perception_output.mat' containing variable 'detections'
% Output: 'planning_output.mat' containing variable 'plans'

function run_planner()
    try
        data = load('perception_output.mat', 'detections');
        detections = data.detections;
    catch
        warning('perception_output.mat not found or missing ''detections''. Using empty array.');
        detections = struct([]);
    end

    plans = struct([]);

    % Try to run Simulink planner if exists
    modelPath1 = fullfile('..','simulink','planner');
    modelPath2 = fullfile('simulink','planner');
    modelFile = '';
    if exist([modelPath1 '.slx'],'file')
        modelFile = modelPath1;
    elseif exist([modelPath2 '.slx'],'file')
        modelFile = modelPath2;
    end

    if ~isempty(modelFile)
        try
            load_system(modelFile);
            simOut = sim(modelFile, 'ReturnWorkspaceOutputs', 'on', 'SaveOutput', 'on', 'SaveFormat', 'Structure');

            if evalin('base','exist(''plans'',''var'')')
                plans = evalin('base','plans');
            elseif isstruct(simOut) && isfield(simOut,'plans')
                plans = simOut.plans;
            elseif isstruct(simOut) && isfield(simOut,'logsout')
                logs = simOut.logsout;
                try
                    idx = -1;
                    for k = 1:logs.numElements
                        nm = logs.getElement(k).Name;
                        if strcmpi(nm,'plans')
                            idx = k; break;
                        end
                    end
                    if idx > 0
                        s = logs.getElement(idx).Values;
                        if isprop(s,'Data')
                            plans = s.Data;
                        end
                    else
                        warning('logsout present but no ''plans'' element found.');
                    end
                catch
                    warning('Could not parse logsout for plans.');
                end
            else
                warning('Simulink planner produced no usable ''plans'' variable; falling back to stub.');
            end
        catch ME
            warning('Simulink simulation failed: %s', ME.message);
        end
    end

    % Fallback: simple stub plan if plans empty
    if isempty(plans)
        for i = 1:length(detections)
            plans(i).id = detections(i).id;
            plans(i).waypoints = [detections(i).x; detections(i).y; detections(i).speed];
            plans(i).status = 'ok';
        end
    end

    save('planning_output.mat', 'plans');
    try
        write_json(plans, fullfile('..','..','planning_output.json'));
    catch
        % ignore
    end
    disp('Planning run complete. Saved planning_output.mat and planning_output.json (if possible)');
end
