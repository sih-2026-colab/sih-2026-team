% run_perception.m
% Starter script to load objects from a MAT-file produced by Python,
% attempt to run a Simulink perception model if available, and save
% detected objects to an output MAT-file.

% Input: 'input_objects.mat' containing variable 'objects' (struct array)
% Output: 'perception_output.mat' containing variable 'detections' (struct array)

function run_perception()
    try
        data = load('input_objects.mat', 'objects');
        objects = data.objects;
    catch
        warning('input_objects.mat not found or missing ''objects''. Using empty array.');
        objects = struct([]);
    end

    detections = struct([]);

    % Try to run Simulink model if file exists
    modelPath1 = fullfile('..','simulink','perception');
    modelPath2 = fullfile('simulink','perception');
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

            % Common places for outputs: base workspace, simOut variable fields
            if evalin('base','exist(''detections'',''var'')')
                detections = evalin('base','detections');
            elseif isstruct(simOut) && isfield(simOut,'detections')
                detections = simOut.detections;
            elseif isstruct(simOut) && isfield(simOut,'logsout')
                % Attempt generic logsout parsing: look for a signal named 'detections'
                logs = simOut.logsout;
                try
                    % logsout is a Simulink.SimulationData.Dataset
                    idx = -1;
                    for k = 1:logs.numElements
                        nm = logs.getElement(k).Name;
                        if strcmpi(nm,'detections')
                            idx = k; break;
                        end
                    end
                    if idx > 0
                        s = logs.getElement(idx).Values;
                        % Expect s to be timeseries or struct — save as detections if possible
                        if isprop(s,'Data')
                            detections = s.Data;
                        end
                    else
                        warning('logsout present but no ''detections'' element found.');
                    end
                catch
                    warning('Could not parse logsout for detections.');
                end
            end
        catch ME
            warning('Simulink simulation failed: %s', ME.message);
        end
    end

    % Fallback: if detections still empty, copy input objects and add confidence
    if isempty(detections)
        detections = objects;
        for i = 1:length(detections)
            detections(i).confidence = 0.9; % placeholder
        end
    end

    save('perception_output.mat', 'detections');
    % Attempt to write JSON output for easier Python consumption
    try
        write_json(detections, fullfile('..','..','input_perception.json'));
    catch
        % ignore
    end
    disp('Perception run complete. Saved perception_output.mat and input_perception.json (if possible)');
end
