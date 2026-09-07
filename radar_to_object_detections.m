function detectionCells = ...
    radar_to_object_detections(detections, ego, time)

    %% =====================================================
    % Convert AutoNex radar detections into MATLAB
    % objectDetection reports for trackerGNN.
    %
    % IMPORTANT:
    % No actor ID is given to the tracker.
    %% =====================================================

    detectionCells = cell(0,1);

    for i = 1:length(detections)

        sensorName = detections(i).sensor;

        %% -------------------------------------------------
        % FOR NOW:
        % use FRONT + REAR radar only.
        %
        % Corner radar fusion comes later.
        %% -------------------------------------------------

        if strcmp(sensorName,'FRONT_RADAR')

            sensorIndex = 1;

        elseif strcmp(sensorName,'REAR_RADAR')

            sensorIndex = 2;

        else

            continue;

        end


        %% Convert radar range/angle into global X,Y

        [globalX, globalY] = ...
            radar_to_cartesian( ...
                detections(i), ego);


        %% Approximate Cartesian measurement covariance
        %
        % Later we will calculate this properly from
        % range + angle uncertainty.

        measurementNoise = ...
            diag([0.40^2 0.40^2]);


        %% Create detection
        %
        % Notice:
        % actorID and actorName are NOT supplied.

        detectionCells{end+1,1} = ...
            objectDetection( ...
                time, ...
                [globalX; globalY], ...
                'MeasurementNoise', ...
                measurementNoise, ...
                'SensorIndex', ...
                sensorIndex);

    end

end