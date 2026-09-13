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
        % Preserve all four sensor identities for multi-sensor association.
        %% -------------------------------------------------

        if strcmp(sensorName,'FRONT_RADAR')

            sensorIndex = 1;

        elseif strcmp(sensorName,'REAR_RADAR')

            sensorIndex = 2;

        elseif strcmp(sensorName,'LEFT_CORNER_RADAR')
            sensorIndex = 3;
        elseif strcmp(sensorName,'RIGHT_CORNER_RADAR')
            sensorIndex = 4;

        else

            continue;

        end


        %% Convert radar range/angle into global X,Y

        [globalX, globalY] = ...
            radar_to_cartesian( ...
                detections(i), ego);


        %% Propagate simulated range/azimuth noise into Cartesian covariance.
        % The third coordinate constrains this planar model to z=0.

        theta=deg2rad(detections(i).angle+detections(i).mountAngle);
        range=abs(detections(i).range);
        J=[cos(theta) -range*sin(theta); sin(theta) range*cos(theta)];
        xyNoise=J*diag([.15^2 deg2rad(.5)^2])*J'+1e-6*eye(2);
        measurementNoise=blkdiag(xyNoise,.01);


        %% Create detection
        %
        % Notice:
        % actorID and actorName are NOT supplied.

        detectionCells{end+1,1} = ...
            objectDetection( ...
                time, ...
                [globalX; globalY; 0], ...
                'MeasurementNoise', ...
                measurementNoise, ...
                'SensorIndex', ...
                sensorIndex);

    end

end
