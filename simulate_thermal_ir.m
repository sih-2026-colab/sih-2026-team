function detections = simulate_thermal_ir(ego, actors)

    %% =====================================================
    % AUTONEX PASSIVE THERMAL IR SENSOR
    %
    % Prototype LWIR-style sensor.
    %
    % It does NOT transmit an IR beam.
    % It observes thermal radiation from objects.
    %
    % Output:
    % - bearing angle
    % - estimated confidence
    % - noisy object position for simulation validation
    %% =====================================================

    detections = struct([]);

    count = 0;

    %% Sensor configuration

    maxRange = 80;          % metres
    fovDeg = 100;           % total field of view

    %% Noise

    sigmaAngle = 0.8;       % degrees
    sigmaPosition = 0.6;    % metres


    for i = 1:length(actors)

        %% Do not detect ego

        if actors(i).id == ego.id
            continue;
        end


        %% Relative location

        dx = actors(i).x - ego.x;
        dy = actors(i).y - ego.y;

        range = sqrt(dx^2 + dy^2);

        angle = atan2d(dy, dx);


        %% Forward thermal camera only for now

        if abs(angle) > fovDeg / 2
            continue;
        end


        if range > maxRange
            continue;
        end


        %% =================================================
        % BASE THERMAL CONFIDENCE BY OBJECT TYPE
        %% =================================================

        if strcmpi(actors(i).type, 'pedestrian')

            baseConfidence = 0.95;

        elseif strcmpi(actors(i).type, 'animal')

            baseConfidence = 0.93;

        elseif strcmpi(actors(i).type, 'car')

            baseConfidence = 0.78;

        elseif strcmpi(actors(i).type, 'bike')

            baseConfidence = 0.82;

        else

            baseConfidence = 0.65;

        end


        %% =================================================
        % RANGE EFFECT
        %% =================================================

        rangeFactor = ...
            max(0.30, 1 - range / 120);


        confidence = ...
            baseConfidence * rangeFactor;


        %% Add small confidence variation

        confidence = ...
            confidence + 0.03 * randn;


        confidence = ...
            max(0, min(confidence, 1));


        %% Noisy thermal bearing

        measuredAngle = ...
            angle + sigmaAngle * randn;


        %% Prototype position estimate
        %
        % In a real monocular thermal camera,
        % accurate depth would require additional geometry,
        % stereo/depth, or radar fusion.
        %
        % We simulate a rough image-to-world estimate here.

        measuredX = ...
            actors(i).x + sigmaPosition * randn;

        measuredY = ...
            actors(i).y + sigmaPosition * randn;


        %% Save

        count = count + 1;

        detections(count).sensor = ...
            'THERMAL_IR';

        detections(count).angle = ...
            measuredAngle;

        detections(count).confidence = ...
            confidence;

        detections(count).x = ...
            measuredX;

        detections(count).y = ...
            measuredY;

        detections(count).type = ...
            actors(i).type;

    end

end