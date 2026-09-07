function detections = ...
    simulate_rgb_camera(ego, actors, environment)

    %% =====================================================
    % AUTONEX RGB CAMERA PERCEPTION MODEL
    %
    % Prototype camera-level perception simulation.
    %
    % Environment:
    % environment.lightLevel
    % environment.visibility
    %
    % Both values:
    % 0 = very poor
    % 1 = excellent
    %% =====================================================

    detections = struct([]);

    count = 0;


    %% =====================================================
    % CAMERA CONFIGURATION
    %% =====================================================

    maxRange = 100;       % metres

    fovDeg = 90;          % forward camera FOV


    %% =====================================================
    % ENVIRONMENT
    %% =====================================================

    lightLevel = ...
        max(0, min(environment.lightLevel,1));

    visibility = ...
        max(0, min(environment.visibility,1));


    %% =====================================================
    % CHECK ALL OBJECTS
    %% =====================================================

    for i = 1:length(actors)

        %% Ignore ego

        if actors(i).id == ego.id
            continue;
        end


        %% Relative position

        dx = ...
            actors(i).x - ego.x;

        dy = ...
            actors(i).y - ego.y;


        range = ...
            sqrt(dx^2 + dy^2);


        angle = ...
            atan2d(dy,dx);


        %% =================================================
        % FORWARD CAMERA ONLY
        %% =================================================

        if dx <= 0
            continue;
        end


        if abs(angle) > fovDeg/2
            continue;
        end


        if range > maxRange
            continue;
        end


        %% =================================================
        % BASE CLASSIFICATION CONFIDENCE
        %% =================================================

        if strcmpi(actors(i).type,'car')

            baseConfidence = 0.94;

        elseif strcmpi(actors(i).type,'pedestrian')

            baseConfidence = 0.95;

        elseif strcmpi(actors(i).type,'bike')

            baseConfidence = 0.92;

        elseif strcmpi(actors(i).type,'animal')

            baseConfidence = 0.86;

        else

            baseConfidence = 0.70;

        end


        %% =================================================
        % LIGHT EFFECT
        %% =================================================

        lightFactor = ...
            0.30 + ...
            0.70 * lightLevel;


        %% =================================================
        % VISIBILITY EFFECT
        %% =================================================

        visibilityFactor = ...
            0.30 + ...
            0.70 * visibility;


        %% =================================================
        % RANGE EFFECT
        %% =================================================

        rangeFactor = ...
            max( ...
                0.20, ...
                1 - range/130);


        %% =================================================
        % RGB CONFIDENCE
        %% =================================================

        confidence = ...
            baseConfidence * ...
            lightFactor * ...
            visibilityFactor * ...
            rangeFactor;


        %% Random detector variation

        confidence = ...
            confidence + ...
            0.025 * randn;


        confidence = ...
            max(0,min(confidence,1));


        %% =================================================
        % CAMERA MEASUREMENT NOISE
        %
        % Camera becomes less accurate in poor lighting
        % or poor visibility.
        %% =================================================

        sigmaAngle = ...
            0.30 + ...
            1.20 * (1-lightLevel) + ...
            0.80 * (1-visibility);


        sigmaPosition = ...
            0.40 + ...
            1.30 * (1-lightLevel) + ...
            1.00 * (1-visibility);


        measuredAngle = ...
            angle + ...
            sigmaAngle * randn;


        measuredX = ...
            actors(i).x + ...
            sigmaPosition * randn;


        measuredY = ...
            actors(i).y + ...
            sigmaPosition * randn;


        %% =================================================
        % SAVE DETECTION
        %% =================================================

        count = count + 1;


        detections(count).sensor = ...
            'RGB_CAMERA';


        detections(count).range = ...
            range;


        detections(count).angle = ...
            measuredAngle;


        detections(count).x = ...
            measuredX;


        detections(count).y = ...
            measuredY;


        detections(count).confidence = ...
            confidence;


        detections(count).type = ...
            actors(i).type;

    end

end