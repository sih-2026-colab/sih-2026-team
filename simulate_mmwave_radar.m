function detections = simulate_mmwave_radar( ...
    ego, actors, sensorName, mountAngleDeg, fovDeg, maxRange)

    %% =====================================================
    % AUTONEX SIMULATED 77–81 GHz FMCW RADAR
    %
    % Outputs:
    % - range
    % - azimuth angle
    % - relative radial velocity
    %
    % Negative radial velocity = object approaching
    % Positive radial velocity = object moving away
    %% =====================================================

    detections = struct([]);

    detectionCount = 0;


    %% -----------------------------------------------------
    % SIMULATED MEASUREMENT NOISE
    %% -----------------------------------------------------

    sigmaRange = 0.15;       % metres
    sigmaAngle = 0.5;        % degrees
    sigmaVelocity = 0.15;    % m/s


    %% -----------------------------------------------------
    % CHECK EVERY SURROUNDING ACTOR
    %% -----------------------------------------------------

    for i = 1:length(actors)

        % Do not detect the ego vehicle itself
        if actors(i).id == ego.id
            continue;
        end


        %% Relative position

        dx = actors(i).x - ego.x;
        dy = actors(i).y - ego.y;


        %% True range

        trueRange = sqrt(dx^2 + dy^2);


        % Ignore zero-distance objects
        if trueRange < 0.01
            continue;
        end


        %% Global azimuth angle

        globalAngle = atan2d(dy, dx);


        %% Convert into radar-relative angle

        relativeAngle = ...
            globalAngle - mountAngleDeg;


        % Wrap angle to [-180, 180]
        relativeAngle = ...
            mod(relativeAngle + 180, 360) - 180;


        %% -------------------------------------------------
        % FIELD OF VIEW CHECK
        %% -------------------------------------------------

        if abs(relativeAngle) > fovDeg / 2
            continue;
        end


        %% -------------------------------------------------
        % RANGE CHECK
        %% -------------------------------------------------

        if trueRange > maxRange
            continue;
        end


        %% -------------------------------------------------
        % RELATIVE VELOCITY
        %% -------------------------------------------------

        relativeVx = ...
            actors(i).vx - ego.vx;

        relativeVy = ...
            actors(i).vy - ego.vy;


        %% Unit vector along line-of-sight

        ux = dx / trueRange;
        uy = dy / trueRange;


        %% Radial relative velocity

        trueRadialVelocity = ...
            relativeVx * ux + ...
            relativeVy * uy;


        %% -------------------------------------------------
        % ADD REALISTIC MEASUREMENT NOISE
        %% -------------------------------------------------

        measuredRange = ...
            trueRange + ...
            sigmaRange * randn;


        measuredAngle = ...
            relativeAngle + ...
            sigmaAngle * randn;


        measuredRadialVelocity = ...
            trueRadialVelocity + ...
            sigmaVelocity * randn;


        %% -------------------------------------------------
        % SAVE DETECTION
        %% -------------------------------------------------

        detectionCount = ...
            detectionCount + 1;


        detections(detectionCount).sensor = ...
            sensorName;

            detections(detectionCount).mountAngle = ...
    mountAngleDeg;


        detections(detectionCount).actorID = ...
            actors(i).id;


        detections(detectionCount).actorName = ...
            actors(i).name;


        detections(detectionCount).range = ...
            measuredRange;


        detections(detectionCount).angle = ...
            measuredAngle;


        detections(detectionCount).radialVelocity = ...
            measuredRadialVelocity;


        detections(detectionCount).trueRange = ...
            trueRange;


        detections(detectionCount).trueAngle = ...
            relativeAngle;

    end

end