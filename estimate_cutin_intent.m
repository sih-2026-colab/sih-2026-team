function intent = estimate_cutin_intent(track, ego, egoLaneY)

    %% =====================================================
    % AUTONEX CUT-IN INTENT ESTIMATOR
    %
    % Uses tracked:
    %   X, Y
    %   Vx, Vy
    %   uncertainty
    %
    % Output:
    %   probability 0 -> 1
    %% =====================================================


    state = track.State;

    x  = state(1);
    vx = state(2);

    y  = state(3);
    vy = state(4);


    %% -----------------------------------------------------
    % RELATIVE POSITION
    %% -----------------------------------------------------

    dx = x - ego.x;

    lateralDistance = ...
        egoLaneY - y;


    %% -----------------------------------------------------
    % IS OBJECT MOVING TOWARD EGO LANE?
    %% -----------------------------------------------------

    movingTowardLane = ...
        lateralDistance * vy > 0;


    %% -----------------------------------------------------
    % ESTIMATED TIME TO CROSS EGO LANE
    %% -----------------------------------------------------

    if movingTowardLane && abs(vy) > 0.05

        timeToLaneCrossing = ...
            abs(lateralDistance) / abs(vy);

    else

        timeToLaneCrossing = inf;

    end


    %% =====================================================
    % FEATURE 1 — LATERAL SPEED
    %% =====================================================

    lateralSpeedScore = ...
        min(abs(vy) / 2.0, 1);


    %% =====================================================
    % FEATURE 2 — TIME TO LANE CROSSING
    %% =====================================================

    if isinf(timeToLaneCrossing)

        crossingScore = 0;

    else

        crossingScore = ...
            exp(-timeToLaneCrossing / 1.5);

    end


    %% =====================================================
    % FEATURE 3 — DISTANCE FROM EGO LANE
    %% =====================================================

    laneDistanceScore = ...
        exp(-abs(lateralDistance) / 2.0);


    %% =====================================================
    % FEATURE 4 — LONGITUDINAL PROXIMITY
    %% =====================================================

    longitudinalScore = ...
        exp(-abs(dx) / 25);


    %% =====================================================
    % RAW CUT-IN PROBABILITY
    %% =====================================================

    probability = ...
        0.35 * lateralSpeedScore + ...
        0.30 * crossingScore + ...
        0.20 * laneDistanceScore + ...
        0.15 * longitudinalScore;


    %% If moving away from ego lane,
    % strongly reduce cut-in probability

    if ~movingTowardLane

        probability = ...
            probability * 0.10;

    end


    %% Keep probability between 0 and 1

    probability = ...
        max(0, min(probability, 1));


    %% =====================================================
    % TRACK UNCERTAINTY
    %% =====================================================

    P = track.StateCovariance;

    positionUncertainty = ...
        sqrt(P(1,1) + P(3,3));


    velocityUncertainty = ...
        sqrt(P(2,2) + P(4,4));


    %% =====================================================
    % INTENT LEVEL
    %% =====================================================

    if probability >= 0.75

        level = 'CRITICAL';

    elseif probability >= 0.55

        level = 'HIGH';

    elseif probability >= 0.35

        level = 'MEDIUM';

    else

        level = 'LOW';

    end


    %% =====================================================
    % OUTPUT STRUCTURE
    %% =====================================================

    intent.trackID = ...
        track.TrackID;

    intent.x = x;
    intent.y = y;

    intent.vx = vx;
    intent.vy = vy;

    intent.dx = dx;

    intent.lateralDistance = ...
        lateralDistance;

    intent.timeToLaneCrossing = ...
        timeToLaneCrossing;

    intent.probability = ...
        probability;

    intent.level = ...
        level;

    intent.positionUncertainty = ...
        positionUncertainty;

    intent.velocityUncertainty = ...
        velocityUncertainty;

end