function candidates = ...
    generate_2d_candidate_set( ...
        ego, ...
        horizons, ...
        referenceY)

    %% =====================================================
    % AUTONEX 2-D MANEUVER CANDIDATE GENERATOR
    %
    % referenceY:
    % nominal desired path center.
    %
    % Current highway example:
    % Lane 3 center = Y 7.0
    %
    % Later this reference will come from
    % drivable-space / free-space planning instead of lanes.
    %% =====================================================


    %% =====================================================
    % BACKWARD COMPATIBILITY
    %% =====================================================

    if nargin < 3

        referenceY = ...
            ego.y;

    end


    candidates = struct([]);

    count = 0;


    %% =====================================================
    % LONGITUDINAL OPTIONS
    %% =====================================================

    speedOptions = [
        75
        73
        70
        65
    ];


    %% =====================================================
    % LATERAL STRATEGIES
    %% =====================================================

    lateralNames = {
        'KEEP'
        'MICRO_POSITIVE_Y'
        'MICRO_NEGATIVE_Y'
        'SHIFT_LANE2'
    };


    %% =====================================================
    % IMPORTANT CHANGE:
    %
    % All temporary avoidance offsets are based on the
    % nominal reference path, NOT the ego's current offset.
    %% =====================================================

    keepTarget = ...
        referenceY;


    positiveTarget = ...
        min( ...
            referenceY + 0.45, ...
            7.50);


    negativeTarget = ...
        max( ...
            referenceY - 0.45, ...
            0.50);


    lane2Target = ...
        3.50;


    lateralTargets = [
        keepTarget
        positiveTarget
        negativeTarget
        lane2Target
    ];


    %% =====================================================
    % MANEUVER TIMES
    %% =====================================================

    lateralTimes = [
        1.2
        1.2
        1.2
        3.0
    ];


    %% =====================================================
    % GENERATE ALL COMBINATIONS
    %% =====================================================

    for l = 1:length(lateralTargets)

        for s = 1:length(speedOptions)

            count = ...
                count + 1;


            targetSpeed = ...
                speedOptions(s);


            targetY = ...
                lateralTargets(l);


            trajectory = ...
                generate_2d_trajectory( ...
                    ego, ...
                    targetSpeed, ...
                    targetY, ...
                    horizons, ...
                    lateralTimes(l));


            candidates(count).id = ...
                count;


            candidates(count).name = ...
                lateralNames{l};


            candidates(count).targetSpeedKmh = ...
                targetSpeed;


            candidates(count).targetY = ...
                targetY;


            candidates(count).maneuverTime = ...
                lateralTimes(l);


            candidates(count).trajectory = ...
                trajectory;

        end

    end

end