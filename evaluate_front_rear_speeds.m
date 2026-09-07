function results = evaluate_front_rear_speeds( ...
    actors, horizons, speedsKmh)

    %% =====================================================
    % PREDICT ALL SURROUNDING VEHICLES
    %% =====================================================

    predictions = ...
        predict_actor_trajectories(actors, horizons);


    %% =====================================================
    % IMPORTANT ACTORS
    %% =====================================================

    ego = actors(1);
    rear = actors(2);

    % Current ego speed in km/h
    currentSpeedKmh = ego.vx * 3.6;


    %% =====================================================
    % SAFETY ENVELOPE
    %% =====================================================

    % Approximate longitudinal safety distance
    safeLongitudinal = 6.0;

    % Approximate lateral safety distance
    safeLateral = 2.2;


    %% Number of candidate speeds

    numSpeeds = length(speedsKmh);

    results = struct([]);


    %% =====================================================
    % TEST EVERY CANDIDATE SPEED
    %% =====================================================

    for s = 1:numSpeeds

        candidateKmh = speedsKmh(s);

        % Convert km/h to m/s
        candidateSpeed = candidateKmh / 3.6;


        %% -------------------------------------------------
        % FRONT / LATERAL RISK VARIABLES
        %% -------------------------------------------------

        surroundingConflicts = 0;

        minFrontClearance = inf;


        %% =================================================
        % FUTURE FRONT + SIDE COLLISION CHECK
        %% =================================================

        for j = 1:length(horizons)

            h = horizons(j);


            % Predict where ego would be
            % if it used this candidate speed
            egoFutureX = ...
                ego.x + candidateSpeed * h;

            egoFutureY = ego.y;


            %% Compare with actors 3 onwards
            %
            % Actor 2 is the rear vehicle.
            % It is handled separately below.

            for i = 3:length(actors)

                otherX = predictions(i,j).x;
                otherY = predictions(i,j).y;


                % Relative position
                dx = otherX - egoFutureX;
                dy = otherY - egoFutureY;


                % Actual Euclidean clearance
                clearance = ...
                    sqrt(dx^2 + dy^2);


                % Save minimum clearance
                if clearance < minFrontClearance

                    minFrontClearance = clearance;

                end


                %% Determine predicted conflict

                longitudinalConflict = ...
                    abs(dx) <= safeLongitudinal;

                lateralConflict = ...
                    abs(dy) <= safeLateral;


                if longitudinalConflict && lateralConflict

                    surroundingConflicts = ...
                        surroundingConflicts + 1;

                end

            end

        end


        %% =================================================
        % REAR COLLISION CHECK
        %% =================================================

        [rearGap, rearTTC] = ...
            calculate_rear_ttc( ...
                ego.x, ...
                candidateSpeed, ...
                rear.x, ...
                rear.vx);


        %% =================================================
        % REAR RISK LEVEL
        %
        % 0 = LOW
        % 1 = CAUTION
        % 2 = HIGH
        % 3 = CRITICAL
        %% =================================================

        if isinf(rearTTC)

            rearRisk = 0;


        elseif rearTTC < 4

            rearRisk = 3;


        elseif rearTTC < 6

            rearRisk = 2;


        elseif rearTTC < 8

            rearRisk = 1;


        else

            rearRisk = 0;

        end


        %% =================================================
        % FRONT SAFETY
        %% =================================================

        frontSafe = ...
            surroundingConflicts == 0;


        %% =================================================
        % REAR SAFETY
        %% =================================================

        rearSafe = ...
            isinf(rearTTC) || rearTTC >= 4;


        %% =================================================
        % OVERALL SAFETY
        %% =================================================

        overallSafe = ...
            frontSafe && rearSafe;


        %% =================================================
        % AUTONEX COST FUNCTION
        %% =================================================

        % Normal desired cruising speed
        desiredCruiseKmh = 75;


        %% 1. Transition cost
        %
        % Avoid unnecessary sudden speed changes.

        transitionCost = ...
            0.25 * abs( ...
                currentSpeedKmh - candidateKmh);


        %% 2. Cruise preference cost
        %
        % Once danger disappears,
        % encourage vehicle to return toward 75 km/h.

        cruisePreferenceCost = ...
            2.0 * abs( ...
                desiredCruiseKmh - candidateKmh);


        %% 3. Rear risk cost

        if isinf(rearTTC)

            rearRiskCost = 0;

        else

            rearRiskCost = ...
                20 / max(rearTTC, 0.1);

        end


        %% 4. Front collision cost
        %
        % Predicted conflicts receive
        % a very large penalty.

        frontRiskCost = ...
            surroundingConflicts * 1000;


        %% =================================================
        % FINAL SCORE
        %
        % Lower score = better action
        %% =================================================

        totalScore = ...
            frontRiskCost + ...
            rearRiskCost + ...
            transitionCost + ...
            cruisePreferenceCost;


        %% =================================================
        % SAVE RESULTS
        %% =================================================

        results(s).speedKmh = ...
            candidateKmh;


        results(s).frontConflicts = ...
            surroundingConflicts;


        results(s).minFrontClearance = ...
            minFrontClearance;


        results(s).rearGap = ...
            rearGap;


        results(s).rearTTC = ...
            rearTTC;


        results(s).rearRisk = ...
            rearRisk;


        results(s).frontSafe = ...
            frontSafe;


        results(s).rearSafe = ...
            rearSafe;


        results(s).safe = ...
            overallSafe;


        results(s).score = ...
            totalScore;

    end

end