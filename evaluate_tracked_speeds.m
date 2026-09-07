function results = evaluate_tracked_speeds( ...
    tracks, ego, horizons, speedsKmh, intent)

    %% =====================================================
    % AUTONEX SENSOR-BASED SPEED EVALUATOR
    %
    % Uses GNN tracks instead of true actor states.
    %% =====================================================

    currentSpeedKmh = ego.vx * 3.6;

    desiredCruiseKmh = 75;

    numSpeeds = length(speedsKmh);

    results = struct([]);


    %% =====================================================
    % IDENTIFY MOST LIKELY REAR VEHICLE
    %% =====================================================

    rearFound = false;

    rearX = 0;
    rearVx = 0;

    closestRearGap = inf;


    for k = 1:length(tracks)

        state = tracks(k).State;

        x = state(1);
        vx = state(2);

        y = state(3);

        dx = x - ego.x;
        dy = y - ego.y;


        % Rear vehicle:
        % behind ego and approximately in same lane
        if dx < 0 && abs(dy) <= 1.8

            gap = abs(dx);

            if gap < closestRearGap

                closestRearGap = gap;

                rearX = x;
                rearVx = vx;

                rearFound = true;

            end

        end

    end


    %% =====================================================
    % TEST EVERY CANDIDATE SPEED
    %% =====================================================

    for s = 1:numSpeeds

        candidateKmh = speedsKmh(s);

        candidateSpeed = ...
            candidateKmh / 3.6;


        surroundingConflicts = 0;

        minClearance = inf;


        %% =================================================
        % FUTURE MULTI-OBJECT CONFLICT CHECK
        %% =================================================

        for j = 1:length(horizons)

            h = horizons(j);


            %% Candidate ego future position

            egoFutureX = ...
                ego.x + candidateSpeed * h;

            egoFutureY = ego.y;


            %% Check all GNN tracks

            for k = 1:length(tracks)

                state = tracks(k).State;

                x = state(1);
                vx = state(2);

                y = state(3);
                vy = state(4);


                %% Ignore identified rear vehicle here
                %
                % Rear risk handled separately below.

                if rearFound

                    if x < ego.x && ...
                       abs(y - ego.y) <= 1.8

                        continue;

                    end

                end


                %% Predict tracked vehicle

                otherFutureX = ...
                    x + vx * h;

                otherFutureY = ...
                    y + vy * h;


                %% Relative future position

                dx = ...
                    otherFutureX - egoFutureX;

                dy = ...
                    otherFutureY - egoFutureY;


                clearance = ...
                    sqrt(dx^2 + dy^2);


                if clearance < minClearance

                    minClearance = clearance;

                end


                %% -----------------------------------------
                % USE TRACK UNCERTAINTY
                %% -----------------------------------------

                P = tracks(k).StateCovariance;

                positionUncertainty = ...
                    sqrt( ...
                        max(P(1,1),0) + ...
                        max(P(3,3),0));


                % Uncertainty grows into future
                futureUncertainty = ...
                    positionUncertainty + ...
                    0.30 * h;


                %% Dynamic safety envelope

                safeLongitudinal = ...
                    6.0 + ...
                    0.50 * futureUncertainty;


                safeLateral = ...
                    2.2 + ...
                    0.30 * futureUncertainty;


                longitudinalConflict = ...
                    abs(dx) <= safeLongitudinal;


                lateralConflict = ...
                    abs(dy) <= safeLateral;


                if longitudinalConflict && ...
                   lateralConflict

                    surroundingConflicts = ...
                        surroundingConflicts + 1;

                end

            end

        end


        %% =================================================
        % REAR TTC FROM TRACKED REAR VEHICLE
        %% =================================================

        if rearFound

            rearGap = ...
                ego.x - rearX;


            closingSpeed = ...
                rearVx - candidateSpeed;


            if rearGap <= 0

                rearTTC = 0;

            elseif closingSpeed > 0

                rearTTC = ...
                    rearGap / closingSpeed;

            else

                rearTTC = inf;

            end

        else

            rearGap = inf;
            rearTTC = inf;

        end


        %% =================================================
        % REAR RISK LEVEL
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
        % HARD SAFETY CHECKS
        %% =================================================

        frontSafe = ...
            surroundingConflicts == 0;


        rearSafe = ...
            isinf(rearTTC) || ...
            rearTTC >= 4;


        overallSafe = ...
            frontSafe && rearSafe;


        %% =================================================
        % COST 1 — PHYSICAL CONFLICT
        %% =================================================

        frontRiskCost = ...
            1000 * surroundingConflicts;


        %% =================================================
        % COST 2 — REAR RISK
        %% =================================================

        if isinf(rearTTC)

            rearRiskCost = 0;

        else

            rearRiskCost = ...
                20 / max(rearTTC,0.1);

        end


        %% =================================================
        % COST 3 — SPEED TRANSITION
        %% =================================================

        transitionCost = ...
            0.25 * ...
            abs(currentSpeedKmh - candidateKmh);


        %% =================================================
        % COST 4 — NORMAL CRUISE PREFERENCE
        %% =================================================

        cruisePreferenceCost = ...
            2.0 * ...
            abs(desiredCruiseKmh - candidateKmh);


        %% =================================================
        % COST 5 — CUT-IN INTENT EARLY WARNING
        %% =================================================

        intentRiskCost = 0;


        if ~isempty(intent)

            modifier = ...
                intent_reflex_modifier(intent);


            preventiveTarget = ...
                desiredCruiseKmh - ...
                modifier.targetSpeedReduction;


            % Penalize speeds above the preventive
            % target while cut-in probability is high.
            if candidateKmh > preventiveTarget

                intentRiskCost = ...
                    intent.probability * ...
                    100 * ...
                    (candidateKmh - preventiveTarget);

            end

        end


        %% =================================================
        % TOTAL SCORE
        %% =================================================

        totalScore = ...
            frontRiskCost + ...
            rearRiskCost + ...
            transitionCost + ...
            cruisePreferenceCost + ...
            intentRiskCost;


        %% =================================================
        % SAVE RESULT
        %% =================================================

        results(s).speedKmh = ...
            candidateKmh;

        results(s).frontConflicts = ...
            surroundingConflicts;

        results(s).minFrontClearance = ...
            minClearance;

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

        results(s).intentCost = ...
            intentRiskCost;

        results(s).score = ...
            totalScore;

    end

end