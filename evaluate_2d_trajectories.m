function results = evaluate_2d_trajectories( ...
    candidates, ...
    tracks, ...
    fusedTracks, ...
    ego, ...
    intent, ...
    roadMinY, ...
    roadMaxY)

    %% =====================================================
    % AUTONEX 2-D UNCERTAINTY-AWARE TRAJECTORY EVALUATOR
    %
    % Evaluates:
    % - front risk
    % - rear risk
    % - side risk
    % - multimodal uncertainty
    % - dynamic safety envelope
    % - road boundaries
    % - lateral comfort
    % - cut-in intent
    %% =====================================================

    results = struct([]);

    desiredCruiseKmh = 75;

    egoHalfWidth = 0.90;

    maxComfortableLateralAcceleration = 3.0;


    %% =====================================================
    % EVALUATE EVERY CANDIDATE
    %% =====================================================

    for c = 1:length(candidates)

        trajectory = ...
            candidates(c).trajectory;


        conflictCount = 0;

        frontConflicts = 0;

        rearConflicts = 0;

        sideConflicts = 0;


        minClearance = inf;

        minNormalizedSeparation = inf;


        proximityCost = 0;


        %% =================================================
        % ROAD BOUNDARY CHECK
        %% =================================================

        minimumAllowedY = ...
            roadMinY + egoHalfWidth;


        maximumAllowedY = ...
            roadMaxY - egoHalfWidth;


        boundaryViolation = ...
            any(trajectory.y < minimumAllowedY) || ...
            any(trajectory.y > maximumAllowedY);


        %% =================================================
        % LATERAL COMFORT
        %% =================================================

        maxLateralAcceleration = ...
            max(abs(trajectory.ay));


        comfortSafe = ...
            maxLateralAcceleration <= ...
            maxComfortableLateralAcceleration;


        %% =================================================
        % FUTURE TRAJECTORY CHECK
        %% =================================================

        for j = 1:length(trajectory.time)

            h = ...
                trajectory.time(j);


            egoFutureX = ...
                trajectory.x(j);


            egoFutureY = ...
                trajectory.y(j);


            egoFutureVx = ...
                trajectory.vx(j);


            egoFutureVy = ...
                trajectory.vy(j);


            %% =============================================
            % CHECK EVERY TRACKED OBJECT
            %% =============================================

            for k = 1:length(tracks)

                state = ...
                    tracks(k).State;


                objectX = ...
                    state(1);


                objectVx = ...
                    state(2);


                objectY = ...
                    state(3);


                objectVy = ...
                    state(4);


                %% -----------------------------------------
                % PREDICT OBJECT
                %% -----------------------------------------

                objectFutureX = ...
                    objectX + ...
                    objectVx * h;


                objectFutureY = ...
                    objectY + ...
                    objectVy * h;


                %% -----------------------------------------
                % RELATIVE FUTURE STATE
                %% -----------------------------------------

                dx = ...
                    objectFutureX - ...
                    egoFutureX;


                dy = ...
                    objectFutureY - ...
                    egoFutureY;


                clearance = ...
                    sqrt(dx^2 + dy^2);


                minClearance = ...
                    min( ...
                        minClearance, ...
                        clearance);


                %% =========================================
                % MULTIMODAL CONFIDENCE / UNCERTAINTY
                %% =========================================

                fusedConfidence = 0.60;

                fusedUncertainty = 1.50;


                trackID = ...
                    tracks(k).TrackID;


                for f = 1:length(fusedTracks)

                    if fusedTracks(f).trackID == trackID

                        fusedConfidence = ...
                            fusedTracks(f).fusedConfidence;


                        fusedUncertainty = ...
                            fusedTracks(f).fusedUncertainty;

                        break;

                    end

                end


                %% -----------------------------------------
                % FUTURE UNCERTAINTY GROWTH
                %% -----------------------------------------

                futureUncertainty = ...
                    fusedUncertainty + ...
                    0.25 * h;


                %% =========================================
                % LONGITUDINAL CLOSING SPEED
                %% =========================================

                if dx >= 0

                    % Object ahead

                    relativeClosingSpeed = ...
                        max( ...
                            egoFutureVx - objectVx, ...
                            0);

                else

                    % Object behind

                    relativeClosingSpeed = ...
                        max( ...
                            objectVx - egoFutureVx, ...
                            0);

                end


                %% =========================================
                % LATERAL CLOSING BEHAVIOUR
                %% =========================================

                relativeLateralVelocity = ...
                    objectVy - egoFutureVy;


                if abs(dy) > 0.05

                    lateralClosingSpeed = ...
                        -sign(dy) * ...
                        relativeLateralVelocity;

                else

                    lateralClosingSpeed = 0;

                end


                movingTowardEgoPath = ...
                    lateralClosingSpeed > 0.10;


                %% =========================================
                % CONTEXT-AWARE SAFETY ENVELOPE
                %% =========================================

                envelope = ...
                    calculate_contextual_safety_envelope( ...
                        futureUncertainty, ...
                        fusedConfidence, ...
                        relativeClosingSpeed, ...
                        relativeLateralVelocity, ...
                        movingTowardEgoPath);


                safeLongitudinal = ...
                    envelope.longitudinal;


                safeLateral = ...
                    envelope.lateral;


                %% =========================================
                % ELLIPTICAL SAFETY REGION
                %
                % normalized = 1 means safety boundary.
                %% =========================================

                normalizedSeparation = ...
                    (dx / safeLongitudinal)^2 + ...
                    (dy / safeLateral)^2;


                minNormalizedSeparation = ...
                    min( ...
                        minNormalizedSeparation, ...
                        normalizedSeparation);


                %% =========================================
                % HARD CONFLICT
                %% =========================================

                if normalizedSeparation <= 1

                    conflictCount = ...
                        conflictCount + 1;


                    %% Classify conflict direction

                    if dx > 1.0

                        frontConflicts = ...
                            frontConflicts + 1;


                    elseif dx < -1.0

                        rearConflicts = ...
                            rearConflicts + 1;


                    else

                        sideConflicts = ...
                            sideConflicts + 1;

                    end

                end


                %% =========================================
                % NEAR-MISS / PROXIMITY COST
                %
                % We also penalize trajectories that pass
                % close to the safety envelope even if they
                % technically remain outside it.
                %% =========================================

                if normalizedSeparation < 2.25

                    proximityCost = ...
                        proximityCost + ...
                        20 * ...
                        (2.25 - normalizedSeparation);

                end

            end

        end


        %% =================================================
        % CUT-IN INTENT COST
        %% =================================================

        intentCost = 0;


        if ~isempty(intent)

            %% Does this lateral maneuver move closer
            % to the suspected cut-in vehicle?

            currentLateralGap = ...
                abs(ego.y - intent.y);


            finalLateralGap = ...
                abs( ...
                    candidates(c).targetY - ...
                    intent.y);


            movingTowardIntent = ...
                max( ...
                    currentLateralGap - ...
                    finalLateralGap, ...
                    0);


            intentCost = ...
                intentCost + ...
                intent.probability * ...
                80 * ...
                movingTowardIntent;


            %% Preventively discourage excessive speed
            % when cut-in probability is high.

            modifier = ...
                intent_reflex_modifier(intent);


            preventiveTarget = ...
                desiredCruiseKmh - ...
                modifier.targetSpeedReduction;


            if candidates(c).targetSpeedKmh > ...
                    preventiveTarget

                intentCost = ...
                    intentCost + ...
                    intent.probability * ...
                    30 * ...
                    ( ...
                    candidates(c).targetSpeedKmh - ...
                    preventiveTarget);

            end

        end


        %% =================================================
        % SPEED COST
        %% =================================================

        speedCost = ...
            1.5 * ...
            abs( ...
                desiredCruiseKmh - ...
                candidates(c).targetSpeedKmh);


        %% =================================================
% LATERAL MANEUVER COST
%
% KEEP = inexpensive recentering.
% Other lateral maneuvers = stronger penalty.
%% =================================================

lateralDisplacement = ...
    abs( ...
        candidates(c).targetY - ...
        ego.y);


if strcmp( ...
        candidates(c).name, ...
        'KEEP')

    maneuverCost = ...
        0.5 * ...
        lateralDisplacement;

else

    maneuverCost = ...
        4.0 * ...
        lateralDisplacement;

end



        %% =================================================
        % COMFORT COST
        %% =================================================

        comfortCost = ...
            5.0 * ...
            maxLateralAcceleration;


        %% =================================================
        % HARD PENALTIES
        %% =================================================

        conflictCost = ...
            1000 * ...
            conflictCount;


        if boundaryViolation

            boundaryCost = 5000;

        else

            boundaryCost = 0;

        end


        if comfortSafe

            comfortViolationCost = 0;

        else

            comfortViolationCost = 1500;

        end


        %% =================================================
        % TOTAL SCORE
        %% =================================================

        totalScore = ...
            conflictCost + ...
            proximityCost + ...
            intentCost + ...
            speedCost + ...
            maneuverCost + ...
            comfortCost + ...
            boundaryCost + ...
            comfortViolationCost;


        %% =================================================
        % SAFE / UNSAFE
        %% =================================================

        overallSafe = ...
            conflictCount == 0 && ...
            ~boundaryViolation && ...
            comfortSafe;


        %% =================================================
        % SAVE RESULT
        %% =================================================

        results(c).candidateID = ...
            candidates(c).id;


        results(c).name = ...
            candidates(c).name;


        results(c).targetSpeedKmh = ...
            candidates(c).targetSpeedKmh;


        results(c).targetY = ...
            candidates(c).targetY;


        results(c).safe = ...
            overallSafe;


        results(c).conflictCount = ...
            conflictCount;


        results(c).frontConflicts = ...
            frontConflicts;


        results(c).rearConflicts = ...
            rearConflicts;


        results(c).sideConflicts = ...
            sideConflicts;


        results(c).minClearance = ...
            minClearance;


        results(c).minNormalizedSeparation = ...
            minNormalizedSeparation;


        results(c).maxLateralAcceleration = ...
            maxLateralAcceleration;


        results(c).boundaryViolation = ...
            boundaryViolation;


        results(c).comfortSafe = ...
            comfortSafe;


        results(c).proximityCost = ...
            proximityCost;


        results(c).intentCost = ...
            intentCost;


        results(c).score = ...
            totalScore;

    end

end