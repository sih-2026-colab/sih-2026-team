function results = ...
    evaluate_uncertainty_aware_speeds( ...
        tracks, ...
        fusedTracks, ...
        ego, ...
        horizons, ...
        speedsKmh, ...
        intent)

    %% =====================================================
    % AUTONEX UNCERTAINTY-AWARE CAT REFLEX SPEED EVALUATOR
    %
    % Uses:
    % - GNN tracks
    % - multimodal confidence
    % - multimodal uncertainty
    % - dynamic safety envelopes
    % - front/rear risk
    % - cut-in intent
    %% =====================================================

    currentSpeedKmh = ...
        ego.vx * 3.6;


    desiredCruiseKmh = 75;


    results = struct([]);


    %% =====================================================
    % 1. IDENTIFY REAR VEHICLE
    %% =====================================================

    rearFound = false;

    rearX = 0;

    rearVx = 0;

    closestRearGap = inf;


    for k = 1:length(tracks)

        state = ...
            tracks(k).State;


        x = state(1);

        vx = state(2);

        y = state(3);


        dx = ...
            x - ego.x;

        dy = ...
            y - ego.y;


        %% Rear + approximately same lane

        if dx < 0 && ...
           abs(dy) <= 1.8

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
    % 2. TEST EVERY SPEED
    %% =====================================================

    for s = 1:length(speedsKmh)

        candidateKmh = ...
            speedsKmh(s);


        candidateSpeed = ...
            candidateKmh / 3.6;


        surroundingConflicts = 0;


        minClearance = inf;


        maxLongitudinalEnvelope = 0;

        maxLateralEnvelope = 0;


        %% =================================================
        % 3. FUTURE TRAJECTORY CHECK
        %% =================================================

        for j = 1:length(horizons)

            h = ...
                horizons(j);


            %% Candidate ego trajectory

            %% =========================================
% ACCELERATION-LIMITED EGO PREDICTION
%% =========================================

[egoFutureX, ...
 egoFutureSpeed, ...
 ~] = ...
    predict_ego_candidate_state( ...
        ego.x, ...
        ego.vx, ...
        candidateKmh, ...
        h);


egoFutureY = ...
    ego.y;


            %% =============================================
            % CHECK ALL TRACKED OBJECTS
            %% =============================================

            for k = 1:length(tracks)

                state = ...
                    tracks(k).State;


                x = state(1);

                vx = state(2);

                y = state(3);

                vy = state(4);


                %% -----------------------------------------
                % REAR VEHICLE HANDLED SEPARATELY
                %% -----------------------------------------

                if rearFound && ...
                   x < ego.x && ...
                   abs(y - ego.y) <= 1.8

                    continue;

                end


                %% -----------------------------------------
                % PREDICT OBJECT
                %% -----------------------------------------

                otherFutureX = ...
                    x + vx * h;


                otherFutureY = ...
                    y + vy * h;


                dx = ...
                    otherFutureX - ...
                    egoFutureX;


                dy = ...
                    otherFutureY - ...
                    egoFutureY;


                clearance = ...
                    sqrt(dx^2 + dy^2);


                minClearance = ...
                    min( ...
                        minClearance, ...
                        clearance);


                %% =========================================
                % FIND MULTIMODAL CONFIDENCE
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


                %% =========================================
                % FUTURE UNCERTAINTY GROWTH
                %% =========================================

                futureUncertainty = ...
                    fusedUncertainty + ...
                    0.25 * h;


                %% =========================================
                % RELATIVE CLOSING SPEED
                %
                % Positive means ego is gaining.
                %% =========================================

                relativeClosingSpeed = ...
    max( ...
        egoFutureSpeed - vx, ...
        0);


                %% =========================================
                % DYNAMIC SAFETY ENVELOPE
                %% =========================================

                %% =========================================
% IS OBJECT MOVING TOWARD EGO PATH?
%% =========================================

lateralDistanceToEgoPath = ...
    ego.y - y;


movingTowardEgoPath = ...
    lateralDistanceToEgoPath * vy > 0;


%% =========================================
% CONTEXT-AWARE DYNAMIC SAFETY ENVELOPE
%% =========================================

envelope = ...
    calculate_contextual_safety_envelope( ...
        futureUncertainty, ...
        fusedConfidence, ...
        relativeClosingSpeed, ...
        vy, ...
        movingTowardEgoPath);


                safeLongitudinal = ...
                    envelope.longitudinal;


                safeLateral = ...
                    envelope.lateral;


                maxLongitudinalEnvelope = ...
                    max( ...
                        maxLongitudinalEnvelope, ...
                        safeLongitudinal);


                maxLateralEnvelope = ...
                    max( ...
                        maxLateralEnvelope, ...
                        safeLateral);


                %% =========================================
                % COLLISION / CONFLICT TEST
                %% =========================================

                longitudinalConflict = ...
                    abs(dx) <= ...
                    safeLongitudinal;


                lateralConflict = ...
                    abs(dy) <= ...
                    safeLateral;


                if longitudinalConflict && ...
                   lateralConflict

                    surroundingConflicts = ...
                        surroundingConflicts + 1;

                end

            end

        end


        %% =================================================
% 4. ACCELERATION-AWARE REAR RISK
%% =================================================

if rearFound

    %% Current rear gap

    rearGap = ...
        ego.x - rearX;


    %% Minimum predicted rear gap

    minRearGap = ...
        rearGap;


    %% Minimum predicted TTC

    rearTTC = inf;


    for rr = 1:length(horizons)

        hRear = ...
            horizons(rr);


        %% Ego predicted using REAL acceleration limits

        [egoRearFutureX, ...
         egoRearFutureSpeed, ...
         ~] = ...
            predict_ego_candidate_state( ...
                ego.x, ...
                ego.vx, ...
                candidateKmh, ...
                hRear);


        %% Rear vehicle constant-velocity prediction

        rearFutureX = ...
            rearX + ...
            rearVx * hRear;


        %% Predicted gap

        predictedRearGap = ...
            egoRearFutureX - ...
            rearFutureX;


        minRearGap = ...
            min( ...
                minRearGap, ...
                predictedRearGap);


        %% Rear closing velocity

        rearClosingSpeed = ...
            rearVx - ...
            egoRearFutureSpeed;


        %% Predicted TTC

        if predictedRearGap <= 0

            predictedRearTTC = 0;


        elseif rearClosingSpeed > 0

            predictedRearTTC = ...
                predictedRearGap / ...
                rearClosingSpeed;


        else

            predictedRearTTC = inf;

        end


        rearTTC = ...
            min( ...
                rearTTC, ...
                predictedRearTTC);

    end


else

    rearGap = inf;

    minRearGap = inf;

    rearTTC = inf;

end


        %% =================================================
        % 5. REAR RISK
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
        % 6. HARD SAFETY
        %% =================================================

        frontSafe = ...
            surroundingConflicts == 0;


        rearSafe = ...
    minRearGap >= 5.0 && ...
    ( ...
        isinf(rearTTC) || ...
        rearTTC >= 4 ...
    );


        overallSafe = ...
            frontSafe && ...
            rearSafe;


        %% =================================================
        % 7. COSTS
        %% =================================================

        frontRiskCost = ...
            1000 * ...
            surroundingConflicts;


        if isinf(rearTTC)

            rearRiskCost = 0;

        else

            rearRiskCost = ...
                20 / ...
                max(rearTTC,0.1);

        end


        transitionCost = ...
            0.25 * ...
            abs( ...
                currentSpeedKmh - ...
                candidateKmh);


        cruisePreferenceCost = ...
            2.0 * ...
            abs( ...
                desiredCruiseKmh - ...
                candidateKmh);


        %% =================================================
        % 8. INTENT EARLY-WARNING COST
        %% =================================================

        intentRiskCost = 0;


        if ~isempty(intent)

            modifier = ...
                intent_reflex_modifier( ...
                    intent);


            preventiveTarget = ...
                desiredCruiseKmh - ...
                modifier.targetSpeedReduction;


            if candidateKmh > ...
                    preventiveTarget

                intentRiskCost = ...
                    intent.probability * ...
                    100 * ...
                    ( ...
                    candidateKmh - ...
                    preventiveTarget);

            end

        end


        %% =================================================
        % 9. TOTAL SCORE
        %% =================================================

        totalScore = ...
            frontRiskCost + ...
            rearRiskCost + ...
            transitionCost + ...
            cruisePreferenceCost + ...
            intentRiskCost;


        %% =================================================
        % 10. RESULT
        %% =================================================

        results(s).speedKmh = ...
            candidateKmh;


        results(s).frontConflicts = ...
            surroundingConflicts;


        results(s).minFrontClearance = ...
            minClearance;


        results(s).rearGap = ...
            rearGap;
        results(s).minRearGap = ...
           minRearGap;


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


        results(s).maxLongitudinalEnvelope = ...
            maxLongitudinalEnvelope;


        results(s).maxLateralEnvelope = ...
            maxLateralEnvelope;


        results(s).score = ...
            totalScore;

    end

end