function [approvedCandidate, ...
          approvedIndex, ...
          guardianMode, ...
          guardianResults] = ...
    cat_reflex_2d_guardian( ...
        candidates, ...
        plannerResults, ...
        proposedCandidate, ...
        proposedIndex, ...
        tracks, ...
        roadMinY, ...
        roadMaxY)

    %% =====================================================
    % AUTONEX CAT REFLEX 2-D SAFETY GUARDIAN
    %
    % Independent physics-based validation layer.
    %
    % Planner / AI proposes.
    % Guardian independently validates.
    %
    % It checks:
    % - front TTC
    % - rear TTC
    % - hard geometric conflict
    % - side proximity
    % - road boundary
    % - lateral acceleration
    %% =====================================================

    approvedCandidate = [];
    approvedIndex = NaN;

    guardianMode = ...
        'NO_ACTION';

    guardianResults = struct([]);


    if isempty(candidates)
        return;
    end


    %% =====================================================
    % GUARDIAN CONSTANTS
    %% =====================================================

    egoHalfWidth = 0.90;

    hardLongitudinalDistance = 5.5;

    hardLateralDistance = 2.0;

    sideCheckLongitudinal = 7.0;

    sideCheckLateral = 1.8;

    minimumFrontTTC = 2.0;

    minimumRearTTC = 1.5;

    maximumLateralAcceleration = 3.5;


    minimumAllowedY = ...
        roadMinY + egoHalfWidth;

    maximumAllowedY = ...
        roadMaxY - egoHalfWidth;


    %% =====================================================
    % CHECK EVERY CANDIDATE INDEPENDENTLY
    %% =====================================================

    for c = 1:length(candidates)

        trajectory = ...
            candidates(c).trajectory;


        hardConflicts = 0;

        sideConflicts = 0;

        minFrontTTC = inf;

        minRearTTC = inf;

        minDistance = inf;


        %% -------------------------------------------------
        % ROAD BOUNDARY
        %% -------------------------------------------------

        boundaryViolation = ...
            any(trajectory.y < minimumAllowedY) || ...
            any(trajectory.y > maximumAllowedY);


        %% -------------------------------------------------
        % LATERAL ACCELERATION
        %% -------------------------------------------------

        maxLateralAcceleration = ...
            max(abs(trajectory.ay));


        comfortViolation = ...
            maxLateralAcceleration > ...
            maximumLateralAcceleration;


        %% =================================================
        % FUTURE PHYSICS CHECK
        %% =================================================

        for j = 1:length(trajectory.time)

            h = ...
                trajectory.time(j);


            egoX = ...
                trajectory.x(j);

            egoY = ...
                trajectory.y(j);

            egoVx = ...
                trajectory.vx(j);


            %% =============================================
            % CHECK TRACKED ROAD USERS
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


                %% Predict object

                futureObjectX = ...
                    objectX + objectVx*h;

                futureObjectY = ...
                    objectY + objectVy*h;


                %% Relative geometry

                dx = ...
                    futureObjectX - egoX;

                dy = ...
                    futureObjectY - egoY;


                distance = ...
                    sqrt(dx^2 + dy^2);


                minDistance = ...
                    min(minDistance,distance);


                %% =========================================
                % HARD COLLISION REGION
                %% =========================================

                if abs(dx) <= hardLongitudinalDistance && ...
                   abs(dy) <= hardLateralDistance

                    hardConflicts = ...
                        hardConflicts + 1;

                end


                %% =========================================
                % SIDE INTRUSION
                %% =========================================

                if abs(dx) <= sideCheckLongitudinal && ...
                   abs(dy) <= sideCheckLateral

                    sideConflicts = ...
                        sideConflicts + 1;

                end


                %% =========================================
                % FRONT TTC
                %% =========================================

                if dx > 0 && ...
                   abs(dy) <= 2.2

                    closingSpeed = ...
                        egoVx - objectVx;


                    if closingSpeed > 0.10

                        frontTTC = ...
                            dx / closingSpeed;


                        if frontTTC >= 0

                            minFrontTTC = ...
                                min( ...
                                    minFrontTTC, ...
                                    frontTTC);

                        end

                    end

                end


                %% =========================================
                % REAR TTC
                %% =========================================

                if dx < 0 && ...
                   abs(dy) <= 2.2

                    rearClosingSpeed = ...
                        objectVx - egoVx;


                    if rearClosingSpeed > 0.10

                        rearTTC = ...
                            abs(dx) / ...
                            rearClosingSpeed;


                        if rearTTC >= 0

                            minRearTTC = ...
                                min( ...
                                    minRearTTC, ...
                                    rearTTC);

                        end

                    end

                end

            end

        end


        %% =================================================
        % PHYSICS SAFETY DECISION
        %% =================================================

        frontSafe = ...
            isinf(minFrontTTC) || ...
            minFrontTTC >= minimumFrontTTC;


        rearSafe = ...
            isinf(minRearTTC) || ...
            minRearTTC >= minimumRearTTC;


        geometrySafe = ...
            hardConflicts == 0;


        sideSafe = ...
            sideConflicts == 0;


        guardianSafe = ...
            frontSafe && ...
            rearSafe && ...
            geometrySafe && ...
            sideSafe && ...
            ~boundaryViolation && ...
            ~comfortViolation;


        %% =================================================
        % INDEPENDENT GUARDIAN RISK SCORE
        %% =================================================

        guardianRisk = 0;


        guardianRisk = ...
            guardianRisk + ...
            5000 * hardConflicts;


        guardianRisk = ...
            guardianRisk + ...
            1500 * sideConflicts;


        if boundaryViolation

            guardianRisk = ...
                guardianRisk + 10000;

        end


        if comfortViolation

            guardianRisk = ...
                guardianRisk + 2000;

        end


        %% Front TTC severity

        if ~isinf(minFrontTTC)

            guardianRisk = ...
                guardianRisk + ...
                700 * ...
                max( ...
                    0, ...
                    3.0 - minFrontTTC);

        end


        %% Rear TTC severity

        if ~isinf(minRearTTC)

            guardianRisk = ...
                guardianRisk + ...
                400 * ...
                max( ...
                    0, ...
                    2.5 - minRearTTC);

        end


        %% Small proximity term

        if isfinite(minDistance)

            guardianRisk = ...
                guardianRisk + ...
                20 / ...
                max(minDistance,0.5);

        end


        %% =================================================
        % SAVE
        %% =================================================

        guardianResults(c).safe = ...
            guardianSafe;

        guardianResults(c).risk = ...
            guardianRisk;

        guardianResults(c).hardConflicts = ...
            hardConflicts;

        guardianResults(c).sideConflicts = ...
            sideConflicts;

        guardianResults(c).minFrontTTC = ...
            minFrontTTC;

        guardianResults(c).minRearTTC = ...
            minRearTTC;

        guardianResults(c).minDistance = ...
            minDistance;

        guardianResults(c).boundaryViolation = ...
            boundaryViolation;

        guardianResults(c).maxLateralAcceleration = ...
            maxLateralAcceleration;

    end


    %% =====================================================
    % FIRST TRY TO APPROVE PLANNER PROPOSAL
    %% =====================================================

    if ~isempty(proposedCandidate) && ...
       ~isnan(proposedIndex)

        if guardianResults(proposedIndex).safe

            approvedCandidate = ...
                proposedCandidate;

            approvedIndex = ...
                proposedIndex;

            guardianMode = ...
                'APPROVED';

            return;

        end

    end


    %% =====================================================
    % PLANNER PROPOSAL REJECTED
    %
    % Find a guardian-safe alternative.
    %% =====================================================

    safeIndices = [];


    for i = 1:length(guardianResults)

        if guardianResults(i).safe

            safeIndices(end+1) = i; %#ok<AGROW>

        end

    end


    if ~isempty(safeIndices)

        %% Among guardian-safe trajectories,
        % allow planner score to decide preference.

        bestPlannerScore = inf;

        bestIndex = NaN;


        for n = 1:length(safeIndices)

            i = ...
                safeIndices(n);


            if plannerResults(i).score < ...
                    bestPlannerScore

                bestPlannerScore = ...
                    plannerResults(i).score;

                bestIndex = i;

            end

        end


        approvedCandidate = ...
            candidates(bestIndex);

        approvedIndex = ...
            bestIndex;

        guardianMode = ...
            'OVERRIDE_SAFE';

        return;

    end


    %% =====================================================
    % NO COMPLETELY SAFE TRAJECTORY
    %
    % Choose minimum guardian risk.
    %% =====================================================

    bestGuardianRisk = inf;

    bestIndex = NaN;


    for i = 1:length(guardianResults)

        totalRisk = ...
            guardianResults(i).risk + ...
            0.001 * plannerResults(i).score;


        if totalRisk < bestGuardianRisk

            bestGuardianRisk = ...
                totalRisk;

            bestIndex = i;

        end

    end


    if ~isnan(bestIndex)

        approvedCandidate = ...
            candidates(bestIndex);

        approvedIndex = ...
            bestIndex;

        guardianMode = ...
            'OVERRIDE_MINIMUM_RISK';

    end

end