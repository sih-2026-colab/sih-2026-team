function [bestIntent, found] = ...
    find_highest_cutin_risk(tracks, ego, egoLaneY, model)

    if nargin<4, model=[]; end

    %% =====================================================
    % AUTONEX CUT-IN CANDIDATE SELECTION
    %
    % Reject tracks that are:
    % - too far from ego lane
    % - far behind ego
    % - far ahead and irrelevant
    %% =====================================================

    found = false;

    bestIntent = [];

    bestProbability = -inf;


    %% Candidate gating limits

    maxLateralDistance = 4.5;   % metres

    minLongitudinalDistance = -10;

    maxLongitudinalDistance = 45;


    %% =====================================================
    % CHECK ALL CONFIRMED TRACKS
    %% =====================================================

    for i = 1:length(tracks)

        intent = ...
            estimate_cutin_intent( ...
                tracks(i), ...
                ego, ...
                egoLaneY);

        if ~isempty(model)
            intent.heuristicProbability=intent.probability;
            intent.learnedProbability=predict_cutin_probability(model,tracks(i),ego,egoLaneY);
            % Conservative hybrid risk score, not a calibrated probability.
            intent.probability=max(intent.probability,intent.learnedProbability);
            levels={'LOW','MEDIUM','HIGH','CRITICAL'};
            intent.level=levels{1+sum(intent.probability>=[.35 .55 .75])};
        end


        %% -----------------------------------------------
        % LATERAL GATE
        %
        % Ignore Lane-1 vehicles when ego is in Lane 3
        %% -----------------------------------------------

        if abs(intent.lateralDistance) > ...
                maxLateralDistance

            continue;

        end


        %% -----------------------------------------------
        % LONGITUDINAL GATE
        %% -----------------------------------------------

        if intent.dx < minLongitudinalDistance

            continue;

        end


        if intent.dx > maxLongitudinalDistance

            continue;

        end


        %% -----------------------------------------------
        % FIND HIGHEST PROBABILITY CANDIDATE
        %% -----------------------------------------------

        if intent.probability > bestProbability

            bestProbability = ...
                intent.probability;

            bestIntent = ...
                intent;

            found = true;

        end

    end

end
