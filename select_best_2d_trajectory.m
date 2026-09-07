function [selectedCandidate, bestIndex, mode] = ...
    select_best_2d_trajectory( ...
        candidates, ...
        results)

    %% =====================================================
    % AUTONEX 2-D MINIMUM-RISK TRAJECTORY SELECTOR
    %% =====================================================

    selectedCandidate = [];

    bestIndex = NaN;

    mode = 'NO_ACTION';


    if isempty(results)
        return;
    end


    %% =====================================================
    % FIRST: SEARCH FULLY SAFE TRAJECTORIES
    %% =====================================================

    safeIndices = [];


    for i = 1:length(results)

        if results(i).safe

            safeIndices(end+1) = i; %#ok<AGROW>

        end

    end


    %% =====================================================
    % SAFE TRAJECTORIES EXIST
    %% =====================================================

    if ~isempty(safeIndices)

        bestScore = inf;


        for n = 1:length(safeIndices)

            i = ...
                safeIndices(n);


            if results(i).score < bestScore

                bestScore = ...
                    results(i).score;

                bestIndex = i;

            end

        end


        selectedCandidate = ...
            candidates(bestIndex);


        mode = ...
            'SAFE';


        return;

    end


    %% =====================================================
    % NO FULLY SAFE TRAJECTORY
    %
    % Select minimum predicted risk.
    %% =====================================================

    bestScore = inf;


    for i = 1:length(results)

        if results(i).score < bestScore

            bestScore = ...
                results(i).score;

            bestIndex = i;

        end

    end


    if ~isnan(bestIndex)

        selectedCandidate = ...
            candidates(bestIndex);


        mode = ...
            'MINIMUM_RISK_FALLBACK';

    end

end