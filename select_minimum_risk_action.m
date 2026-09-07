function [selectedSpeed, bestIndex, mode] = ...
    select_minimum_risk_action(results)

    %% =====================================================
    % AUTONEX MINIMUM-RISK ACTION SELECTOR
    %
    % Priority:
    %
    % 1. Choose safest lowest-score candidate
    % 2. If NO candidate is fully safe,
    %    choose lowest-risk fallback candidate
    %% =====================================================

    selectedSpeed = NaN;

    bestIndex = NaN;

    mode = 'NO_ACTION';


    if isempty(results)
        return;
    end


    %% =====================================================
    % FIRST SEARCH ONLY SAFE CANDIDATES
    %% =====================================================

    safeIndices = [];


    for i = 1:length(results)

        if results(i).safe

            safeIndices(end+1) = i; %#ok<AGROW>

        end

    end


    %% =====================================================
    % SAFE CANDIDATES EXIST
    %% =====================================================

    if ~isempty(safeIndices)

        bestScore = inf;


        for n = 1:length(safeIndices)

            i = safeIndices(n);


            if results(i).score < bestScore

                bestScore = ...
                    results(i).score;

                bestIndex = i;

            end

        end


        selectedSpeed = ...
            results(bestIndex).speedKmh;


        mode = ...
            'SAFE';


        return;

    end


    %% =====================================================
    % NO FULLY SAFE CANDIDATE
    %
    % Select the candidate with minimum total risk.
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

        selectedSpeed = ...
            results(bestIndex).speedKmh;


        mode = ...
            'MINIMUM_RISK_FALLBACK';

    end

end