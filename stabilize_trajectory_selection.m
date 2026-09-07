function [selectedCandidate, ...
          selectedIndex, ...
          selectionMode, ...
          state, ...
          stabilityMode] = ...
    stabilize_trajectory_selection( ...
        candidates, ...
        results, ...
        proposedCandidate, ...
        proposedIndex, ...
        proposedMode, ...
        state, ...
        currentTime)

    %% =====================================================
    % AUTONEX TRAJECTORY HYSTERESIS / MANEUVER COMMITMENT
    %
    % Prevents rapid:
    %
    % KEEP -> MICRO -> KEEP -> MICRO
    %
    % because of small sensor fluctuations.
    %
    % Safety improvements are NEVER blocked.
    %% =====================================================


    %% Configuration

    minimumHoldTime = 0.50;       % seconds

    minimumScoreImprovement = 5;

    relativeImprovement = 0.15;


    %% Default outputs

    selectedCandidate = ...
        proposedCandidate;

    selectedIndex = ...
        proposedIndex;

    selectionMode = ...
        proposedMode;

    stabilityMode = ...
        'PROPOSED';


    %% =====================================================
    % INITIALIZE STATE
    %% =====================================================

    if isempty(state) || ...
       ~isfield(state,'initialized') || ...
       ~state.initialized

        if isempty(proposedCandidate)

            state.initialized = false;

            return;

        end


        state.initialized = true;

        state.name = ...
            proposedCandidate.name;

        state.targetSpeedKmh = ...
            proposedCandidate.targetSpeedKmh;

        state.lastSwitchTime = ...
            currentTime;


        stabilityMode = ...
            'INITIALIZED';

        return;

    end


    %% =====================================================
    % FIND PREVIOUS MANEUVER IN CURRENT CANDIDATE SET
    %
    % Important:
    % We use the CURRENT generation of the trajectory,
    % not an old stored path.
    %% =====================================================

    previousIndex = NaN;


    for i = 1:length(candidates)

        sameName = ...
            strcmp( ...
                candidates(i).name, ...
                state.name);


        sameSpeed = ...
            abs( ...
                candidates(i).targetSpeedKmh - ...
                state.targetSpeedKmh) < 0.01;


        if sameName && sameSpeed

            previousIndex = i;

            break;

        end

    end


    %% Previous maneuver no longer exists

    if isnan(previousIndex)

        if ~isempty(proposedCandidate)

            state.name = ...
                proposedCandidate.name;

            state.targetSpeedKmh = ...
                proposedCandidate.targetSpeedKmh;

            state.lastSwitchTime = ...
                currentTime;

        end


        stabilityMode = ...
            'SWITCHED';

        return;

    end


    %% =====================================================
    % CURRENT VERSION OF PREVIOUS CANDIDATE
    %% =====================================================

    previousCandidate = ...
        candidates(previousIndex);


    previousResult = ...
        results(previousIndex);


    %% =====================================================
    % NO PROPOSAL
    %% =====================================================

    if isempty(proposedCandidate)

        selectedCandidate = ...
            previousCandidate;

        selectedIndex = ...
            previousIndex;


        if previousResult.safe

            selectionMode = 'SAFE';

        else

            selectionMode = ...
                'MINIMUM_RISK_FALLBACK';

        end


        stabilityMode = ...
            'HELD_NO_PROPOSAL';

        return;

    end


    proposedResult = ...
        results(proposedIndex);


    %% =====================================================
    % SAME MANEUVER ALREADY
    %% =====================================================

    sameManeuver = ...
        strcmp( ...
            proposedCandidate.name, ...
            previousCandidate.name) && ...
        abs( ...
            proposedCandidate.targetSpeedKmh - ...
            previousCandidate.targetSpeedKmh) < 0.01;


    if sameManeuver

        selectedCandidate = ...
            proposedCandidate;

        selectedIndex = ...
            proposedIndex;

        selectionMode = ...
            proposedMode;

        stabilityMode = ...
            'UNCHANGED';

        return;

    end


    %% =====================================================
    % SAFETY OVERRIDE #1
    %
    % New path SAFE, previous path UNSAFE:
    % switch immediately.
    %% =====================================================

    if proposedResult.safe && ...
       ~previousResult.safe

        selectedCandidate = ...
            proposedCandidate;

        selectedIndex = ...
            proposedIndex;

        selectionMode = ...
            proposedMode;


        state.name = ...
            proposedCandidate.name;

        state.targetSpeedKmh = ...
            proposedCandidate.targetSpeedKmh;

        state.lastSwitchTime = ...
            currentTime;


        stabilityMode = ...
            'SAFETY_SWITCH';

        return;

    end


    %% =====================================================
    % SAFETY OVERRIDE #2
    %
    % Previous is SAFE but proposal is UNSAFE:
    % keep previous.
    %% =====================================================

    if previousResult.safe && ...
       ~proposedResult.safe

        selectedCandidate = ...
            previousCandidate;

        selectedIndex = ...
            previousIndex;

        selectionMode = ...
            'SAFE';

        stabilityMode = ...
            'HELD_SAFE_PATH';

        return;

    end


    %% =====================================================
    % SAFETY OVERRIDE #3
    %
    % Both unsafe, but proposed candidate has fewer
    % predicted conflicts.
    %% =====================================================

    if ~previousResult.safe && ...
       ~proposedResult.safe && ...
       proposedResult.conflictCount < ...
       previousResult.conflictCount

        selectedCandidate = ...
            proposedCandidate;

        selectedIndex = ...
            proposedIndex;

        selectionMode = ...
            proposedMode;


        state.name = ...
            proposedCandidate.name;

        state.targetSpeedKmh = ...
            proposedCandidate.targetSpeedKmh;

        state.lastSwitchTime = ...
            currentTime;


        stabilityMode = ...
            'RISK_REDUCTION_SWITCH';

        return;

    end


    %% =====================================================
    % MINIMUM COMMITMENT PERIOD
    %% =====================================================

    timeSinceSwitch = ...
        currentTime - ...
        state.lastSwitchTime;


    if timeSinceSwitch < minimumHoldTime

        selectedCandidate = ...
            previousCandidate;

        selectedIndex = ...
            previousIndex;


        if previousResult.safe

            selectionMode = ...
                'SAFE';

        else

            selectionMode = ...
                'MINIMUM_RISK_FALLBACK';

        end


        stabilityMode = ...
            'HELD_COMMITMENT';

        return;

    end


    %% =====================================================
    % SCORE IMPROVEMENT TEST
    %% =====================================================

    scoreImprovement = ...
        previousResult.score - ...
        proposedResult.score;


    requiredImprovement = ...
        max( ...
            minimumScoreImprovement, ...
            relativeImprovement * ...
            abs(previousResult.score));


    %% =====================================================
    % SWITCH ONLY IF MEANINGFULLY BETTER
    %% =====================================================

    if scoreImprovement > requiredImprovement

        selectedCandidate = ...
            proposedCandidate;

        selectedIndex = ...
            proposedIndex;

        selectionMode = ...
            proposedMode;


        state.name = ...
            proposedCandidate.name;

        state.targetSpeedKmh = ...
            proposedCandidate.targetSpeedKmh;

        state.lastSwitchTime = ...
            currentTime;


        stabilityMode = ...
            'BETTER_PATH_SWITCH';


    else

        selectedCandidate = ...
            previousCandidate;

        selectedIndex = ...
            previousIndex;


        if previousResult.safe

            selectionMode = ...
                'SAFE';

        else

            selectionMode = ...
                'MINIMUM_RISK_FALLBACK';

        end


        stabilityMode = ...
            'HELD_HYSTERESIS';

    end

end