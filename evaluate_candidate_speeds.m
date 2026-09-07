function results = evaluate_candidate_speeds(actors, horizons, speedsKmh)

    predictions = ...
        predict_actor_trajectories(actors, horizons);

    ego = actors(1);

    safeLongitudinal = 6.0;
    safeLateral = 2.2;

    numSpeeds = length(speedsKmh);
    numHorizons = length(horizons);

    results = struct([]);

    for s = 1:numSpeeds

        candidateSpeed = speedsKmh(s) / 3.6;

        conflictCount = 0;

        minClearance = inf;

        for j = 1:numHorizons

            h = horizons(j);

            % Candidate ego future position
            egoFutureX = ...
                ego.x + candidateSpeed * h;

            egoFutureY = ego.y;

            % Compare candidate ego trajectory
            % against every other actor
            for i = 2:length(actors)

                otherX = predictions(i,j).x;
                otherY = predictions(i,j).y;

                dx = otherX - egoFutureX;
                dy = otherY - egoFutureY;

                clearance = sqrt(dx^2 + dy^2);

                if clearance < minClearance
                    minClearance = clearance;
                end

                longitudinalConflict = ...
                    abs(dx) <= safeLongitudinal;

                lateralConflict = ...
                    abs(dy) <= safeLateral;

                if longitudinalConflict && lateralConflict

                    conflictCount = ...
                        conflictCount + 1;

                end

            end

        end

        results(s).speedKmh = speedsKmh(s);
        results(s).conflicts = conflictCount;
        results(s).minClearance = minClearance;

        results(s).safe = ...
            conflictCount == 0;

    end

end