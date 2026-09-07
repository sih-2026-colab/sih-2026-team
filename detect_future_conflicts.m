function conflicts = detect_future_conflicts(actors, predictions, horizons)

    egoIndex = 1;

    numActors = length(actors);
    numHorizons = length(horizons);

    conflicts = struct([]);

    c = 0;

    % Approximate safety envelope
    safeLongitudinal = 6.0;
    safeLateral = 2.2;

    for i = 2:numActors

        for j = 1:numHorizons

            egoX = predictions(egoIndex,j).x;
            egoY = predictions(egoIndex,j).y;

            otherX = predictions(i,j).x;
            otherY = predictions(i,j).y;

            dx = otherX - egoX;
            dy = otherY - egoY;

            longitudinalConflict = ...
                abs(dx) <= safeLongitudinal;

            lateralConflict = ...
                abs(dy) <= safeLateral;

            if longitudinalConflict && lateralConflict

                c = c + 1;

                conflicts(c).actor = actors(i).name;
                conflicts(c).horizon = horizons(j);

                conflicts(c).dx = dx;
                conflicts(c).dy = dy;

                conflicts(c).risk = 'HIGH';

            end

        end

    end

end