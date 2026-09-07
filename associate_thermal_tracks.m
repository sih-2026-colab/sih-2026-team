function associations = ...
    associate_thermal_tracks(tracks, thermalDetections)

    %% =====================================================
    % AUTONEX RADAR/GNN <-> THERMAL ASSOCIATION
    %
    % Match thermal detections with GNN tracks
    % using spatial proximity.
    %% =====================================================

    associations = struct([]);

    count = 0;

    % Maximum allowed distance between
    % track position and thermal detection.
    maxAssociationDistance = 3.0;


    %% =====================================================
    % CHECK EVERY TRACK
    %% =====================================================

    for k = 1:length(tracks)

        state = tracks(k).State;

        trackX = state(1);
        trackY = state(3);


        bestDistance = inf;
        bestThermalIndex = NaN;


        %% -------------------------------------------------
        % FIND CLOSEST THERMAL DETECTION
        %% -------------------------------------------------

        for j = 1:length(thermalDetections)

            dx = ...
                thermalDetections(j).x - trackX;

            dy = ...
                thermalDetections(j).y - trackY;


            distance = ...
                sqrt(dx^2 + dy^2);


            if distance < bestDistance

                bestDistance = distance;

                bestThermalIndex = j;

            end

        end


        %% -------------------------------------------------
        % APPLY ASSOCIATION GATE
        %% -------------------------------------------------

        if ~isnan(bestThermalIndex) && ...
           bestDistance <= maxAssociationDistance

            count = count + 1;


            associations(count).trackID = ...
                tracks(k).TrackID;


            associations(count).thermalIndex = ...
                bestThermalIndex;


            associations(count).distance = ...
                bestDistance;


            associations(count).thermalConfidence = ...
                thermalDetections( ...
                bestThermalIndex).confidence;


            associations(count).thermalType = ...
                thermalDetections( ...
                bestThermalIndex).type;

        end

    end

end