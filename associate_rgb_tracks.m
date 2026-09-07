function associations = ...
    associate_rgb_tracks(tracks, rgbDetections)

    %% =====================================================
    % AUTONEX GNN TRACK <-> RGB ASSOCIATION
    %% =====================================================

    associations = struct([]);

    count = 0;

    maxAssociationDistance = 3.5;


    for k = 1:length(tracks)

        state = tracks(k).State;

        trackX = state(1);
        trackY = state(3);

        bestDistance = inf;
        bestRGBIndex = NaN;


        for j = 1:length(rgbDetections)

            dx = ...
                rgbDetections(j).x - trackX;

            dy = ...
                rgbDetections(j).y - trackY;

            distance = ...
                sqrt(dx^2 + dy^2);


            if distance < bestDistance

                bestDistance = distance;
                bestRGBIndex = j;

            end

        end


        if ~isnan(bestRGBIndex) && ...
           bestDistance <= maxAssociationDistance

            count = count + 1;

            associations(count).trackID = ...
                tracks(k).TrackID;

            associations(count).rgbIndex = ...
                bestRGBIndex;

            associations(count).distance = ...
                bestDistance;

            associations(count).rgbConfidence = ...
                rgbDetections(bestRGBIndex).confidence;

            associations(count).rgbType = ...
                rgbDetections(bestRGBIndex).type;

        end

    end

end