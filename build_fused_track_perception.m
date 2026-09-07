function fusedTracks = ...
    build_fused_track_perception( ...
        tracks, ...
        rgbDetections, ...
        thermalDetections, ...
        environment)

    %% =====================================================
    % AUTONEX MULTIMODAL PER-TRACK FUSION
    %
    % Creates one fused perception record for every
    % confirmed GNN track.
    %% =====================================================

    fusedTracks = struct([]);


    if isempty(tracks)
        return;
    end


    %% =====================================================
    % SENSOR ASSOCIATIONS
    %% =====================================================

    thermalAssociations = ...
        associate_thermal_tracks( ...
            tracks, ...
            thermalDetections);


    rgbAssociations = ...
        associate_rgb_tracks( ...
            tracks, ...
            rgbDetections);


    %% =====================================================
    % PROCESS EVERY TRACK
    %% =====================================================

    for k = 1:length(tracks)

        trackID = ...
            tracks(k).TrackID;


        %% Default = sensor unavailable

        thermalConfidence = NaN;

        rgbConfidence = NaN;


        %% -------------------------------------------------
        % FIND THERMAL MATCH
        %% -------------------------------------------------

        for a = 1:length(thermalAssociations)

            if thermalAssociations(a).trackID == trackID

                thermalConfidence = ...
                    thermalAssociations(a).thermalConfidence;

                break;

            end

        end


        %% -------------------------------------------------
        % FIND RGB MATCH
        %% -------------------------------------------------

        for a = 1:length(rgbAssociations)

            if rgbAssociations(a).trackID == trackID

                rgbConfidence = ...
                    rgbAssociations(a).rgbConfidence;

                break;

            end

        end


        %% -------------------------------------------------
        % ADAPTIVE SENSOR FUSION
        %% -------------------------------------------------

        fused = ...
            adaptive_multimodal_fusion( ...
                tracks(k), ...
                rgbConfidence, ...
                thermalConfidence, ...
                environment);


        %% -------------------------------------------------
        % SAVE
        %% -------------------------------------------------

        fusedTracks(k).trackID = ...
            trackID;


        fusedTracks(k).fusedConfidence = ...
            fused.fusedConfidence;


        fusedTracks(k).fusedUncertainty = ...
            fused.fusedUncertainty;


        fusedTracks(k).radarConfidence = ...
            fused.radarConfidence;


        fusedTracks(k).rgbConfidence = ...
            fused.rgbConfidence;


        fusedTracks(k).thermalConfidence = ...
            fused.thermalConfidence;


        fusedTracks(k).radarWeight = ...
            fused.radarWeight;


        fusedTracks(k).rgbWeight = ...
            fused.rgbWeight;


        fusedTracks(k).thermalWeight = ...
            fused.thermalWeight;


        if isfield(fused,'numSensors')

    fusedTracks(k).numSensors = ...
        fused.numSensors;

else

    %% Fallback for compatibility

    fusedTracks(k).numSensors = ...
        1 + ...
        ~isnan(rgbConfidence) + ...
        ~isnan(thermalConfidence);

    end

    end

end