clear;
clc;
close all;

rng(40);


%% =========================================================
% AUTONEX ADAPTIVE MULTIMODAL SENSOR FUSION
%% =========================================================

conditions = {
    'DAY'
    'NIGHT'
};


for conditionIndex = 1:2

    %% New world and tracker for each condition

    actors = ...
        create_highway_scenario();

    tracker = ...
        create_autonex_gnn_tracker();


    %% =====================================================
    % ENVIRONMENT
    %% =====================================================

    if conditionIndex == 1

        environment.lightLevel = 1.0;
        environment.visibility = 1.0;

        environment.radarQuality = 0.95;
        environment.thermalQuality = 0.85;

    else

        environment.lightLevel = 0.15;
        environment.visibility = 0.90;

        environment.radarQuality = 0.95;
        environment.thermalQuality = 0.95;

    end


    fprintf('\n');
    fprintf('============================================================\n');
    fprintf(' AUTONEX MULTIMODAL FUSION — %s\n', ...
        conditions{conditionIndex});
    fprintf('============================================================\n');


    dt = 0.05;

    T = 0.8;


    for t = 0:dt:T

        ego = actors(1);


        %% -------------------------------------------------
        % RADAR
        %% -------------------------------------------------

        radar = ...
            scan_autonex_radar_suite(actors);


        radarObjects = ...
            radar_to_object_detections( ...
                radar, ...
                ego, ...
                t);


        [confirmedTracks,~,~] = ...
            tracker( ...
                radarObjects, ...
                t);


        %% -------------------------------------------------
        % THERMAL
        %% -------------------------------------------------

        thermal = ...
            simulate_thermal_ir( ...
                ego, ...
                actors);


        %% -------------------------------------------------
        % RGB
        %% -------------------------------------------------

        rgb = ...
            simulate_rgb_camera( ...
                ego, ...
                actors, ...
                environment);


        %% -------------------------------------------------
        % ASSOCIATION
        %% -------------------------------------------------

        if ~isempty(confirmedTracks)

            thermalAssociations = ...
                associate_thermal_tracks( ...
                    confirmedTracks, ...
                    thermal);


            rgbAssociations = ...
                associate_rgb_tracks( ...
                    confirmedTracks, ...
                    rgb);


            %% =============================================
            % FIND TRACK 1
            %
            % Only for visualization/test here.
            %% =============================================

            for k = 1:length(confirmedTracks)

                currentTrack = ...
                    confirmedTracks(k);

                trackID = ...
                    currentTrack.TrackID;


                thermalConfidence = NaN;
                rgbConfidence = NaN;


                %% Find thermal match

                for a = 1:length(thermalAssociations)

                    if thermalAssociations(a).trackID == trackID

                        thermalConfidence = ...
                            thermalAssociations(a).thermalConfidence;

                        break;

                    end

                end


                %% Find RGB match

                for a = 1:length(rgbAssociations)

                    if rgbAssociations(a).trackID == trackID

                        rgbConfidence = ...
                            rgbAssociations(a).rgbConfidence;

                        break;

                    end

                end


                %% Need all three sensors for this test

                if isnan(thermalConfidence) || ...
                   isnan(rgbConfidence)

                    continue;

                end


                %% -----------------------------------------
                % ADAPTIVE FUSION
                %% -----------------------------------------

                fused = ...
                    adaptive_multimodal_fusion( ...
                        currentTrack, ...
                        rgbConfidence, ...
                        thermalConfidence, ...
                        environment);


                %% Print Track 1 only for clean output

                if trackID == 1

                    fprintf( ...
                        ['t=%4.2f | ' ...
                         'Radar=%5.1f%% RGB=%5.1f%% Thermal=%5.1f%% | ' ...
                         'Weights R=%.2f C=%.2f T=%.2f | ' ...
                         'FUSED=%5.1f%% | Unc=%.2f\n'], ...
                        t, ...
                        fused.radarConfidence * 100, ...
                        fused.rgbConfidence * 100, ...
                        fused.thermalConfidence * 100, ...
                        fused.radarWeight, ...
                        fused.rgbWeight, ...
                        fused.thermalWeight, ...
                        fused.fusedConfidence * 100, ...
                        fused.fusedUncertainty);

                end

            end

        end


        actors = ...
            update_highway_actors( ...
                actors, ...
                dt);

    end

end