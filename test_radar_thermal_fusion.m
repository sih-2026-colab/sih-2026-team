clear;
clc;
close all;

rng(20);

%% =========================================================
% AUTONEX RADAR + THERMAL FUSION TEST
%% =========================================================

actors = ...
    create_highway_scenario();


tracker = ...
    create_autonex_gnn_tracker();


dt = 0.05;

T = 1.5;


fprintf('\n');
fprintf('============================================================\n');
fprintf(' AutoNex 77-81 GHz Radar + Passive Thermal IR Fusion\n');
fprintf('============================================================\n\n');


%% =========================================================
% MAIN LOOP
%% =========================================================

for t = 0:dt:T

    ego = actors(1);


    %% -----------------------------------------------------
    % 1. RADAR SCAN
    %% -----------------------------------------------------

    radarDetections = ...
        scan_autonex_radar_suite(actors);


    %% -----------------------------------------------------
    % 2. RADAR -> OBJECT DETECTIONS
    %% -----------------------------------------------------

    radarObjects = ...
        radar_to_object_detections( ...
            radarDetections, ...
            ego, ...
            t);


    %% -----------------------------------------------------
    % 3. GNN TRACKING
    %% -----------------------------------------------------

    [confirmedTracks,~,~] = ...
        tracker( ...
            radarObjects, ...
            t);


    %% -----------------------------------------------------
    % 4. PASSIVE THERMAL IR
    %% -----------------------------------------------------

    thermalDetections = ...
        simulate_thermal_ir( ...
            ego, ...
            actors);


    %% -----------------------------------------------------
    % 5. ASSOCIATE THERMAL WITH TRACKS
    %% -----------------------------------------------------

    if ~isempty(confirmedTracks) && ...
       ~isempty(thermalDetections)

        associations = ...
            associate_thermal_tracks( ...
                confirmedTracks, ...
                thermalDetections);


        fprintf('\nTIME = %.2f s\n',t);


        %% -------------------------------------------------
        % 6. FUSE EACH ASSOCIATED TRACK
        %% -------------------------------------------------

        for a = 1:length(associations)

            trackID = ...
                associations(a).trackID;


            trackIndex = [];


            %% Find matching track

            for k = 1:length(confirmedTracks)

                if confirmedTracks(k).TrackID == trackID

                    trackIndex = k;

                    break;

                end

            end


            if isempty(trackIndex)

                continue;

            end


            %% Fuse confidence

            fused = ...
                fuse_sensor_confidence( ...
                    confirmedTracks(trackIndex), ...
                    associations(a).thermalConfidence);


            %% Track state

            state = ...
                confirmedTracks(trackIndex).State;


            x = state(1);
            vx = state(2);

            y = state(3);
            vy = state(4);


            %% Print results

            fprintf( ...
                ['Track=%d | X=%6.2f Y=%5.2f | ' ...
                 'Vx=%6.2f Vy=%5.2f | ' ...
                 'Radar=%5.1f%% | Thermal=%5.1f%% | ' ...
                 'FUSED=%5.1f%% | Uncertainty=%.2f | Type=%s\n'], ...
                trackID, ...
                x, ...
                y, ...
                vx, ...
                vy, ...
                fused.radarConfidence * 100, ...
                fused.thermalConfidence * 100, ...
                fused.fusedConfidence * 100, ...
                fused.fusedUncertainty, ...
                associations(a).thermalType);

        end

    end


    %% -----------------------------------------------------
    % UPDATE PHYSICAL WORLD
    %% -----------------------------------------------------

    actors = ...
        update_highway_actors( ...
            actors, ...
            dt);

end