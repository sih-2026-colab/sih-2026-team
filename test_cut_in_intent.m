clear;
clc;
close all;

rng(10);

%% =========================================================
% AUTONEX CUT-IN INTENT PREDICTION TEST
%% =========================================================

actors = ...
    create_highway_scenario();

tracker = ...
    create_autonex_gnn_tracker();

dt = 0.05;
T = 2.5;

egoLaneY = 7.0;


fprintf('\n');
fprintf('============================================================\n');
fprintf(' AutoNex Radar -> GNN -> Cut-In Intent Prediction\n');
fprintf('============================================================\n\n');


for t = 0:dt:T

    %% -----------------------------------------------------
    % CURRENT EGO STATE
    %% -----------------------------------------------------

    ego = actors(1);


    %% -----------------------------------------------------
    % 1. 77-81 GHz RADAR
    %% -----------------------------------------------------

    radarDetections = ...
        scan_autonex_radar_suite(actors);


    %% -----------------------------------------------------
    % 2. RADAR -> OBJECT DETECTIONS
    %% -----------------------------------------------------

    detectionCells = ...
        radar_to_object_detections( ...
            radarDetections, ...
            ego, ...
            t);


    %% -----------------------------------------------------
    % 3. GNN TRACKING
    %% -----------------------------------------------------

    [confirmedTracks, ~, ~] = ...
        tracker( ...
            detectionCells, ...
            t);


    %% -----------------------------------------------------
    % 4. FIND HIGHEST CUT-IN RISK
    %% -----------------------------------------------------

    if ~isempty(confirmedTracks)

        [intent, found] = ...
            find_highest_cutin_risk( ...
                confirmedTracks, ...
                ego, ...
                egoLaneY);


        if found

            if isinf(intent.timeToLaneCrossing)

                crossText = 'INF';

            else

                crossText = ...
                    sprintf( ...
                    '%.2f', ...
                    intent.timeToLaneCrossing);

            end


            fprintf( ...
                't=%4.2f | Track=%d | Y=%5.2f | Vy=%5.2f | LaneCross=%s s | P(CUT-IN)=%5.1f%% | %s\n', ...
                t, ...
                intent.trackID, ...
                intent.y, ...
                intent.vy, ...
                crossText, ...
                intent.probability * 100, ...
                intent.level);

        end

    end


    %% -----------------------------------------------------
    % UPDATE TRUE WORLD
    %% -----------------------------------------------------

    actors = ...
        update_highway_actors( ...
            actors, ...
            dt);

end