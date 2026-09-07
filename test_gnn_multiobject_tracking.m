clear;
clc;
close all;

rng(10);

%% =========================================================
% AUTONEX GNN MULTI-OBJECT RADAR TRACKING
%% =========================================================

actors = create_highway_scenario();

tracker = ...
    create_autonex_gnn_tracker();

dt = 0.05;
T = 3.0;


fprintf('\n');
fprintf('============================================================\n');
fprintf(' AutoNex GNN Multi-Object Radar Tracking\n');
fprintf('============================================================\n\n');


%% =========================================================
% SIMULATION LOOP
%% =========================================================

for t = 0:dt:T

    %% Current ego

    ego = actors(1);


    %% -----------------------------------------------------
    % 1. RADAR SCAN
    %% -----------------------------------------------------

    radarDetections = ...
        scan_autonex_radar_suite(actors);


    %% -----------------------------------------------------
    % 2. CONVERT TO ANONYMOUS OBJECT DETECTIONS
    %% -----------------------------------------------------

    detectionCells = ...
        radar_to_object_detections( ...
            radarDetections, ...
            ego, ...
            t);


    %% -----------------------------------------------------
    % 3. UPDATE GLOBAL NEAREST-NEIGHBOR TRACKER
    %% -----------------------------------------------------

    [confirmedTracks, ...
     tentativeTracks, ...
     allTracks] = ...
        tracker( ...
            detectionCells, ...
            t);


    %% -----------------------------------------------------
    % 4. DISPLAY TRACK INFORMATION
    %% -----------------------------------------------------

    fprintf('\n---------------------------------------------\n');
    fprintf('TIME = %.2f s\n', t);

    fprintf( ...
        'Detections = %d | Confirmed = %d | Tentative = %d\n', ...
        length(detectionCells), ...
        length(confirmedTracks), ...
        length(tentativeTracks));


    if isempty(allTracks)

        fprintf('No tracks yet.\n');

    else

        fprintf('\n');
        fprintf('Track | Confirmed | X     | Y     | Vx     | Vy\n');
        fprintf('------------------------------------------------\n');


        for k = 1:length(allTracks)

            state = ...
                allTracks(k).State;


            % initcvkf 2-D state:
            %
            % [x
            %  vx
            %  y
            %  vy]

            x = state(1);
            vx = state(2);

            y = state(3);
            vy = state(4);


            fprintf( ...
                '%5d | %9d | %5.2f | %5.2f | %6.2f | %6.2f\n', ...
                allTracks(k).TrackID, ...
                allTracks(k).IsConfirmed, ...
                x, ...
                y, ...
                vx, ...
                vy);

        end

    end


    %% -----------------------------------------------------
    % 5. UPDATE TRUE WORLD
    %% -----------------------------------------------------

    actors = ...
        update_highway_actors( ...
            actors, dt);

end