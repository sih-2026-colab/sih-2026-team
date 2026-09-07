clear;
clc;
close all;

rng(10);

%% =========================================================
% AUTONEX FULL SENSOR -> REFLEX PIPELINE
%% =========================================================

actors = ...
    create_highway_scenario();


tracker = ...
    create_autonex_gnn_tracker();


dt = 0.05;
T = 6.0;

horizons = ...
    [0.5 1.0 1.5 2.0];

speedsKmh = ...
    generate_candidate_speeds();

egoLaneY = 7.0;


%% =========================================================
% FIGURE
%% =========================================================

figure( ...
    'Name', ...
    'AutoNex Sensor-Driven Cat Reflex');


%% =========================================================
% MAIN LOOP
%% =========================================================

for t = 0:dt:T

    %% -----------------------------------------------------
    % 1. CURRENT EGO STATE
    %% -----------------------------------------------------

    ego = actors(1);

    currentSpeedKmh = ...
        ego.vx * 3.6;


    %% -----------------------------------------------------
    % 2. 77–81 GHz RADAR SUITE
    %% -----------------------------------------------------

    radarDetections = ...
        scan_autonex_radar_suite(actors);


    %% -----------------------------------------------------
    % 3. RADAR -> ANONYMOUS DETECTIONS
    %% -----------------------------------------------------

    detectionCells = ...
        radar_to_object_detections( ...
            radarDetections, ...
            ego, ...
            t);


    %% -----------------------------------------------------
    % 4. GNN MULTI-OBJECT TRACKING
    %% -----------------------------------------------------

    [confirmedTracks,~,~] = ...
        tracker( ...
            detectionCells, ...
            t);


    %% -----------------------------------------------------
    % 5. CUT-IN INTENT
    %% -----------------------------------------------------

    intent = [];

    intentFound = false;


    if ~isempty(confirmedTracks)

        [intent,intentFound] = ...
            find_highest_cutin_risk( ...
                confirmedTracks, ...
                ego, ...
                egoLaneY);

    end


    %% -----------------------------------------------------
    % 6. SENSOR-BASED SPEED EVALUATION
    %% -----------------------------------------------------

    if isempty(confirmedTracks)

        % No confirmed tracks yet:
        % maintain normal cruise.

        selectedSpeed = 75;

        bestIndex = NaN;

        results = [];

    else

        results = ...
            evaluate_tracked_speeds( ...
                confirmedTracks, ...
                ego, ...
                horizons, ...
                speedsKmh, ...
                intent);


        [selectedSpeed,bestIndex] = ...
            select_minimum_risk_speed( ...
                results);

    end


    %% -----------------------------------------------------
    % 7. CAT REFLEX
    %% -----------------------------------------------------

    command = ...
        cat_reflex_guardian( ...
            currentSpeedKmh, ...
            selectedSpeed);


    %% No safe candidate

    if isnan(selectedSpeed)

        selectedSpeed = 0;

        command = ...
            'EMERGENCY_BRAKE';

    end


    %% -----------------------------------------------------
    % 8. SPEED CONTROLLER
    %% -----------------------------------------------------

    acceleration = ...
        speed_tracking_controller( ...
            currentSpeedKmh, ...
            selectedSpeed, ...
            command);


    %% Apply controller to true simulated ego
    %
    % IMPORTANT:
    % True actor data is only used here as our
    % simulated physical world.
    %
    % Decision was made from radar/GNN tracks.

    actors(1).ax = ...
        acceleration;


    %% -----------------------------------------------------
    % 9. DASHBOARD VARIABLES
    %% -----------------------------------------------------

    if intentFound

        intentProbability = ...
            intent.probability * 100;

        intentLevel = ...
            intent.level;

    else

        intentProbability = 0;

        intentLevel = 'NONE';

    end


    %% Rear TTC from selected candidate

    if ~isnan(bestIndex) && ...
       ~isempty(results)

        selectedRearTTC = ...
            results(bestIndex).rearTTC;

        selectedConflicts = ...
            results(bestIndex).frontConflicts;

    else

        selectedRearTTC = inf;

        selectedConflicts = 0;

    end


    %% =====================================================
    % 10. VISUALIZATION
    %% =====================================================

    clf;
    hold on;
    grid on;


    ego = actors(1);


    %% Road

    plot( ...
        [ego.x-35 ego.x+110], ...
        [-1.75 -1.75], ...
        'k', ...
        'LineWidth',2);


    plot( ...
        [ego.x-35 ego.x+110], ...
        [8.75 8.75], ...
        'k', ...
        'LineWidth',2);


    plot( ...
        [ego.x-35 ego.x+110], ...
        [1.75 1.75], ...
        'k--');


    plot( ...
        [ego.x-35 ego.x+110], ...
        [5.25 5.25], ...
        'k--');


    %% -----------------------------------------------------
    % TRUE ACTORS — SIMULATION WORLD
    %% -----------------------------------------------------

    for i = 1:length(actors)

        plot( ...
            actors(i).x, ...
            actors(i).y, ...
            's', ...
            'MarkerSize',12, ...
            'LineWidth',2);


        text( ...
            actors(i).x + 1, ...
            actors(i).y + 0.25, ...
            actors(i).name);

    end


    %% -----------------------------------------------------
    % GNN TRACK ESTIMATES
    %% -----------------------------------------------------

    for k = 1:length(confirmedTracks)

        state = ...
            confirmedTracks(k).State;


        trackX = state(1);
        trackY = state(3);


        plot( ...
            trackX, ...
            trackY, ...
            'x', ...
            'MarkerSize',12, ...
            'LineWidth',2);


        text( ...
            trackX + 1, ...
            trackY - 0.35, ...
            sprintf( ...
            'Track %d', ...
            confirmedTracks(k).TrackID));

    end


    %% -----------------------------------------------------
    % DASHBOARD
    %% -----------------------------------------------------

    dashX = ...
        ego.x - 25;


    text( ...
        dashX, ...
        8.25, ...
        sprintf( ...
        'Time %.2f s',t));


    text( ...
        dashX + 20, ...
        8.25, ...
        sprintf( ...
        'EGO %.2f km/h', ...
        currentSpeedKmh));


    text( ...
        dashX + 50, ...
        8.25, ...
        sprintf( ...
        'Target %.1f km/h', ...
        selectedSpeed));


    text( ...
        dashX + 78, ...
        8.25, ...
        sprintf( ...
        'Accel %.2f m/s^2', ...
        acceleration));


    text( ...
        ego.x - 15, ...
        -0.25, ...
        sprintf( ...
        'P(CUT-IN): %.1f%%  [%s]', ...
        intentProbability, ...
        intentLevel), ...
        'FontWeight','bold');


    text( ...
        ego.x - 15, ...
        -1.0, ...
        sprintf( ...
        'CAT REFLEX: %s', ...
        command), ...
        'FontWeight','bold');


    %% Rear TTC

    if isinf(selectedRearTTC)

        rearText = 'INF';

    else

        rearText = ...
            sprintf( ...
            '%.2f', ...
            selectedRearTTC);

    end


    text( ...
        ego.x + 35, ...
        -0.25, ...
        sprintf( ...
        'Rear TTC: %s s', ...
        rearText));


    text( ...
        ego.x + 35, ...
        -1.0, ...
        sprintf( ...
        'Future conflicts: %d', ...
        selectedConflicts));


    %% -----------------------------------------------------
    % VIEW
    %% -----------------------------------------------------

    xlabel( ...
        'Longitudinal Position X (m)');

    ylabel( ...
        'Lateral Position Y (m)');


    title( ...
        'AutoNex — 77–81 GHz Radar + GNN + Intent + Cat Reflex');


    xlim( ...
        [ego.x-30 ego.x+100]);

    ylim([-2.5 9.5]);


    drawnow;


    %% =====================================================
    % 11. UPDATE PHYSICAL WORLD
    %% =====================================================

    actors = ...
        update_highway_actors( ...
            actors, ...
            dt);


    if actors(1).vx < 0

        actors(1).vx = 0;

    end


    pause(0.01);

end