clear;
clc;
close all;

rng(60);

%% =========================================================
% AUTONEX 2-D MINIMUM-RISK TRAJECTORY PLANNER
%% =========================================================

actors = ...
    create_highway_scenario();


tracker = ...
    create_autonex_gnn_tracker();


%% DAY ENVIRONMENT

environment.lightLevel = 1.0;

environment.visibility = 1.0;

environment.radarQuality = 0.95;

environment.thermalQuality = 0.85;


dt = 0.05;

planningTime = 1.0;

egoLaneY = 7.0;


%% =========================================================
% BUILD SENSOR/TRACK HISTORY UNTIL PLANNING TIME
%% =========================================================

numSteps = ...
    round(planningTime / dt);


confirmedTracks = [];

fusedTracks = [];

intent = [];


for step = 0:numSteps

    t = ...
        step * dt;


    ego = ...
        actors(1);


    %% Radar

    radar = ...
        scan_autonex_radar_suite(actors);


    radarObjects = ...
        radar_to_object_detections( ...
            radar, ...
            ego, ...
            t);


    %% GNN

    [confirmedTracks,~,~] = ...
        tracker( ...
            radarObjects, ...
            t);


    %% RGB

    rgb = ...
        simulate_rgb_camera( ...
            ego, ...
            actors, ...
            environment);


    %% Thermal

    thermal = ...
        simulate_thermal_ir( ...
            ego, ...
            actors);


    %% Fusion

    fusedTracks = ...
        build_fused_track_perception( ...
            confirmedTracks, ...
            rgb, ...
            thermal, ...
            environment);


    %% Cut-in intent

    intent = [];


    if ~isempty(confirmedTracks)

        [tempIntent,found] = ...
            find_highest_cutin_risk( ...
                confirmedTracks, ...
                ego, ...
                egoLaneY);


        if found

            intent = ...
                tempIntent;

        end

    end


    %% Advance world except after final sensor scan

    if step < numSteps

        actors = ...
            update_highway_actors( ...
                actors, ...
                dt);

    end

end


%% =========================================================
% CURRENT EGO AT PLANNING TIME
%% =========================================================

ego = ...
    actors(1);


%% =========================================================
% GENERATE 2-D CANDIDATES
%
% Do not include t=0 in risk evaluation.
%% =========================================================

planningHorizons = ...
    0.25:0.25:3.0;


candidates = ...
    generate_2d_candidate_set( ...
        ego, ...
        planningHorizons);


%% Road edges

roadMinY = -1.75;

roadMaxY = 8.75;


%% =========================================================
% EVALUATE ALL CANDIDATES
%% =========================================================

results = ...
    evaluate_2d_trajectories( ...
        candidates, ...
        confirmedTracks, ...
        fusedTracks, ...
        ego, ...
        intent, ...
        roadMinY, ...
        roadMaxY);


%% =========================================================
% SELECT BEST
%% =========================================================

[selectedCandidate, ...
 bestIndex, ...
 selectionMode] = ...
    select_best_2d_trajectory( ...
        candidates, ...
        results);


%% =========================================================
% PRINT RESULTS
%% =========================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' AutoNex 2-D Minimum-Risk Trajectory Planner\n');
fprintf('============================================================\n\n');


if isempty(intent)

    fprintf('Cut-In Probability : NONE\n\n');

else

    fprintf( ...
        'Cut-In Probability : %.1f%%\n\n', ...
        intent.probability * 100);

end


fprintf( ...
    'ID | Maneuver           | Speed | TargetY | State  | F | R | S | Score\n');

fprintf( ...
    '------------------------------------------------------------------------\n');


for i = 1:length(results)

    if results(i).safe

        stateText = ...
            'SAFE';

    else

        stateText = ...
            'UNSAFE';

    end


    fprintf( ...
        '%2d | %-18s | %5.1f | %7.2f | %-6s | %1d | %1d | %1d | %8.1f\n', ...
        results(i).candidateID, ...
        results(i).name, ...
        results(i).targetSpeedKmh, ...
        results(i).targetY, ...
        stateText, ...
        results(i).frontConflicts, ...
        results(i).rearConflicts, ...
        results(i).sideConflicts, ...
        results(i).score);

end


%% =========================================================
% SELECTED ACTION
%% =========================================================

fprintf('\n============================================================\n');
fprintf(' AUTONEX SELECTED TRAJECTORY\n');
fprintf('============================================================\n');


if isempty(selectedCandidate)

    fprintf('NO TRAJECTORY AVAILABLE\n');

else

    fprintf( ...
        'Mode         : %s\n', ...
        selectionMode);


    fprintf( ...
        'Candidate ID : %d\n', ...
        selectedCandidate.id);


    fprintf( ...
        'Maneuver     : %s\n', ...
        selectedCandidate.name);


    fprintf( ...
        'Target Speed : %.1f km/h\n', ...
        selectedCandidate.targetSpeedKmh);


    fprintf( ...
        'Target Y     : %.2f m\n', ...
        selectedCandidate.targetY);


    fprintf( ...
        'Risk Score   : %.2f\n', ...
        results(bestIndex).score);


    fprintf( ...
        'Front Risk   : %d\n', ...
        results(bestIndex).frontConflicts);


    fprintf( ...
        'Rear Risk    : %d\n', ...
        results(bestIndex).rearConflicts);


    fprintf( ...
        'Side Risk    : %d\n', ...
        results(bestIndex).sideConflicts);

end


%% =========================================================
% VISUALIZE
%% =========================================================

figure( ...
    'Name', ...
    'AutoNex 2-D Minimum-Risk Planner');


hold on;
grid on;


%% Road

plot( ...
    [ego.x ego.x+80], ...
    [roadMinY roadMinY], ...
    'k', ...
    'LineWidth',2);


plot( ...
    [ego.x ego.x+80], ...
    [roadMaxY roadMaxY], ...
    'k', ...
    'LineWidth',2);


plot( ...
    [ego.x ego.x+80], ...
    [1.75 1.75], ...
    'k--');


plot( ...
    [ego.x ego.x+80], ...
    [5.25 5.25], ...
    'k--');


%% All candidates

for i = 1:length(candidates)

    plot( ...
        candidates(i).trajectory.x, ...
        candidates(i).trajectory.y, ...
        'LineWidth',0.8);

end


%% Selected trajectory

if ~isempty(selectedCandidate)

    plot( ...
        selectedCandidate.trajectory.x, ...
        selectedCandidate.trajectory.y, ...
        'LineWidth',4);

end


%% Ego

plot( ...
    ego.x, ...
    ego.y, ...
    's', ...
    'MarkerSize',12, ...
    'LineWidth',3);


%% Tracked objects

for k = 1:length(confirmedTracks)

    state = ...
        confirmedTracks(k).State;


    plot( ...
        state(1), ...
        state(3), ...
        'x', ...
        'MarkerSize',12, ...
        'LineWidth',2);


    text( ...
        state(1)+0.5, ...
        state(3)+0.2, ...
        sprintf( ...
            'Track %d', ...
            confirmedTracks(k).TrackID));

end


xlabel('Longitudinal X (m)');

ylabel('Lateral Y (m)');


title( ...
    'AutoNex — 2-D Uncertainty-Aware Minimum-Risk Trajectory Selection');


xlim( ...
    [ego.x ego.x+80]);


ylim( ...
    [-2 9]);


hold off;