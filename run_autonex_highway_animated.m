function run_autonex_highway_animated(projectFolder, exportVideo)
% Live 3-D presentation of the existing 2-D closed-loop planner.
% Pass the folder containing the existing AutoNex MATLAB functions.
if nargin < 1
    projectFolder = fileparts(mfilename('fullpath'));
end
if nargin < 2, exportVideo = true; end
assert(isfolder(projectFolder), 'AutoNex:Folder', 'Project folder does not exist.');
oldPath = path;
pathCleanup = onCleanup(@() path(oldPath)); %#ok<NASGU>
addpath(projectFolder);
clc;
% Existing figures remain open.

rng(60);

%% =========================================================
% AUTONEX FULL CLOSED-LOOP 2-D PLANNER
%% =========================================================

actors = ...
    create_highway_scenario();

tracker = ...
    create_autonex_gnn_tracker();


%% =========================================================
% ENVIRONMENT
%% =========================================================

environment.lightLevel = 1.0;
environment.visibility = 1.0;
environment.radarQuality = 0.95;
environment.thermalQuality = 0.85;


%% =========================================================
% SETTINGS
%% =========================================================

dt = 0.05;
T = 6.0;

planningHorizons = ...
    0.25:0.25:3.0;

egoLaneY = 7.0;

roadMinY = -1.75;
roadMaxY = 8.75;


%% =========================================================
% FIGURE
%% =========================================================

scene = autonex_animation_scene('create', actors, roadMinY, roadMaxY);
videoCleanup = []; %#ok<NASGU>
if exportVideo
    videoPath = fullfile(fileparts(mfilename('fullpath')), 'autonex_driving_animation.mp4');
    video = VideoWriter(videoPath, 'MPEG-4');
    video.FrameRate = 1 / dt;
    video.Quality = 95;
    open(video);
    videoCleanup = onCleanup(@() close(video));
end

%% =========================================================
% MANEUVER HYSTERESIS STATE
%% =========================================================

selectionState = [];


%% =========================================================
% MAIN LOOP
%% =========================================================

for t = 0:dt:T
    frameClock = tic;
    if ~isgraphics(scene.figure), break; end

    %% =====================================================
    % CURRENT EGO
    %% =====================================================

    ego = ...
        actors(1);

    currentSpeedKmh = ...
        ego.vx * 3.6;


    %% =====================================================
    % 1. RADAR
    %% =====================================================

    radar = ...
        scan_autonex_radar_suite( ...
            actors);

    radarObjects = ...
        radar_to_object_detections( ...
            radar, ...
            ego, ...
            t);


    %% =====================================================
    % 2. GNN TRACKING
    %% =====================================================

    [confirmedTracks,~,~] = ...
        tracker( ...
            radarObjects, ...
            t);


    %% =====================================================
    % 3. RGB CAMERA
    %% =====================================================

    rgb = ...
        simulate_rgb_camera( ...
            ego, ...
            actors, ...
            environment);


    %% =====================================================
    % 4. THERMAL IR
    %% =====================================================

    thermal = ...
        simulate_thermal_ir( ...
            ego, ...
            actors);


    %% =====================================================
    % 5. MULTIMODAL FUSION
    %% =====================================================

    fusedTracks = ...
        build_fused_track_perception( ...
            confirmedTracks, ...
            rgb, ...
            thermal, ...
            environment);


    %% =====================================================
    % 6. CUT-IN INTENT
    %% =====================================================

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


    %% =====================================================
    % 7. GENERATE 2-D TRAJECTORIES
    %% =====================================================

    candidates = ...
        generate_2d_candidate_set( ...
            ego, ...
            planningHorizons, ...
            egoLaneY);


    %% =====================================================
    % 8. EVALUATE + SELECT + STABILIZE + GUARD
    %% =====================================================

    if ~isempty(confirmedTracks)

        %% -------------------------------------------------
        % EVALUATE TRAJECTORIES
        %% -------------------------------------------------

        results = ...
            evaluate_2d_trajectories( ...
                candidates, ...
                confirmedTracks, ...
                fusedTracks, ...
                ego, ...
                intent, ...
                roadMinY, ...
                roadMaxY);


        %% -------------------------------------------------
        % RAW PLANNER PROPOSAL
        %% -------------------------------------------------

        [proposedCandidate, ...
         proposedIndex, ...
         proposedMode] = ...
            select_best_2d_trajectory( ...
                candidates, ...
                results);


        %% -------------------------------------------------
        % MANEUVER HYSTERESIS
        %% -------------------------------------------------

        [selectedCandidate, ...
         bestIndex, ...
         selectionMode, ...
         selectionState, ...
         stabilityMode] = ...
            stabilize_trajectory_selection( ...
                candidates, ...
                results, ...
                proposedCandidate, ...
                proposedIndex, ...
                proposedMode, ...
                selectionState, ...
                t);


        %% -------------------------------------------------
        % CAT REFLEX 2-D SAFETY GUARDIAN
        %% -------------------------------------------------

        [guardianCandidate, ...
         guardianIndex, ...
         guardianMode, ...
         guardianResults] = ...
            cat_reflex_2d_guardian( ...
                candidates, ...
                results, ...
                selectedCandidate, ...
                bestIndex, ...
                confirmedTracks, ...
                roadMinY, ...
                roadMaxY);


        %% -------------------------------------------------
        % GUARDIAN HAS FINAL AUTHORITY
        %% -------------------------------------------------

        selectedCandidate = ...
            guardianCandidate;

        bestIndex = ...
            guardianIndex;


        %% -------------------------------------------------
        % SYNCHRONIZE HYSTERESIS AFTER OVERRIDE
        %% -------------------------------------------------

        if ~isempty(selectedCandidate) && ...
           ~strcmp(guardianMode,'APPROVED')

            selectionState.initialized = true;

            selectionState.name = ...
                selectedCandidate.name;

            selectionState.targetSpeedKmh = ...
                selectedCandidate.targetSpeedKmh;

            selectionState.lastSwitchTime = ...
                t;

        end


    else

        %% =================================================
        % NO CONFIRMED TRACKS YET
        %% =================================================

        selectedCandidate = ...
            candidates(1);

        bestIndex = 1;

        selectionMode = ...
            'CRUISE';

        stabilityMode = ...
            'NO_TRACKS';

        guardianMode = ...
            'NO_TRACKS';

        guardianResults = ...
            struct([]);

    end


    %% =====================================================
    % 9. TARGETS
    %% =====================================================

    if isempty(selectedCandidate)

        targetSpeedKmh = 0;

        targetY = ...
            ego.y;

        selectionMode = ...
            'EMERGENCY';

        selectedName = ...
            'NONE';

    else

        targetSpeedKmh = ...
            selectedCandidate.targetSpeedKmh;

        targetY = ...
            selectedCandidate.targetY;

        selectedName = ...
            selectedCandidate.name;

    end


    %% =====================================================
    % 10. LONGITUDINAL CAT REFLEX
    %% =====================================================

    command = ...
        cat_reflex_guardian( ...
            currentSpeedKmh, ...
            targetSpeedKmh);

    longitudinalAcceleration = ...
        speed_tracking_controller( ...
            currentSpeedKmh, ...
            targetSpeedKmh, ...
            command);


    %% =====================================================
    % 11. LATERAL CONTROLLER
    %% =====================================================

    lateralAcceleration = ...
        lateral_tracking_controller( ...
            ego.y, ...
            ego.vy, ...
            targetY);


    %% =====================================================
    % 12. APPLY CONTROL
    %% =====================================================

    actors(1).ax = ...
        longitudinalAcceleration;

    actors(1).ay = ...
        lateralAcceleration;


    %% =====================================================
    % 13. VISUALIZATION
    %% =====================================================

    dashboard.time = t;
    dashboard.speed = currentSpeedKmh;
    dashboard.targetSpeed = targetSpeedKmh;
    dashboard.targetY = targetY;
    dashboard.selectionMode = selectionMode;
    dashboard.selectedName = selectedName;
    dashboard.stabilityMode = stabilityMode;
    dashboard.guardianMode = guardianMode;
    dashboard.pCut = 0;
    if ~isempty(intent), dashboard.pCut = intent.probability * 100; end
    scene = autonex_animation_scene('update', scene, actors, candidates, ...
        selectedCandidate, confirmedTracks, dashboard);
    drawnow;
    if ~isgraphics(scene.figure), break; end
    if exportVideo, writeVideo(video, getframe(scene.figure)); end

    %% =====================================================
    % PLANNER DEBUG OUTPUT EVERY 0.25 s
    %% =====================================================

    if mod(round(t/dt),5) == 0

        if isempty(intent)

            debugPCut = 0;

        else

            debugPCut = ...
                intent.probability * 100;

        end


        fprintf( ...
            ['t=%4.2f | ' ...
             'Pcut=%5.1f%% | ' ...
             'Maneuver=%-18s | ' ...
             'Speed=%5.1f | ' ...
             'TargetY=%5.2f | ' ...
             'Planner=%s | ' ...
             'Stability=%s | ' ...
             'Guardian=%s\n'], ...
            t, ...
            debugPCut, ...
            selectedName, ...
            targetSpeedKmh, ...
            targetY, ...
            selectionMode, ...
            stabilityMode, ...
            guardianMode);

    end


    %% =====================================================
    % 14. UPDATE PHYSICAL WORLD
    %% =====================================================

    actors = ...
        update_highway_actors( ...
            actors, ...
            dt);


    %% Prevent reverse

    actors(1).vx = ...
        max( ...
            actors(1).vx, ...
            0);


    %% Keep ego physically inside road

    actors(1).y = ...
        min( ...
            max( ...
                actors(1).y, ...
                roadMinY + 0.9), ...
            roadMaxY - 0.9);


    pause(max(0.001, dt - toc(frameClock)));

end
end

