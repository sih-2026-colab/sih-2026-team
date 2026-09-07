clear;
clc;
close all;

rng(50);

%% =========================================================
% AUTONEX UNCERTAINTY-AWARE CAT REFLEX PLANNER
%% =========================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' AutoNex Uncertainty-Aware Cat Reflex Planner\n');
fprintf('============================================================\n');


conditions = {
    'DAY'
    'NIGHT'
};


%% =========================================================
% TEST BOTH CONDITIONS
%% =========================================================

for c = 1:2

    %% =====================================================
    % CREATE NEW WORLD
    %% =====================================================

    actors = ...
        create_highway_scenario();


    tracker = ...
        create_autonex_gnn_tracker();


    %% =====================================================
    % ENVIRONMENT
    %% =====================================================

    if c == 1

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


    %% =====================================================
    % SIMULATION SETTINGS
    %% =====================================================

    dt = 0.05;

    T = 1.50;


    horizons = ...
        [0.5 1.0 1.5 2.0];


    speedsKmh = ...
        generate_candidate_speeds();


    egoLaneY = 7.0;


    fprintf('\n');
    fprintf('------------------------------------------------------------\n');

    fprintf( ...
        ' CONDITION: %s\n', ...
        conditions{c});

    fprintf('------------------------------------------------------------\n');


    %% =====================================================
    % SIMULATION LOOP
    %% =====================================================

    for t = 0:dt:T

        %% -------------------------------------------------
        % CURRENT EGO STATE
        %% -------------------------------------------------

        ego = ...
            actors(1);


        %% -------------------------------------------------
        % 1. RADAR
        %% -------------------------------------------------

        radar = ...
            scan_autonex_radar_suite( ...
                actors);


        radarObjects = ...
            radar_to_object_detections( ...
                radar, ...
                ego, ...
                t);


        %% -------------------------------------------------
        % 2. GNN TRACKING
        %% -------------------------------------------------

        [confirmedTracks,~,~] = ...
            tracker( ...
                radarObjects, ...
                t);


        %% -------------------------------------------------
        % 3. RGB CAMERA
        %% -------------------------------------------------

        rgb = ...
            simulate_rgb_camera( ...
                ego, ...
                actors, ...
                environment);


        %% -------------------------------------------------
        % 4. PASSIVE THERMAL IR
        %% -------------------------------------------------

        thermal = ...
            simulate_thermal_ir( ...
                ego, ...
                actors);


        %% -------------------------------------------------
        % 5. MULTIMODAL FUSION
        %% -------------------------------------------------

        fusedTracks = ...
            build_fused_track_perception( ...
                confirmedTracks, ...
                rgb, ...
                thermal, ...
                environment);


        %% -------------------------------------------------
        % 6. CUT-IN INTENT
        %% -------------------------------------------------

        intent = [];


        if ~isempty(confirmedTracks)

            [tempIntent, found] = ...
                find_highest_cutin_risk( ...
                    confirmedTracks, ...
                    ego, ...
                    egoLaneY);


            if found

                intent = ...
                    tempIntent;

            end

        end


        %% -------------------------------------------------
        % 7. PLANNER
        %% -------------------------------------------------

        if ~isempty(confirmedTracks)

            results = ...
                evaluate_uncertainty_aware_speeds( ...
                    confirmedTracks, ...
                    fusedTracks, ...
                    ego, ...
                    horizons, ...
                    speedsKmh, ...
                    intent);


            %% ---------------------------------------------
            % NEW MINIMUM-RISK SELECTOR
            %% ---------------------------------------------

            [selectedSpeed, ...
             bestIndex, ...
             selectionMode] = ...
                select_minimum_risk_action( ...
                    results);


            %% =============================================
            % PRINT EVERY 0.10 SECOND
            %% =============================================

            if mod(round(t/dt),2) == 0

                %% Cut-in probability

                if isempty(intent)

                    pCut = 0;

                else

                    pCut = ...
                        intent.probability * 100;

                end


                %% -----------------------------------------
                % NO ACTION AVAILABLE
                %% -----------------------------------------

                if isnan(selectedSpeed) || ...
                   isnan(bestIndex)

                    fprintf( ...
                        ['t=%4.2f | ' ...
                         'Pcut=%5.1f%% | ' ...
                         'NO ACTION\n'], ...
                        t, ...
                        pCut);


                %% -----------------------------------------
                % ACTION FOUND
                %% -----------------------------------------

                else

                    %% Rear TTC text

                    if isinf(results(bestIndex).rearTTC)

                        rearTTCText = ...
                            'INF';

                    else

                        rearTTCText = ...
                            sprintf( ...
                                '%.2f', ...
                                results(bestIndex).rearTTC);

                    end


                    %% Print planner status

                    fprintf( ...
                        ['t=%4.2f | ' ...
                         'Pcut=%5.1f%% | ' ...
                         'Target=%5.1f km/h | ' ...
                         'Mode=%s | ' ...
                         'Long=%5.2f m | ' ...
                         'Lat=%4.2f m | ' ...
                         'Front=%d | ' ...
                         'RearTTC=%s\n'], ...
                        t, ...
                        pCut, ...
                        selectedSpeed, ...
                        selectionMode, ...
                        results(bestIndex).maxLongitudinalEnvelope, ...
                        results(bestIndex).maxLateralEnvelope, ...
                        results(bestIndex).frontConflicts, ...
                        rearTTCText);

                end

            end

        end


        %% -------------------------------------------------
        % 8. UPDATE PHYSICAL WORLD
        %% -------------------------------------------------

        actors = ...
            update_highway_actors( ...
                actors, ...
                dt);

    end

end