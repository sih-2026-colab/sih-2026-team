clear;
clc;
close all;

rng(5);

%% =========================================================
% AUTONEX RADAR + KALMAN TRACKING TEST
%% =========================================================

actors = create_highway_scenario();

dt = 0.05;
T = 3.0;

kf = [];

trackerInitialized = false;


fprintf('\n');
fprintf('===============================================================\n');
fprintf(' AutoNex 77-81 GHz Radar + Kalman Tracking\n');
fprintf('===============================================================\n\n');

fprintf( ...
    'Time | True X | True Y | Meas X | Meas Y | Est X | Est Y | Est Vx | Est Vy\n');

fprintf( ...
    '----------------------------------------------------------------------------\n');


%% =========================================================
% SIMULATION LOOP
%% =========================================================

for t = 0:dt:T

    ego = actors(1);


    %% -----------------------------------------------------
    % RADAR SCAN
    %% -----------------------------------------------------

    detections = ...
        scan_autonex_radar_suite(actors);


    %% -----------------------------------------------------
    % FIND CUT-IN USING FRONT RADAR
    %
    % actorID is temporarily used ONLY for simulation
    % validation.
    %
    % Later trackerGNN will replace this association.
    %% -----------------------------------------------------

    detectionFound = false;

    cutDetection = [];


    for i = 1:length(detections)

        if detections(i).actorID == 3 && ...
           strcmp(detections(i).sensor, 'FRONT_RADAR')

            cutDetection = detections(i);

            detectionFound = true;

            break;

        end

    end


    %% -----------------------------------------------------
    % IF RADAR SEES THE CUT-IN VEHICLE
    %% -----------------------------------------------------

    if detectionFound

        [measX, measY] = ...
            radar_to_cartesian( ...
                cutDetection, ego);


        %% -----------------------------------------------
        % INITIALIZE FILTER FROM FIRST MEASUREMENT
        %% -----------------------------------------------

        if ~trackerInitialized

            kf = ...
                create_vehicle_tracker( ...
                    measX, measY);

            trackerInitialized = true;


        else

            %% Predict next state

            predict(kf, dt);


            %% Correct using radar X,Y measurement

            correct( ...
                kf, ...
                [measX; measY]);

        end


        %% -----------------------------------------------
        % GET CURRENT FILTER STATE
        %% -----------------------------------------------

        state = kf.State;

        estX  = state(1);
        estVx = state(2);

        estY  = state(3);
        estVy = state(4);


        %% Ground truth only for evaluation

        trueX = actors(3).x;
        trueY = actors(3).y;


        %% Print

        fprintf( ...
            '%4.2f | %6.2f | %6.2f | %6.2f | %6.2f | %6.2f | %6.2f | %7.2f | %7.2f\n', ...
            t, ...
            trueX, ...
            trueY, ...
            measX, ...
            measY, ...
            estX, ...
            estY, ...
            estVx, ...
            estVy);

    end


    %% -----------------------------------------------------
    % UPDATE TRUE SIMULATION
    %% -----------------------------------------------------

    actors = ...
        update_highway_actors( ...
            actors, dt);

end