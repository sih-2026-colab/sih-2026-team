clear;
clc;
close all;

%% =========================================================
% AUTONEX CLOSED-LOOP HIGHWAY SIMULATION
%% =========================================================

actors = create_highway_scenario();

dt = 0.05;
T = 8.0;

horizons = [0.5 1.0 1.5 2.0];

speedsKmh = ...
    generate_candidate_speeds();


%% =========================================================
% FIGURE
%% =========================================================

figure( ...
    'Name', ...
    'AutoNex Live Cat Reflex Highway Simulation');


%% =========================================================
% MAIN SIMULATION LOOP
%% =========================================================

for t = 0:dt:T

    %% -----------------------------------------------------
    % 1. CURRENT EGO SPEED
    %% -----------------------------------------------------

    currentSpeedKmh = ...
        actors(1).vx * 3.6;


    %% -----------------------------------------------------
    % 2. EVALUATE ALL CANDIDATE SPEEDS
    %% -----------------------------------------------------

    results = ...
        evaluate_front_rear_speeds( ...
            actors, ...
            horizons, ...
            speedsKmh);


    %% -----------------------------------------------------
    % 3. SELECT MINIMUM-RISK SPEED
    %% -----------------------------------------------------

    [selectedSpeed, bestIndex] = ...
        select_minimum_risk_speed(results);


    %% -----------------------------------------------------
    % 4. CAT REFLEX GUARDIAN
    %% -----------------------------------------------------

    command = ...
        cat_reflex_guardian( ...
            currentSpeedKmh, ...
            selectedSpeed);


    %% -----------------------------------------------------
    % 5. IF NO SAFE SPEED EXISTS
    %% -----------------------------------------------------

    if isnan(selectedSpeed)

        selectedSpeed = 0;

        command = ...
            'EMERGENCY_BRAKE';

    end


    %% -----------------------------------------------------
    % 6. SPEED CONTROLLER
    %% -----------------------------------------------------

    acceleration = ...
        speed_tracking_controller( ...
            currentSpeedKmh, ...
            selectedSpeed, ...
            command);


    % Apply acceleration to ego
    actors(1).ax = acceleration;


    %% -----------------------------------------------------
    % 7. IMPORTANT ACTORS
    %% -----------------------------------------------------

    ego = actors(1);
    rear = actors(2);
    cutin = actors(3);


    %% -----------------------------------------------------
    % 8. REAR TTC
    %% -----------------------------------------------------

    [rearGap, rearTTC] = ...
        calculate_rear_ttc( ...
            ego.x, ...
            ego.vx, ...
            rear.x, ...
            rear.vx);


    %% -----------------------------------------------------
    % 9. CUT-IN GAP
    %% -----------------------------------------------------

    cutInGap = ...
        cutin.x - ego.x;


    %% -----------------------------------------------------
    % 10. VISUALIZATION
    %% -----------------------------------------------------

    clf;
    hold on;
    grid on;


    %% Road boundaries

    plot( ...
        [ego.x-40 ego.x+120], ...
        [-1.75 -1.75], ...
        'k', ...
        'LineWidth', 2);

    plot( ...
        [ego.x-40 ego.x+120], ...
        [8.75 8.75], ...
        'k', ...
        'LineWidth', 2);


    %% Lane boundaries

    plot( ...
        [ego.x-40 ego.x+120], ...
        [1.75 1.75], ...
        'k--');

    plot( ...
        [ego.x-40 ego.x+120], ...
        [5.25 5.25], ...
        'k--');


    %% -----------------------------------------------------
    % DRAW ALL VEHICLES
    %% -----------------------------------------------------

    for i = 1:length(actors)

        plot( ...
            actors(i).x, ...
            actors(i).y, ...
            's', ...
            'MarkerSize', 12, ...
            'LineWidth', 2);


        text( ...
            actors(i).x + 1, ...
            actors(i).y + 0.25, ...
            actors(i).name);

    end


    %% -----------------------------------------------------
    % DASHBOARD TEXT
    %% -----------------------------------------------------

    dashboardX = ...
        ego.x - 25;


    text( ...
        dashboardX, ...
        8.2, ...
        sprintf( ...
        'Time: %.2f s', ...
        t));


    text( ...
        dashboardX + 25, ...
        8.2, ...
        sprintf( ...
        'EGO: %.2f km/h', ...
        currentSpeedKmh));


    text( ...
        dashboardX + 55, ...
        8.2, ...
        sprintf( ...
        'Target: %.1f km/h', ...
        selectedSpeed));


    text( ...
        dashboardX + 85, ...
        8.2, ...
        sprintf( ...
        'Accel: %.2f m/s^2', ...
        acceleration));


    %% Rear TTC text

    if isinf(rearTTC)

        rearTTCText = 'INF';

    else

        rearTTCText = ...
            sprintf('%.2f', rearTTC);

    end


    text( ...
        dashboardX + 115, ...
        8.2, ...
        sprintf( ...
        'Rear TTC: %s s', ...
        rearTTCText));


    %% Cat Reflex command

    text( ...
        ego.x - 10, ...
        -1.0, ...
        sprintf( ...
        'CAT REFLEX: %s', ...
        command), ...
        'FontWeight', ...
        'bold');


    %% Cut-in distance

    text( ...
        ego.x - 10, ...
        -0.3, ...
        sprintf( ...
        'Cut-In Gap: %.2f m', ...
        cutInGap));


    %% Selected action conflict count

    if ~isnan(bestIndex)

        text( ...
            ego.x + 25, ...
            -1.0, ...
            sprintf( ...
            'Future conflicts: %d', ...
            results(bestIndex).frontConflicts));

    end


    %% -----------------------------------------------------
    % VIEW SETTINGS
    %% -----------------------------------------------------

    xlabel( ...
        'Longitudinal Position X (m)');

    ylabel( ...
        'Lateral Position Y (m)');


    title( ...
        'AutoNex — Live Front + Rear Cat Reflex');


    xlim( ...
        [ego.x-30 ego.x+100]);

    ylim([-2.5 9.5]);


    drawnow;


    %% -----------------------------------------------------
    % 11. UPDATE ALL VEHICLES
    %% -----------------------------------------------------

    actors = ...
        update_highway_actors( ...
            actors, ...
            dt);


    %% -----------------------------------------------------
    % 12. PREVENT NEGATIVE SPEED
    %% -----------------------------------------------------

    if actors(1).vx < 0

        actors(1).vx = 0;

    end


    pause(0.01);

end