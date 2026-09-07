function visualize_autonex()

clc;
close all;

%% Simulation
dt = 0.05;
T  = 6.0;

%% Ego initial state
egoX = 0;
egoY = 0;
egoV = 11.11;       % 40 km/h

%% Pedestrian
pedX = 35;
pedStartY = 5;
pedVy = -1.5;
pedStartTime = 1.0;

%% Prediction
horizon = 1.5;

%% Figure
figure('Name','AutoNex Closed-Loop Collision Avoidance');

for t = 0:dt:T

    %% --------------------------------
    % 1. PEDESTRIAN MOTION
    %% --------------------------------

    if t < pedStartTime
        pedY = pedStartY;
        currentPedVy = 0;
    else
        currentPedVy = pedVy;
        pedY = pedStartY + ...
               currentPedVy*(t-pedStartTime);
    end


    %% --------------------------------
    % 2. PEDESTRIAN PREDICTION
    %% --------------------------------

    [futureY, uncertainty] = ...
        predict_pedestrian( ...
        pedY,currentPedVy,horizon);


    %% --------------------------------
    % 3. DYNAMIC SAFETY BUBBLE
    %% --------------------------------

    bubbleRadius = ...
        dynamic_safety_bubble( ...
        egoV,uncertainty);


    %% --------------------------------
    % 4. CANDIDATE PATHS
    %% --------------------------------

    paths = generate_candidate_paths();

    [scores,safe] = ...
        score_paths( ...
        paths,pedX,futureY, ...
        bubbleRadius,egoX, ...
        egoV,horizon);

    [bestPath,bestIndex] = ...
        select_best_path(paths,scores);


    %% --------------------------------
    % 5. CURRENT COLLISION RISK
    %% --------------------------------

    [distance,ttc,risk] = ...
        risk_estimator( ...
        egoX,egoV,pedX,pedY);


    %% --------------------------------
    % 6. FINAL AUTONEX DECISION
    %% --------------------------------

    command = ...
        autonex_decision(risk,safe);


    %% --------------------------------
    % 7. VEHICLE CONTROLLER
    %% --------------------------------

    acceleration = ...
        vehicle_controller(command);


    %% --------------------------------
    % 8. STOPPING DISTANCE
    %% --------------------------------

    emergencyDecel = 6.0;

    stoppingDistance = ...
        egoV^2/(2*emergencyDecel);


    %% --------------------------------
    % 9. DASHBOARD DECISION TEXT
    %% --------------------------------

    if command == 0
        decisionText = 'CRUISE';

    elseif command == 1
        decisionText = 'BRAKE';

    else
        decisionText = 'REPLAN';
    end


    %% --------------------------------
    % 10. VISUALIZATION
    %% --------------------------------

    clf;
    hold on;
    grid on;

    xlim([0 60]);
    ylim([-8 8]);

    xlabel('X Position (m)');
    ylabel('Y Position (m)');

    title('AutoNex — Closed-Loop Uncertainty-Aware Planner');


    %% Road boundaries

    plot([0 60],[-4 -4],'k--');
    plot([0 60],[4 4],'k--');
    plot([0 60],[0 0],'k:');


    %% Ego vehicle

    plot(egoX,egoY,'s', ...
        'MarkerSize',14, ...
        'MarkerFaceColor','b');


    %% Pedestrian

    plot(pedX,pedY,'o', ...
        'MarkerSize',10, ...
        'MarkerFaceColor','r');


    %% Predicted pedestrian position

    plot(pedX,futureY,'x', ...
        'MarkerSize',14, ...
        'LineWidth',2);


    %% Safety bubble

    theta = linspace(0,2*pi,120);

    bubbleX = ...
        pedX + bubbleRadius*cos(theta);

    bubbleY = ...
        futureY + bubbleRadius*sin(theta);

    plot(bubbleX,bubbleY,'--');


    %% Candidate paths

    for i = 1:length(paths)

        targetX = min(egoX+20,60);

        if safe(i)

            plot( ...
                [egoX targetX], ...
                [egoY paths(i)], ...
                'LineWidth',1.5);

        else

            plot( ...
                [egoX targetX], ...
                [egoY paths(i)], ...
                '--', ...
                'LineWidth',1.5);

        end

    end


    %% Selected path

    if ~isnan(bestPath)

        targetX = min(egoX+20,60);

        plot( ...
            [egoX targetX], ...
            [egoY bestPath], ...
            'LineWidth',4);

    end


    %% Dashboard

    text(2,7.2, ...
        sprintf('Time: %.2f s',t));

    text(2,6.4, ...
        sprintf('Speed: %.1f km/h',egoV*3.6));

    text(2,5.6, ...
        sprintf('Acceleration: %.1f m/s^2',acceleration));

    text(2,4.8, ...
        sprintf('TTC: %.2f s',ttc));

    text(2,4.0, ...
        sprintf('Risk: %d',risk));

    text(2,3.2, ...
        sprintf('Uncertainty: %.2f m',uncertainty));

    text(2,2.4, ...
        sprintf('Safety Bubble: %.2f m',bubbleRadius));

    text(2,1.6, ...
        sprintf('Stopping Distance: %.2f m',stoppingDistance));

    text(2,0.8, ...
        sprintf('Decision: %s',decisionText));


    %% Planner information

    if ~any(safe)

        text(22,-6, ...
            'NO SAFE PATH -> EMERGENCY BRAKE', ...
            'FontWeight','bold');

    elseif command == 2

        text(22,-6, ...
            sprintf('REPLAN -> Selected Y = %.1f m',bestPath), ...
            'FontWeight','bold');

    end


    drawnow;


    %% --------------------------------
    % 11. UPDATE VEHICLE STATE
    %% --------------------------------

    [egoX,egoV] = ...
        update_vehicle( ...
        egoX,egoV,acceleration,dt);


    %% Stop simulation if vehicle stopped

    if egoV <= 0.01

        fprintf('\nAUTO NEX VEHICLE STOPPED\n');
        fprintf('Time             : %.2f s\n',t);
        fprintf('Vehicle X        : %.2f m\n',egoX);
        fprintf('Pedestrian X     : %.2f m\n',pedX);
        fprintf('Remaining X gap  : %.2f m\n',pedX-egoX);

        break;

    end

    pause(0.02);

end

end