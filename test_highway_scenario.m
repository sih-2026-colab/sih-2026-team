clear;
clc;
close all;

%% Create scenario

actors = create_highway_scenario();

%% Simulation settings

dt = 0.05;
T = 6;

figure('Name','AutoNex 75-80-70 Highway Scenario');

for t = 0:dt:T

    clf;
    hold on;
    grid on;

    %% ---------------------------------
    % ROAD
    %% ---------------------------------

    % Road outer boundaries
    plot([-30 160], [-1.75 -1.75], 'k', 'LineWidth', 2);
    plot([-30 160], [8.75 8.75], 'k', 'LineWidth', 2);

    % Lane boundaries
    plot([-30 160], [1.75 1.75], 'k--');
    plot([-30 160], [5.25 5.25], 'k--');

    %% Lane labels

    text(-27, 0,   'LANE 1');
    text(-27, 3.5, 'LANE 2');
    text(-27, 7.0, 'LANE 3');

    %% ---------------------------------
    % DRAW VEHICLES
    %% ---------------------------------

    for i = 1:length(actors)

        plot(actors(i).x, ...
             actors(i).y, ...
             's', ...
             'MarkerSize', 12, ...
             'LineWidth', 2);

        text(actors(i).x + 1, ...
             actors(i).y + 0.3, ...
             actors(i).name);

    end

    %% ---------------------------------
    % DASHBOARD
    %% ---------------------------------

    ego = actors(1);
    rear = actors(2);
    cutin = actors(3);

    text(-25, 8.2, ...
        sprintf('Time: %.2f s', t));

    text(30, 8.2, ...
        sprintf('EGO: %.1f km/h', ego.vx * 3.6));

    text(65, 8.2, ...
        sprintf('CUT-IN: %.1f km/h', cutin.vx * 3.6));

    text(105, 8.2, ...
        sprintf('REAR: %.1f km/h', rear.vx * 3.6));

    %% ---------------------------------
    % VIEW
    %% ---------------------------------

    xlabel('Longitudinal Position X (m)');
    ylabel('Lateral Position Y (m)');

    title('AutoNex — 75 / 80 / 70 km/h Cut-In Scenario');

    xlim([-30 160]);
    ylim([-2.5 9.5]);

    drawnow;

    %% ---------------------------------
    % UPDATE ALL ACTORS
    %% ---------------------------------

    actors = update_highway_actors(actors, dt);

    pause(0.01);

end