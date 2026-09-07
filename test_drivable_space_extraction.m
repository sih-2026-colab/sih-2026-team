clear;
clc;
close all;

%% =========================================================
% AUTONEX LANE-INDEPENDENT DRIVABLE-SPACE TEST
%% =========================================================

actors = ...
    create_highway_scenario();


ego = ...
    actors(1);


%% =========================================================
% CREATE SYNTHETIC 3-D WORLD
%% =========================================================

[worldPoints, ...
 sensorPoints, ...
 sensorPose] = ...
    create_autonex_3d_scene( ...
        actors);


sensorCloud = ...
    pointCloud(sensorPoints);


%% =========================================================
% BUILD 3-D OCCUPANCY MAP
%% =========================================================

mapResolution = 4;


map3D = ...
    occupancyMap3D( ...
        mapResolution);


maxSensorRange = 80;


inverseSensorModel = [
    0.30 ...
    0.75
];

insertPointCloud( ...
    map3D, ...
    sensorPose, ...
    sensorCloud, ...
    maxSensorRange, ...
    inverseSensorModel);


%% =========================================================
% EXTRACT LANE-INDEPENDENT SPACE
%% =========================================================

forwardDistance = 60;

lateralMin = -1.75;

lateralMax = 8.75;


drivable = ...
    extract_drivable_space_from_3d( ...
        map3D, ...
        ego, ...
        forwardDistance, ...
        lateralMin, ...
        lateralMax);
fprintf( ...
    'Minimum map probability : %.3f\n', ...
    min(drivable.minimumOccupancy(:)));

fprintf( ...
    'Maximum map probability : %.3f\n', ...
    max(drivable.maximumOccupancy(:)));

%% =========================================================
% STATISTICS
%% =========================================================

freeCells = ...
    nnz(drivable.freeMask);


occupiedCells = ...
    nnz(drivable.occupiedMask);


unknownCells = ...
    nnz(drivable.unknownMask);


totalCells = ...
    numel(drivable.classification);


fprintf('\n');
fprintf('============================================================\n');
fprintf(' AutoNex Lane-Independent Drivable-Space Extraction\n');
fprintf('============================================================\n\n');


fprintf( ...
    'Total cells    : %d\n', ...
    totalCells);


fprintf( ...
    'Free cells     : %d\n', ...
    freeCells);


fprintf( ...
    'Occupied cells : %d\n', ...
    occupiedCells);


fprintf( ...
    'Unknown cells  : %d\n', ...
    unknownCells);


fprintf( ...
    'Free ratio     : %.1f%%\n', ...
    100 * freeCells / totalCells);


%% =========================================================
% 2-D DRIVABLE-SPACE VIEW
%% =========================================================

figure( ...
    'Name', ...
    'AutoNex Lane-Independent Drivable Space');


imagesc( ...
    drivable.xValues, ...
    drivable.yValues, ...
    drivable.classification);


axis xy;

hold on;


%% Ego

plot( ...
    ego.x, ...
    ego.y, ...
    's', ...
    'MarkerSize',12, ...
    'LineWidth',3);


%% True actor positions only for simulation/debug comparison

for i = 2:length(actors)

    plot( ...
        actors(i).x, ...
        actors(i).y, ...
        'x', ...
        'MarkerSize',10, ...
        'LineWidth',2);


    text( ...
        actors(i).x + 0.5, ...
        actors(i).y + 0.15, ...
        actors(i).name);

end


xlabel( ...
    'Forward X (m)');


ylabel( ...
    'Lateral Y (m)');


title( ...
    ['AutoNex — Occupancy-Derived Free / Unknown / ' ...
     'Occupied Space']);


xlim( ...
    [ego.x ego.x + forwardDistance]);


ylim( ...
    [lateralMin lateralMax]);


grid on;

hold off;


%% =========================================================
% 3-D MAP FOR COMPARISON
%% =========================================================

figure( ...
    'Name', ...
    'AutoNex 3-D Occupancy Reference');


show(map3D);

title( ...
    'AutoNex — Original 3-D Occupancy Map');


xlabel('X');
ylabel('Y');
zlabel('Z');

view(3);

grid on;