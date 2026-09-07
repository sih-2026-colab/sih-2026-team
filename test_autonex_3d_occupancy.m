clear;
clc;
close all;

%% =========================================================
% AUTONEX 3-D OCCUPANCY TEST
%% =========================================================

actors = ...
    create_highway_scenario();


ego = ...
    actors(1);


%% =========================================================
% CREATE SYNTHETIC 3-D SENSOR SCENE
%% =========================================================

[worldPoints, ...
 sensorPoints, ...
 sensorPose] = ...
    create_autonex_3d_scene( ...
        actors);


fprintf('\n');
fprintf('============================================================\n');
fprintf(' AutoNex 3-D Occupancy Mapping\n');
fprintf('============================================================\n');

fprintf( ...
    'Generated 3-D points : %d\n', ...
    size(worldPoints,1));


%% =========================================================
% CREATE POINT CLOUD
%% =========================================================

sensorCloud = ...
    pointCloud(sensorPoints);


worldCloud = ...
    pointCloud(worldPoints);


%% =========================================================
% SHOW RAW 3-D WORLD
%% =========================================================

figure( ...
    'Name', ...
    'AutoNex Raw 3-D Point Cloud');


pcshow(worldCloud);

hold on;


plot3( ...
    ego.x, ...
    ego.y, ...
    1.0, ...
    's', ...
    'MarkerSize',14, ...
    'LineWidth',3);


title( ...
    'AutoNex — Synthetic 3-D Sensor Scene');


xlabel('X Forward (m)');
ylabel('Y Lateral (m)');
zlabel('Z Height (m)');


hold off;


%% =========================================================
% BUILD 3-D OCCUPANCY MAP
%% =========================================================

mapResolution = 4;

map3D = ...
    occupancyMap3D( ...
        mapResolution);


maxSensorRange = 80;


insertPointCloud( ...
    map3D, ...
    sensorPose, ...
    sensorCloud, ...
    maxSensorRange);


%% =========================================================
% VISUALIZE OCCUPANCY MAP
%% =========================================================

figure( ...
    'Name', ...
    'AutoNex 3-D Occupancy Map');


show(map3D);

hold on;


plot3( ...
    ego.x, ...
    ego.y, ...
    1.0, ...
    's', ...
    'MarkerSize',14, ...
    'LineWidth',3);


title( ...
    'AutoNex — Probabilistic 3-D Occupancy Map');


xlabel('X Forward (m)');
ylabel('Y Lateral (m)');
zlabel('Z Height (m)');


view(3);

grid on;

hold off;


%% =========================================================
% BASIC OCCUPANCY QUERY
%% =========================================================

queryPoints = [
    4.1  3.5  0.75
    20.0 7.0  0.75
    30.0 5.0  0.75
];


occupancyValues = ...
    getOccupancy( ...
        map3D, ...
        queryPoints);


fprintf('\nOccupancy queries:\n');

for i = 1:size(queryPoints,1)

    fprintf( ...
        'X=%5.1f Y=%4.1f Z=%4.2f -> Pocc=%.3f\n', ...
        queryPoints(i,1), ...
        queryPoints(i,2), ...
        queryPoints(i,3), ...
        occupancyValues(i));

end