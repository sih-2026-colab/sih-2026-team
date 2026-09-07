clear;
clc;
close all;

%% =========================================================
% AUTONEX VEHICLE-FOOTPRINT INFLATION TEST
%% =========================================================

actors = ...
    create_highway_scenario();


ego = ...
    actors(1);


%% =========================================================
% SYNTHETIC 3-D WORLD
%% =========================================================

[~, ...
 sensorPoints, ...
 sensorPose] = ...
    create_autonex_3d_scene( ...
        actors);


sensorCloud = ...
    pointCloud(sensorPoints);


%% =========================================================
% 3-D OCCUPANCY MAP
%% =========================================================

map3D = ...
    occupancyMap3D(4);


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
% EXTRACT DRIVABLE SPACE
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


%% =========================================================
% EGO VEHICLE DIMENSIONS
%% =========================================================

vehicleLength = 4.5;

vehicleWidth = 1.9;


%% =========================================================
% INFLATE BLOCKED SPACE
%% =========================================================

safeSpace = ...
    inflate_drivable_space_for_vehicle( ...
        drivable, ...
        vehicleLength, ...
        vehicleWidth);


%% =========================================================
% STATISTICS
%% =========================================================

rawFreeCells = ...
    nnz(drivable.freeMask);


safeCenterCells = ...
    nnz(safeSpace.safeFreeMask);


blockedCells = ...
    nnz(safeSpace.blockedMask);


inflatedBlockedCells = ...
    nnz(safeSpace.inflatedBlockedMask);


fprintf('\n');
fprintf('============================================================\n');
fprintf(' AutoNex Vehicle-Footprint Safety Inflation\n');
fprintf('============================================================\n\n');


fprintf( ...
    'Raw free cells       : %d\n', ...
    rawFreeCells);


fprintf( ...
    'Blocked cells        : %d\n', ...
    blockedCells);


fprintf( ...
    'Inflated blocked     : %d\n', ...
    inflatedBlockedCells);


fprintf( ...
    'Safe ego-center cells: %d\n', ...
    safeCenterCells);


fprintf( ...
    'Inflation X          : %d cells\n', ...
    safeSpace.inflateXCells);


fprintf( ...
    'Inflation Y          : %d cells\n', ...
    safeSpace.inflateYCells);


fprintf( ...
    'Safe-space ratio     : %.1f%%\n', ...
    100 * safeCenterCells / ...
    numel(drivable.freeMask));


%% =========================================================
% BUILD VISUALIZATION GRID
%
% -1 = unsafe / unknown / occupied
%  0 = observed free but too close to blocked space
%  1 = safe for ego center
%% =========================================================

planningView = ...
    -1 * ones( ...
        size(drivable.classification));


planningView( ...
    drivable.freeMask) = 0;


planningView( ...
    safeSpace.safeFreeMask) = 1;


%% =========================================================
% VISUALIZATION
%% =========================================================

figure( ...
    'Name', ...
    'AutoNex Vehicle Footprint Safe Space');


imagesc( ...
    drivable.xValues, ...
    drivable.yValues, ...
    planningView);


axis xy;

hold on;


%% Ego

plot( ...
    ego.x, ...
    ego.y, ...
    's', ...
    'MarkerSize',12, ...
    'LineWidth',3);


%% True actors are shown only for simulation validation

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
    ['AutoNex — Vehicle-Footprint-Aware ' ...
     'Lane-Independent Safe Space']);


xlim( ...
    [ego.x ego.x + forwardDistance]);


ylim( ...
    [lateralMin lateralMax]);


grid on;

hold off;