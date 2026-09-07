clear;
clc;

rng(30);


%% =========================================================
% WORLD
%% =========================================================

actors = ...
    create_highway_scenario();


ego = ...
    actors(1);


%% =========================================================
% DAYLIGHT ENVIRONMENT
%% =========================================================

environment.lightLevel = 1.0;

environment.visibility = 1.0;


%% =========================================================
% RGB CAMERA
%% =========================================================

rgb = ...
    simulate_rgb_camera( ...
        ego, ...
        actors, ...
        environment);


fprintf('\n');
fprintf('====================================================\n');
fprintf(' AutoNex RGB Camera — Daylight\n');
fprintf('====================================================\n\n');


for i = 1:length(rgb)

    fprintf( ...
        ['RGB | %-5s | Range=%6.2f m | ' ...
         'X=%6.2f | Y=%5.2f | ' ...
         'Angle=%7.2f deg | Confidence=%5.1f%%\n'], ...
        rgb(i).type, ...
        rgb(i).range, ...
        rgb(i).x, ...
        rgb(i).y, ...
        rgb(i).angle, ...
        rgb(i).confidence * 100);

end