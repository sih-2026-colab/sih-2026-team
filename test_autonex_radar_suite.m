clear;
clc;

%% Make results repeatable
rng(1);


%% =========================================================
% CREATE HIGHWAY SCENARIO
%% =========================================================

actors = ...
    create_highway_scenario();


%% =========================================================
% SCAN USING AUTONEX RADAR SUITE
%% =========================================================

detections = ...
    scan_autonex_radar_suite(actors);


%% =========================================================
% DISPLAY RESULTS
%% =========================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' AutoNex 77–81 GHz Multi-Radar Sensor Simulation\n');
fprintf('============================================================\n\n');


if isempty(detections)

    fprintf('No objects detected.\n');

else

    for i = 1:length(detections)

        fprintf( ...
            '%-20s | %-12s | Range=%6.2f m | Angle=%7.2f deg | Radial V=%7.2f m/s\n', ...
            detections(i).sensor, ...
            detections(i).actorName, ...
            detections(i).range, ...
            detections(i).angle, ...
            detections(i).radialVelocity);

    end

end


fprintf('\n');

fprintf('NOTE:\n');
fprintf('Negative radial velocity = object closing toward ego.\n');
fprintf('Positive radial velocity = separation increasing.\n');