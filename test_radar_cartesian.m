clear;
clc;

rng(1);

actors = create_highway_scenario();

detections = ...
    scan_autonex_radar_suite(actors);

ego = actors(1);

fprintf('\n');
fprintf('==============================================\n');
fprintf(' AutoNex Radar -> Cartesian Conversion\n');
fprintf('==============================================\n\n');


for i = 1:length(detections)

    [globalX, globalY, relativeX, relativeY] = ...
        radar_to_cartesian( ...
            detections(i), ego);

    fprintf( ...
        '%-20s | %-12s | Xrel=%7.2f | Yrel=%7.2f | X=%7.2f | Y=%7.2f\n', ...
        detections(i).sensor, ...
        detections(i).actorName, ...
        relativeX, ...
        relativeY, ...
        globalX, ...
        globalY);

end