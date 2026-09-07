clear;
clc;

rng(20);

actors = create_highway_scenario();

ego = actors(1);

thermal = ...
    simulate_thermal_ir(ego, actors);


fprintf('\n');
fprintf('=============================================\n');
fprintf(' AutoNex Passive Thermal IR Simulation\n');
fprintf('=============================================\n\n');


if isempty(thermal)

    fprintf('No thermal objects detected.\n');

else

    for i = 1:length(thermal)

        fprintf( ...
            'Object %-5s | X=%6.2f | Y=%5.2f | Angle=%6.2f deg | Confidence=%5.1f%%\n', ...
            thermal(i).type, ...
            thermal(i).x, ...
            thermal(i).y, ...
            thermal(i).angle, ...
            thermal(i).confidence * 100);

    end

end