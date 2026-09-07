clear;
clc;

actors = create_highway_scenario();

horizons = [0.5 1.0 1.5 2.0];

predictions = ...
    predict_actor_trajectories(actors, horizons);

fprintf('\n--- AutoNex Multi-Actor Prediction ---\n\n');

for i = 1:length(actors)

    fprintf('Actor: %s\n', actors(i).name);

    for j = 1:length(horizons)

        fprintf( ...
            '  t+%.1f s -> X = %.2f m | Y = %.2f m | Uncertainty = %.2f m\n', ...
            horizons(j), ...
            predictions(i,j).x, ...
            predictions(i,j).y, ...
            predictions(i,j).uncertainty);

    end

    fprintf('\n');

end