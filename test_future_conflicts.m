clear;
clc;

actors = create_highway_scenario();

horizons = [0.5 1.0 1.5 2.0];

predictions = ...
    predict_actor_trajectories(actors, horizons);

conflicts = ...
    detect_future_conflicts( ...
        actors, predictions, horizons);

fprintf('\n--- AutoNex Future Conflict Detection ---\n\n');

if isempty(conflicts)

    fprintf('No predicted conflicts found.\n');

else

    for i = 1:length(conflicts)

        fprintf( ...
            'CONFLICT: %s | t+%.1f s | dx=%.2f m | dy=%.2f m | Risk=%s\n', ...
            conflicts(i).actor, ...
            conflicts(i).horizon, ...
            conflicts(i).dx, ...
            conflicts(i).dy, ...
            conflicts(i).risk);

    end

end