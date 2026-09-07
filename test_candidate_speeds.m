clear;
clc;

actors = create_highway_scenario();

horizons = [0.5 1.0 1.5 2.0];

speedsKmh = ...
    generate_candidate_speeds();

results = ...
    evaluate_candidate_speeds( ...
        actors, horizons, speedsKmh);

fprintf('\n');
fprintf('--- AutoNex Candidate Speed Analysis ---\n\n');

for i = 1:length(results)

    if results(i).safe

        status = 'SAFE';

    else

        status = 'UNSAFE';

    end

    fprintf( ...
        '%2.0f km/h -> %-6s | Conflicts = %d | Min Clearance = %.2f m\n', ...
        results(i).speedKmh, ...
        status, ...
        results(i).conflicts, ...
        results(i).minClearance);

end


selectedSpeed = ...
    select_safe_speed(results);


fprintf('\n');

if isnan(selectedSpeed)

    fprintf( ...
        'Decision: NO SAFE SPEED -> BRAKE\n');

else

    fprintf( ...
        'AutoNex Selected Speed: %.0f km/h\n', ...
        selectedSpeed);

end