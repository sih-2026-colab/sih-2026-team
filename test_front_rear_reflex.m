clear;
clc;

%% Scenario

actors = create_highway_scenario();

horizons = [0.5 1.0 1.5 2.0];

speedsKmh = ...
    generate_candidate_speeds();


%% Evaluate

results = ...
    evaluate_front_rear_speeds( ...
        actors, horizons, speedsKmh);


fprintf('\n');
fprintf('=============================================\n');
fprintf(' AutoNex Front + Rear Cat Reflex Analysis\n');
fprintf('=============================================\n\n');


for i = 1:length(results)

    if results(i).safe

        status = 'SAFE';

    else

        status = 'UNSAFE';

    end


    if isinf(results(i).rearTTC)

        rearText = 'INF';

    else

        rearText = ...
            sprintf('%.2f',results(i).rearTTC);

    end


    fprintf( ...
        '%2.0f km/h | %-6s | Front=%d | Rear TTC=%s s | Rear Risk=%d | Score=%.2f\n', ...
        results(i).speedKmh, ...
        status, ...
        results(i).frontConflicts, ...
        rearText, ...
        results(i).rearRisk, ...
        results(i).score);

end


%% Select minimum-risk action

[selectedSpeed,bestIndex] = ...
    select_minimum_risk_speed(results);


currentSpeed = ...
    actors(1).vx * 3.6;


command = ...
    cat_reflex_guardian( ...
        currentSpeed,selectedSpeed);


fprintf('\n');


if isnan(selectedSpeed)

    fprintf('No safe candidate speed found.\n');

else

    fprintf( ...
        'AutoNex Selected Speed : %.1f km/h\n', ...
        selectedSpeed);

end


fprintf( ...
    'Cat Reflex Command      : %s\n', ...
    command);


if ~isnan(bestIndex)

    fprintf( ...
        'Selected Rear TTC     : %.2f s\n', ...
        results(bestIndex).rearTTC);

    fprintf( ...
        'Selected Front Conflicts : %d\n', ...
        results(bestIndex).frontConflicts);

end