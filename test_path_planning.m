clear;
clc;

t = 2.0;

[egoX, egoV, pedX, pedY, pedVy] = ...
    scenario_pedestrian(t);

predictionHorizon = 1.5;

[futureY, uncertainty] = ...
    predict_pedestrian(pedY, pedVy, predictionHorizon);

bubbleRadius = ...
    dynamic_safety_bubble(egoV, uncertainty);

paths = generate_candidate_paths();

[scores, safe] = ...
    score_paths(paths, pedX, futureY, ...
                bubbleRadius, egoX, egoV, predictionHorizon);

[bestPath, bestIndex] = ...
    select_best_path(paths, scores);

fprintf('\n--- AutoNex Candidate Path Evaluation ---\n\n');

for i = 1:length(paths)

    if safe(i)
        fprintf('Path %.1f m -> SAFE | Score %.2f\n', ...
            paths(i), scores(i));
    else
        fprintf('Path %.1f m -> UNSAFE\n', paths(i));
    end

end

fprintf('\n');

if isnan(bestPath)
    fprintf('Decision: NO SAFE PATH -> BRAKE\n');
else
    fprintf('Selected safest path: %.1f m\n', bestPath);
end