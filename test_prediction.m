clear;
clc;

t = 2.0;

[egoX, egoV, pedX, pedY, pedVy] = scenario_pedestrian(t);

predictionHorizon = 1.5;

[futureY, uncertainty] = ...
    predict_pedestrian(pedY, pedVy, predictionHorizon);

bubbleRadius = ...
    dynamic_safety_bubble(egoV, uncertainty);

fprintf('\n--- AutoNex Prediction ---\n');

fprintf('Current pedestrian Y   : %.2f m\n', pedY);
fprintf('Pedestrian velocity    : %.2f m/s\n', pedVy);
fprintf('Predicted Y after %.1f s: %.2f m\n', ...
    predictionHorizon, futureY);

fprintf('Prediction uncertainty : %.2f m\n', uncertainty);
fprintf('Dynamic safety bubble  : %.2f m\n', bubbleRadius);