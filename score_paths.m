function [scores, safe] = score_paths(paths, pedX, futureY, bubbleRadius, egoX, egoV, horizon)

    n = length(paths);

    scores = zeros(1,n);
    safe = true(1,n);

    % Predict ego vehicle position after prediction horizon
    futureEgoX = egoX + egoV * horizon;

    for i = 1:n

        pathY = paths(i);

        % Relative future position
        dx = pedX - futureEgoX;
        dy = futureY - pathY;

        predictedClearance = sqrt(dx^2 + dy^2);

        if predictedClearance <= bubbleRadius
            safe(i) = false;
            scores(i) = inf;
        else
            % Lower score is better
            collisionRisk = 10 / predictedClearance;
            pathDeviation = abs(pathY);

            scores(i) = collisionRisk + pathDeviation;
        end

    end
end