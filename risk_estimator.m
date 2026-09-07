function [distance, ttc, risk] = risk_estimator(egoX, egoV, pedX, pedY)

    % Relative position
    dx = pedX - egoX;
    dy = pedY;

    % Actual distance to pedestrian
    distance = sqrt(dx^2 + dy^2);

    % Time To Collision
    if egoV > 0 && dx > 0
        ttc = dx / egoV;
    else
        ttc = inf;
    end

    % Risk levels
    % 1 = LOW
    % 2 = MEDIUM
    % 3 = HIGH

    % HIGH RISK
    if ttc <= 1.5 && abs(pedY) <= 2.5

        risk = 3;

    % MEDIUM RISK
    elseif ttc <= 3.0 && abs(pedY) <= 4.0

        risk = 2;

    % LOW RISK
    else

        risk = 1;

    end

end