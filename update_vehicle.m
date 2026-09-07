function [newX, newV] = update_vehicle(x, v, acceleration, dt)

    % Update velocity
    newV = v + acceleration * dt;

    % Vehicle cannot move backwards
    newV = max(newV, 0);

    % Update position
    newX = x + newV * dt;

end