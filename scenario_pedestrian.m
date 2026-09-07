function [egoX, egoV, pedX, pedY, pedVy] = scenario_pedestrian(t)

    egoV = 11.11;
    egoX = egoV * t;

    pedX = 35;

    if t < 1
        pedY = 5;
        pedVy = 0;
    else
        pedVy = -1.5;
        pedY = 5 + pedVy * (t - 1);
    end

end