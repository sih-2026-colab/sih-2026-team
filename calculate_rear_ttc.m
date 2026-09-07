function [rearGap, rearTTC] = calculate_rear_ttc( ...
    egoX, egoSpeed, rearX, rearSpeed)

    % Distance from rear vehicle to ego vehicle
    rearGap = egoX - rearX;

    % Positive closing speed means rear vehicle
    % is approaching the ego vehicle.
    closingSpeed = rearSpeed - egoSpeed;

    if rearGap <= 0

        rearTTC = 0;

    elseif closingSpeed > 0

        rearTTC = rearGap / closingSpeed;

    else

        % Rear vehicle is not gaining on ego
        rearTTC = inf;

    end

end