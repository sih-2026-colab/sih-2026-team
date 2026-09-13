function command = cat_reflex_guardian( ...
    currentSpeedKmh, selectedSpeedKmh)

    if isnan(selectedSpeedKmh)

        command = 'EMERGENCY_BRAKE';

        return;

    end


    speedDifference = ...
        selectedSpeedKmh - currentSpeedKmh;


    if abs(speedDifference) < 0.1

        command = 'CRUISE';


    elseif speedDifference < 0 && ...
           abs(speedDifference) <= 5

        command = 'MICRO_DECELERATE';


    elseif speedDifference < -5

        command = 'CONTROLLED_BRAKE';


    elseif speedDifference > 0 && ...
           speedDifference <= 5

        command = 'MICRO_ACCELERATE';


    else

        % The planner has already selected a guarded target. Allow bounded
        % acceleration even when the positive speed gap exceeds 5 km/h.
        command = 'MICRO_ACCELERATE';

    end

end
