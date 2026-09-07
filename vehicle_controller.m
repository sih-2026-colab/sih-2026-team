function acceleration = vehicle_controller(command)

    % command:
    % 0 = CRUISE
    % 1 = BRAKE
    % 2 = REPLAN

    if command == 1

        % Emergency braking
        acceleration = -6.0;     % m/s^2

    elseif command == 2

        % Gentle slowdown while replanning
        acceleration = -2.0;

    else

        % Cruise
        acceleration = 0;

    end

end