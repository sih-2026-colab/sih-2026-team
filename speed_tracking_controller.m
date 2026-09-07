function acceleration = speed_tracking_controller( ...
    currentSpeedKmh, targetSpeedKmh, command)

    % Convert speed error from km/h to m/s
    speedError = ...
        (targetSpeedKmh - currentSpeedKmh) / 3.6;


    % =====================================================
    % EMERGENCY BRAKE
    % =====================================================

    if strcmp(command, 'EMERGENCY_BRAKE')

        acceleration = -6.0;
        return;

    end


    % =====================================================
    % CONTROLLED BRAKING
    % =====================================================

    if strcmp(command, 'CONTROLLED_BRAKE')

        Kp = 1.5;

        acceleration = ...
            Kp * speedError;

        acceleration = ...
            max(acceleration, -3.5);

        acceleration = ...
            min(acceleration, 0);

        return;

    end


    % =====================================================
    % MICRO DECELERATION
    % =====================================================

    if strcmp(command, 'MICRO_DECELERATE')

        Kp = 1.2;

        acceleration = ...
            Kp * speedError;

        % Gentle deceleration
        acceleration = ...
            max(acceleration, -1.5);

        acceleration = ...
            min(acceleration, 0);

        return;

    end


    % =====================================================
    % MICRO ACCELERATION
    % =====================================================

    if strcmp(command, 'MICRO_ACCELERATE')

        Kp = 1.0;

        acceleration = ...
            Kp * speedError;

        acceleration = ...
            max(acceleration, 0);

        acceleration = ...
            min(acceleration, 1.2);

        return;

    end


    % =====================================================
    % CRUISE
    % =====================================================

    acceleration = 0;

end