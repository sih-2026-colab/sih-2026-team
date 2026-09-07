function lateralAcceleration = ...
    lateral_tracking_controller( ...
        currentY, ...
        currentVy, ...
        targetY)

    %% =====================================================
    % AUTONEX LATERAL TRACKING CONTROLLER
    %
    % Simple PD controller for prototype closed-loop motion.
    %% =====================================================

    Kp = 1.8;
    Kd = 1.4;


    %% Position error

    lateralError = ...
        targetY - currentY;


    %% PD control

    lateralAcceleration = ...
        Kp * lateralError - ...
        Kd * currentVy;


    %% =====================================================
    % PHYSICAL / COMFORT LIMIT
    %% =====================================================

    maxLateralAcceleration = 2.5;


    lateralAcceleration = ...
        max( ...
            lateralAcceleration, ...
            -maxLateralAcceleration);


    lateralAcceleration = ...
        min( ...
            lateralAcceleration, ...
            maxLateralAcceleration);

end