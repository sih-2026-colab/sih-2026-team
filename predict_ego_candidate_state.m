function [futureX, futureSpeed, acceleration] = ...
    predict_ego_candidate_state( ...
        currentX, ...
        currentSpeed, ...
        targetSpeedKmh, ...
        horizon)

    %% =====================================================
    % AUTONEX ACCELERATION-LIMITED EGO PREDICTION
    %
    % currentSpeed     : m/s
    % targetSpeedKmh   : km/h
    % horizon          : seconds
    %
    % The vehicle cannot instantly jump to target speed.
    %% =====================================================


    targetSpeed = ...
        targetSpeedKmh / 3.6;


    %% =====================================================
    % SPEED ERROR
    %% =====================================================

    speedError = ...
        targetSpeed - currentSpeed;


    %% =====================================================
    % CONTROLLER-LIKE ACCELERATION COMMAND
    %% =====================================================

    Kp = 1.2;


    acceleration = ...
        Kp * speedError;


    %% Maximum acceleration

    acceleration = ...
        min(acceleration, 1.2);


    %% Maximum controlled deceleration

    acceleration = ...
        max(acceleration, -3.5);


    %% =====================================================
    % ALREADY AT TARGET
    %% =====================================================

    if abs(speedError) < 0.01

        acceleration = 0;

        futureSpeed = ...
            currentSpeed;

        futureX = ...
            currentX + ...
            currentSpeed * horizon;

        return;

    end


    %% =====================================================
    % TIME REQUIRED TO REACH TARGET
    %% =====================================================

    timeToTarget = ...
        speedError / acceleration;


    %% Numerical protection

    timeToTarget = ...
        max(timeToTarget, 0);


    %% =====================================================
    % TARGET NOT YET REACHED
    %% =====================================================

    if horizon <= timeToTarget

        futureSpeed = ...
            currentSpeed + ...
            acceleration * horizon;


        futureX = ...
            currentX + ...
            currentSpeed * horizon + ...
            0.5 * acceleration * horizon^2;


    %% =====================================================
    % TARGET REACHED INSIDE HORIZON
    %% =====================================================

    else

        distanceToTarget = ...
            currentSpeed * timeToTarget + ...
            0.5 * acceleration * ...
            timeToTarget^2;


        remainingTime = ...
            horizon - timeToTarget;


        futureX = ...
            currentX + ...
            distanceToTarget + ...
            targetSpeed * remainingTime;


        futureSpeed = ...
            targetSpeed;

    end


    %% Prevent negative vehicle speed

    futureSpeed = ...
        max(futureSpeed,0);

end