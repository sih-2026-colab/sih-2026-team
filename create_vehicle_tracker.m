function kf = create_vehicle_tracker(initialX, initialY)

    %% =====================================================
    % AUTONEX 2D CONSTANT-VELOCITY KALMAN FILTER
    %
    % State:
    %
    % [ x
    %   vx
    %   y
    %   vy ]
    %% =====================================================


    initialState = [
        initialX
        0
        initialY
        0
    ];


    %% Initial uncertainty
    %
    % Position uncertainty relatively small.
    % Velocity uncertainty initially large because
    % one measurement cannot determine velocity.

    initialCovariance = diag([
        1.0
        25.0
        1.0
        25.0
    ]);


    %% Approximate Cartesian measurement noise

    measurementNoise = diag([
        0.30^2
        0.30^2
    ]);


    %% Create tracker

    kf = trackingKF( ...
        'MotionModel', ...
        '2D Constant Velocity', ...
        'State', ...
        initialState, ...
        'StateCovariance', ...
        initialCovariance, ...
        'MeasurementNoise', ...
        measurementNoise);

end