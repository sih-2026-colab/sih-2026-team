function filter = create_pedestrian_tracker(initialMeasurement)

    % State:
    %
    % [x
    %  vx
    %  y
    %  vy]

    x0 = initialMeasurement(1);
    y0 = initialMeasurement(2);

    initialState = [
        x0
        0
        y0
        0
    ];

    initialCovariance = diag([
        1
        10
        1
        10
    ]);

    filter = trackingKF( ...
        'MotionModel','2D Constant Velocity', ...
        'State',initialState, ...
        'StateCovariance',initialCovariance);

end