function trajectory = generate_2d_trajectory( ...
    ego, ...
    targetSpeedKmh, ...
    targetY, ...
    horizons, ...
    maneuverTime)

    %% =====================================================
    % AUTONEX 2-D CANDIDATE TRAJECTORY GENERATOR
    %
    % Longitudinal:
    % acceleration-limited speed transition
    %
    % Lateral:
    % smooth quintic trajectory
    %
    % s(t) = 10*tau^3 - 15*tau^4 + 6*tau^5
    %% =====================================================

    n = length(horizons);


    trajectory.time = ...
        horizons;


    trajectory.x = ...
        zeros(1,n);


    trajectory.y = ...
        zeros(1,n);


    trajectory.vx = ...
        zeros(1,n);


    trajectory.vy = ...
        zeros(1,n);


    trajectory.ay = ...
        zeros(1,n);


    trajectory.targetSpeedKmh = ...
        targetSpeedKmh;


    trajectory.targetY = ...
        targetY;


    trajectory.maneuverTime = ...
        maneuverTime;


    %% =====================================================
    % LATERAL DISPLACEMENT
    %% =====================================================

    deltaY = ...
        targetY - ego.y;


    %% =====================================================
    % GENERATE FUTURE STATES
    %% =====================================================

    for j = 1:n

        h = ...
            horizons(j);


        %% -------------------------------------------------
        % LONGITUDINAL MOTION
        %% -------------------------------------------------

        [futureX, ...
         futureSpeed, ...
         ~] = ...
            predict_ego_candidate_state( ...
                ego.x, ...
                ego.vx, ...
                targetSpeedKmh, ...
                h);


        trajectory.x(j) = ...
            futureX;


        trajectory.vx(j) = ...
            futureSpeed;


        %% -------------------------------------------------
        % LATERAL MOTION
        %% -------------------------------------------------

        if maneuverTime <= 0 || ...
           abs(deltaY) < 0.001

            trajectory.y(j) = ...
                ego.y;

            trajectory.vy(j) = 0;

            trajectory.ay(j) = 0;

            continue;

        end


        tau = ...
            min(h / maneuverTime, 1);


        %% Quintic smooth-step position

        blend = ...
            10*tau^3 - ...
            15*tau^4 + ...
            6*tau^5;


        trajectory.y(j) = ...
            ego.y + ...
            deltaY * blend;


        %% -----------------------------------------------
        % LATERAL VELOCITY
        %% -----------------------------------------------

        if h < maneuverTime

            dBlend = ...
                30*tau^2 - ...
                60*tau^3 + ...
                30*tau^4;


            trajectory.vy(j) = ...
                deltaY * ...
                dBlend / ...
                maneuverTime;


            %% -------------------------------------------
            % LATERAL ACCELERATION
            %% -------------------------------------------

            ddBlend = ...
                60*tau - ...
                180*tau^2 + ...
                120*tau^3;


            trajectory.ay(j) = ...
                deltaY * ...
                ddBlend / ...
                maneuverTime^2;

        else

            trajectory.vy(j) = 0;

            trajectory.ay(j) = 0;

        end

    end

end