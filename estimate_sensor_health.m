function health = ...
    estimate_sensor_health(environment)

    %% =====================================================
    % AUTONEX ADAPTIVE SENSOR HEALTH ESTIMATOR
    %
    % 0 = unusable
    % 1 = excellent
    %% =====================================================


    lightLevel = ...
        max(0,min(environment.lightLevel,1));

    visibility = ...
        max(0,min(environment.visibility,1));


    %% -----------------------------------------------------
    % RGB HEALTH
    %
    % Strongly depends on light and visibility
    %% -----------------------------------------------------

    rgbHealth = ...
        0.65 * lightLevel + ...
        0.35 * visibility;


    %% -----------------------------------------------------
    % RADAR HEALTH
    %
    % Radar does not depend strongly on visible light.
    %% -----------------------------------------------------

    if isfield(environment,'radarQuality')

        radarHealth = ...
            environment.radarQuality;

    else

        radarHealth = 0.95;

    end


    %% -----------------------------------------------------
    % THERMAL HEALTH
    %
    % Thermal is mostly independent of visible lighting.
    %% -----------------------------------------------------

    if isfield(environment,'thermalQuality')

        thermalHealth = ...
            environment.thermalQuality;

    else

        thermalHealth = 0.90;

    end


    %% Clamp

    rgbHealth = ...
        max(0,min(rgbHealth,1));

    radarHealth = ...
        max(0,min(radarHealth,1));

    thermalHealth = ...
        max(0,min(thermalHealth,1));


    %% Output

    health.rgb = rgbHealth;

    health.radar = radarHealth;

    health.thermal = thermalHealth;

end