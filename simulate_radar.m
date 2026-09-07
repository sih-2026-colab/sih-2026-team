function measurement = simulate_radar(pedX, pedY)

    % Simulated radar measurement noise
    sigmaX = 0.5;    % metres
    sigmaY = 0.4;

    measuredX = pedX + sigmaX * randn;
    measuredY = pedY + sigmaY * randn;

    measurement = [measuredX; measuredY];

end