clear;
clc;
close all;

pedX = 35;
pedY = 3.5;

N = 100;

measurements = zeros(N,2);

for i = 1:N

    z = simulate_radar(pedX,pedY);

    measurements(i,1) = z(1);
    measurements(i,2) = z(2);

end

figure;
hold on;
grid on;

scatter(measurements(:,1),measurements(:,2));

plot(pedX,pedY,'x', ...
    'MarkerSize',15, ...
    'LineWidth',3);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('AutoNex — Simulated Radar Measurements');

legend('Radar Measurements','True Position');