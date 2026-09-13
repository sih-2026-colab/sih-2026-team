function trajectory=generate_2d_trajectory(ego,targetSpeedKmh,targetY,horizons,maneuverTime)
% Acceleration-limited longitudinal motion and quintic Hermite lateral motion.
% Preserve the measured velocity vector and current geometric curvature.
T=max(maneuverTime,.1);
initialVy=ego.vy; initialAy=0;
[~,~,initialAx]=predict_ego_candidate_state(ego.x,ego.vx,targetSpeedKmh,0);
if isfield(ego,'curvature') && abs(ego.vx)>1e-6
    speed=hypot(ego.vx,ego.vy);
    initialAy=(ego.curvature*speed^3+ego.vy*initialAx)/ego.vx;
end
b0=ego.y; b1=initialVy*T; b2=.5*initialAy*T^2; deltaY=targetY-ego.y;
b3=10*deltaY-6*b1-3*b2;
b4=-15*deltaY+8*b1+3*b2;
b5=6*deltaY-3*b1-b2;
trajectory=struct('time',horizons,'x',zeros(size(horizons)), ...
    'y',zeros(size(horizons)),'vx',zeros(size(horizons)), ...
    'vy',zeros(size(horizons)),'ay',zeros(size(horizons)), ...
    'targetSpeedKmh',targetSpeedKmh,'targetY',targetY,'maneuverTime',maneuverTime);
for j=1:numel(horizons)
    h=horizons(j); tau=min(h/T,1);
    [trajectory.x(j),trajectory.vx(j),~]=predict_ego_candidate_state(ego.x,ego.vx,targetSpeedKmh,h);
    trajectory.y(j)=b0+b1*tau+b2*tau^2+b3*tau^3+b4*tau^4+b5*tau^5;
    if h<T
        trajectory.vy(j)=(b1+2*b2*tau+3*b3*tau^2+4*b4*tau^3+5*b5*tau^4)/T;
        trajectory.ay(j)=(2*b2+6*b3*tau+12*b4*tau^2+20*b5*tau^3)/T^2;
    end
end
end
