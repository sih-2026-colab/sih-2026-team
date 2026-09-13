function [ego,yaw,motion]=update_ego_bicycle(ego,yaw,acceleration,steering,opts)
% Midpoint kinematic bicycle integration; scalar speed cannot reverse.
% The path controller enforces steering slew; enforce hard plant limits too.
acceleration=max(-opts.maxDeceleration,min(opts.maxAcceleration,acceleration));
steering=max(-opts.maxSteeringAngle,min(opts.maxSteeringAngle,steering));
speed=hypot(ego.vx,ego.vy);
if speed==0 && acceleration<0, acceleration=0; end
movingDt=opts.dt;
if acceleration<0, movingDt=min(movingDt,speed/(-acceleration)); end
nextSpeed=max(0,speed+acceleration*movingDt);
meanSpeed=(speed+nextSpeed)/2;
yawRate=meanSpeed/opts.wheelbase*tan(steering);
middleYaw=yaw+yawRate*movingDt/2;
ego.x=ego.x+meanSpeed*cos(middleYaw)*movingDt;
ego.y=ego.y+meanSpeed*sin(middleYaw)*movingDt;
yaw=atan2(sin(yaw+yawRate*movingDt),cos(yaw+yawRate*movingDt));
ego.vx=nextSpeed*cos(yaw); ego.vy=nextSpeed*sin(yaw);
ego.heading=yaw;
% Retain the existing actor schema; expose explicit SI motion state separately.
ego.ax=acceleration; ego.ay=nextSpeed^2/opts.wheelbase*tan(steering);
motion=struct('x',ego.x,'y',ego.y,'yaw',yaw,'speed',nextSpeed, ...
    'steering',steering,'acceleration',acceleration);
end
