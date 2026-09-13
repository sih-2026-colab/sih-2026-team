function frame=autonex_sensor_frame(actors,environment,t,enabled,config)
% Sensor -> ego -> world, including mounting translation and turning velocity.
if nargin<5, config=struct; end
config=autonex_sensor_config(config);
frame=struct('time',t,'camera',struct([]),'radar',struct([]),'lidar',struct([]));
if enabled(1)
    [local,R,origin,~]=sensor_scene(actors,config.cameraPose,config.yawRate);
    frame.camera=simulate_rgb_camera(local(1),local,environment);
    sigma=.40+1.30*(1-environment.lightLevel)+1.00*(1-environment.visibility);
    for k=1:numel(frame.camera)
        p=R*[frame.camera(k).x;frame.camera(k).y]+origin;
        frame.camera(k).x=p(1); frame.camera(k).y=p(2);
        frame.camera(k).covariance=sigma^2*eye(2);
    end
end
if enabled(2)
    names={'FRONT_RADAR','REAR_RADAR','LEFT_CORNER_RADAR','RIGHT_CORNER_RADAR'};
    fovs=[120 120 100 100]; ranges=[150 100 80 80];
    for sensor=1:4
        [local,R,origin,sensorVelocity]=sensor_scene(actors,config.radarPoses(sensor,:),config.yawRate);
        raw=simulate_mmwave_radar(local(1),local,names{sensor},0,fovs(sensor),ranges(sensor));
        if environment.radarQuality<.5 && ~isempty(raw), raw=raw(rand(size(raw))>.5); end
        for k=1:numel(raw)
            u=R*[cosd(raw(k).angle);sind(raw(k).angle)]; tangent=[-u(2);u(1)]; range=raw(k).range;
            p=origin+range*u;
            J=[u range*tangent]; P=J*diag([.15^2 deg2rad(.5)^2])*J'+1e-4*eye(2);
            velocity=(raw(k).radialVelocity+dot(sensorVelocity,u))*u;
            V=.15^2*(u*u')+1e4*(tangent*tangent');
            row=struct('x',p(1),'y',p(2),'covariance',P,'velocity',velocity, ...
                'velocityCovariance',V,'confidence',environment.radarQuality, ...
                'sensor',raw(k).sensor,'range',range,'angle',raw(k).angle, ...
                'radialVelocity',raw(k).radialVelocity,'sensorOrigin',origin, ...
                'sensorVelocity',sensorVelocity,'lineOfSight',u);
            if isempty(frame.radar), frame.radar=row; else, frame.radar(end+1)=row; end %#ok<AGROW>
        end
    end
end
if enabled(3), frame.lidar=simulate_autonex_lidar_objects(actors,config); end
end
function [local,R,origin,velocity]=sensor_scene(actors,pose,yawRate)
ego=actors(1); yaw=ego.heading;
E=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)]; offset=E*pose(1:2)';
origin=[ego.x;ego.y]+offset;
velocity=[ego.vx;ego.vy]+yawRate*[-offset(2);offset(1)];
angle=yaw+pose(3); R=[cos(angle) -sin(angle);sin(angle) cos(angle)];
local=actors;
for k=1:numel(actors)
    p=R'*([actors(k).x;actors(k).y]-origin); v=R'*[actors(k).vx;actors(k).vy];
    local(k).x=p(1); local(k).y=p(2); local(k).vx=v(1); local(k).vy=v(2);
    local(k).heading=actors(k).heading-angle;
end
v=R'*velocity;
local(1).x=0; local(1).y=0; local(1).vx=v(1); local(1).vy=v(2); local(1).heading=0;
end
