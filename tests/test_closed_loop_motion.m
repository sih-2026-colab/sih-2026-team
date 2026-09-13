function test_closed_loop_motion()
% Headless component and real-planner integration tests. SI motion units.
opts=autonex_options(struct('useLearnedIntent',false));
actors=create_highway_scenario(); start=actors(1); start.x=0; start.y=0;
start.vx=4; start.vy=0; start.heading=0;
straight=struct('x',0:.25:80,'y',zeros(1,321));
ego=start; yaw=0; steering=0;
for k=1:100
    c=path_following_controller(ego,yaw,steering,straight,opts);
    [ego,yaw,m]=update_ego_bicycle(ego,yaw,0,c.steeringAngle,opts);
    steering=m.steering;
end
assert(abs(ego.x-20)<1e-9 && abs(ego.y)<1e-9 && abs(yaw)<1e-9);
fprintf('A STRAIGHT_PASS steps=100 distance=%.3f m\n',ego.x);
theta=0:.005:2; curve=struct('x',20*sin(theta),'y',20*(1-cos(theta)));
ego=start; yaw=0; steering=0; errors=zeros(1,140);
for k=1:140
    c=path_following_controller(ego,yaw,steering,curve,opts);
    [ego,yaw,m]=update_ego_bicycle(ego,yaw,0,c.steeringAngle,opts);
    assert(abs(m.steering-steering)<=opts.maxSteeringRate*opts.dt+1e-10);
    steering=m.steering; errors(k)=abs(hypot(ego.x,ego.y-20)-20);
end
assert(max(errors)<.35 && ego.y>10 && yaw>1);
fprintf('B CURVED_PASS steps=140 max_radial_error=%.4f m yaw=%.3f rad\n',max(errors),yaw);
sharp=struct('x',[0 .1 .2],'y',[0 2 4]);
c=path_following_controller(start,0,opts.maxSteeringAngle,sharp,opts,[],true);
assert(c.steeringSaturation && abs(c.steeringAngle-opts.maxSteeringAngle)<1e-10);
c=path_following_controller(start,0,0,sharp,opts,[],true);
assert(c.steeringRateSaturation && abs(c.steeringRate-opts.maxSteeringRate)<1e-10);
fprintf('C D STEERING_ANGLE_AND_RATE_PASS\n');
limited=autonex_options(struct('maxAcceleration',.7,'maxDeceleration',2));
assert(speed_tracking_controller(0,100,'MICRO_ACCELERATE',limited)==.7);
assert(speed_tracking_controller(100,0,'CONTROLLED_BRAKE',limited)==-2);
assert(speed_tracking_controller(100,0,'EMERGENCY_BRAKE',limited)==-2);
[~,~,m]=update_ego_bicycle(start,0,100,0,limited);
assert(m.acceleration==.7 && abs(m.speed-4-.7*limited.dt)<1e-10);
ego=start; yaw=0;
for k=1:60
    [ego,yaw,m]=update_ego_bicycle(ego,yaw,-100,0,limited);
    assert(m.acceleration>=-2 && m.speed>=0);
end
assert(m.speed==0 && abs(ego.x-4)<1e-9);
% Fractional final step must integrate only until stopping, not the full dt.
ego=start; ego.vx=.03;
[ego,~,m]=update_ego_bicycle(ego,0,-100,0,limited);
assert(m.speed==0 && abs(ego.x-.03^2/4)<1e-12);
% Braking is based on speed, not the sign of world-frame vx.
ego=start; ego.vx=-4;
[~,~,m]=update_ego_bicycle(ego,pi,-2,0,limited);
assert(m.speed<4 && m.acceleration==-2);
fprintf('E ACCELERATION_BRAKING_PASS stop_distance=4.000 m fractional_stop=PASS\n');
run_closed_loop_demo();
fprintf('CLOSED_LOOP_MOTION_ALL_PASS\n');
end
