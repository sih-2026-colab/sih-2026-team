function test_steering_actuator()
opts=autonex_options(struct('enforcePhysicalSteeringRate',true));
actors=create_highway_scenario(); ego=actors(1); ego.vx=5; ego.vy=0;
path=struct('x',ego.x+[0 3 6],'y',ego.y+[0 -2 -3]);
for guardian=[false true]
    c=path_following_controller(ego,0,0,path,opts,[],guardian);
    assert(abs(c.appliedSteeringRate)<=opts.maxSteeringRate+1e-10);
    assert(abs(c.appliedSteeringAngle)<=opts.maxSteeringAngle+1e-10);
    assert(c.steeringRateSaturation && abs(c.requestedSteeringRate)>opts.maxSteeringRate);
    assert(c.guardianOverrideActive==guardian);
end
% STOP requests neutral steering, but must not teleport from a turned wheel.
previous=deg2rad(20);
c=path_following_controller(ego,0,previous,[],opts,[],true);
assert(c.requestedSteeringAngle==0 && c.appliedSteeringAngle>0);
assert(abs(c.appliedSteeringAngle-(previous-opts.maxSteeringRate*opts.dt))<1e-10);
% A newly tighter comfort envelope cannot circumvent physical slew either.
ego.vx=20;
c=path_following_controller(ego,0,previous,path,opts,[],false);
assert(abs(c.appliedSteeringRate)<=opts.maxSteeringRate+1e-10);
assert(speed_tracking_controller(36,0,'EMERGENCY_BRAKE')==-6);
fprintf('STEERING_ACTUATOR_COMPONENTS_PASS\n');
end
