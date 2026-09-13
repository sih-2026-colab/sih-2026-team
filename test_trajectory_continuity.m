function test_trajectory_continuity()
opts=autonex_options(); actors=create_highway_scenario(); ego=actors(1);
ego.vx=5*cos(-.3); ego.vy=5*sin(-.3); ego.curvature=tan(-.12)/opts.wheelbase;
for target=[ego.y ego.y-2 ego.y+2]
    p=generate_2d_trajectory(ego,20,target,[0 1e-5 2.5 3],2.5);
    assert(p.x(1)==ego.x && p.y(1)==ego.y);
    assert(abs(p.vx(1)-ego.vx)<1e-10 && abs(p.vy(1)-ego.vy)<1e-10);
    angle=atan2(p.y(2)-p.y(1),p.x(2)-p.x(1));
    assert(abs(angle-atan2(ego.vy,ego.vx))<1e-4,'Initial heading reset');
    [~,~,ax]=predict_ego_candidate_state(ego.x,ego.vx,20,0);
    curvature=(p.vx(1)*p.ay(1)-p.vy(1)*ax)/hypot(p.vx(1),p.vy(1))^3;
    assert(abs(curvature-ego.curvature)<1e-10,'Initial curvature reset');
    assert(abs(p.y(end)-target)<1e-9 && p.vy(end)==0 && p.ay(end)==0);
end
path=struct('x',ego.x+[0 3 6],'y',ego.y+[0 2 3]);
c=path_following_controller(ego,-.3,0,path,opts,[],false);
assert(abs(c.steeringRate)<=opts.maxSteeringRate+1e-9);
override=path_following_controller(ego,-.3,0,path,opts,[],true);
assert(abs(override.steeringAngle)<=opts.maxSteeringAngle+1e-9);
assert(abs(override.appliedSteeringRate)<=opts.maxSteeringRate+1e-9, ...
    'Guardian exceeded physical actuator rate');
assert(strcmp(cat_reflex_guardian(36,NaN),'EMERGENCY_BRAKE'));
assert(speed_tracking_controller(36,0,'EMERGENCY_BRAKE')==-6);
fprintf('TRAJECTORY_CONTINUITY_COMPONENTS_PASS\n');
end
