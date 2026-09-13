function test_closed_loop_path_following()
opts=autonex_options(); actors=create_highway_scenario(); ego=actors(1);
ego.vx=5; ego.vy=0;
path=struct('x',ego.x+[0 5 10 15],'y',ego.y+[0 -.5 -2 -3]);
c=path_following_controller(ego,0,0,path,opts);
assert(c.steeringAngle<0 && c.targetHeading<0,'Safe path has no steering influence');
[moved,yaw]=update_ego_bicycle(ego,0,0,c.steeringAngle,opts);
assert(moved.y<ego.y && yaw<0 && moved.vy<0,'Steering does not affect motion');
assert(abs(hypot(moved.vx,moved.vy)-5)<1e-10,'Steering changed scalar speed');
empty=path_following_controller(ego,0,0,[],opts);
assert(empty.steeringAngle==0,'Empty path generated a maneuver');
braking=ego;
for k=1:50, [braking,~]=update_ego_bicycle(braking,0,-6,0,opts); end
assert(hypot(braking.vx,braking.vy)==0,'Emergency braking failed to stop');
assert(abs(c.steeringAngle)<=opts.maxSteeringRate*opts.dt,'Steering rate unbounded');
fprintf('PATH_FOLLOWING_COMPONENTS_PASS\n');
end
