function metrics=validate_closed_loop_path_following()
% End-to-end traversal of both unchanged staggered missing_lane blockers.
test_closed_loop_path_following;
test_trajectory_continuity;
before=jsondecode(fileread('results/trajectory_continuity_before_metrics.json'));
previous=jsondecode(fileread('results/trajectory_continuity_before.json'));
before.maxSteeringRateDegrees=rad2deg(max(abs(diff([previous.samples.steeringAngle])))/previous.options.dt);
opts=autonex_options(struct('scenario','missing_lane','duration',25, ...
    'showLaneMarkings',false,'reportFile','results/closed_loop_path_following.json'));
r=run_autonex_simulation(opts); s=r.samples;
steering=[s.steeringAngle]; lateral=[s.lateralError];
turns=sign(steering(abs(steering)>deg2rad(2)));
[~,firstBlocker]=min(abs([s.x]-28)); [~,secondBlocker]=min(abs([s.x]-48));
metrics=struct('distanceTravelled',r.distance,'minimumClearance',r.minClearance, ...
    'maxSteeringAngleDegrees',rad2deg(max(abs(steering))), ...
    'maxSteeringRateDegrees',rad2deg(max(abs([s.steeringRate]))), ...
    'maxLateralError',max(abs(lateral)),'rmsLateralError',sqrt(mean(lateral.^2)), ...
    'replans',sum(~strcmp({s(1:end-1).selectedName},{s(2:end).selectedName})), ...
    'emergencySteps',r.emergencySteps,'collisionCount',sum([s.collision]), ...
    'boundaryViolations',sum([s.boundaryViolation]), ...
    'steeringReversals',sum(diff(turns)~=0),'lateralExcursion',range([s.y]), ...
    'passedBothBlockers',s(end).x>52.5, ...
    'yAtUpperBlocker',s(firstBlocker).y,'yAtLowerBlocker',s(secondBlocker).y);
fid=fopen('results/closed_loop_path_following_metrics.json','w');
fprintf(fid,'%s',jsonencode(metrics,PrettyPrint=true)); fclose(fid); disp(metrics);
comparison=struct('before',before,'after',metrics,'clearanceChange',r.minClearance-before.minimumClearance);
fid=fopen('results/trajectory_continuity_comparison.json','w');
fprintf(fid,'%s',jsonencode(comparison,PrettyPrint=true)); fclose(fid);
assert(r.distance>=.95*before.distanceTravelled,'More than 5 percent progress regression');
assert(metrics.rmsLateralError<=.10,'Tracking error regression');
assert(r.minClearance>=before.minimumClearance-1e-4,'Clearance sacrificed');
assert(~r.collision && ~r.boundaryViolation,'Collision or road departure');
assert(metrics.passedBothBlockers,'Vehicle did not pass both blockers');
assert(metrics.lateralExcursion>1,'No meaningful lateral response');
% Direction is required; prescribing a half-metre second shift would reject
% a path that already entered the safe shared space with greater clearance.
assert(s(firstBlocker).y<s(1).y-1 && s(secondBlocker).y>s(firstBlocker).y+1e-3, ...
    'Ego did not follow both changes in the safe corridor');
assert(max(abs(steering))<=opts.maxSteeringAngle+1e-10,'Steering limit');
assert(metrics.steeringReversals<=10,'Severe steering oscillation');
assert(r.minClearance>=.5,'Insufficient clearance');
assert(all(isfinite([s.x s.y s.speed s.egoYaw s.steeringAngle s.lateralError])),'Nonfinite state');
assert(all(isfinite([s.rawSteeringAngle s.limitedSteeringAngle s.steeringRate])),'Nonfinite steering telemetry');
assert(all([s([s.emergency]).targetSpeed]==0),'Emergency target overridden');
% Exercise authoritative no-path braking with missing sensor frames as well.
e=run_autonex_simulation(struct('scenario','tracking_loss','duration',2));
assert(any([e.samples.emergency]),'Emergency guardian unavailable');
assert(all([e.samples([e.samples.emergency]).ax]<=0),'Emergency accelerated');
regression=validate_corridor_stability();
assert(~regression.collision && ~regression.boundaryViolation,'Corridor safety regression');
assert(regression.jitter.stabilizedTotalVariation<=regression.jitter.rawTotalVariation+1e-8,'Corridor jitter regression');
fprintf('CLOSED_LOOP_PATH_FOLLOWING_PASS\n');
end
