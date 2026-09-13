function m=autonex_demo_metrics(m,out)
% Bounded, observational aggregation. Distance is sampled ego path length.
if isempty(m)
    m=struct('lastTime',-inf,'lastX',out.x,'lastY',out.y,'distance',0, ...
        'collision',false,'boundary',false,'minClearance',inf,'maxSteeringDeg',0, ...
        'brakingEvents',0,'replans',0,'previousBrake',false,'previousName','', ...
        'finite',true,'samples',0);
end
if out.time<=m.lastTime, return; end
m.distance=m.distance+hypot(out.x-m.lastX,out.y-m.lastY);
m.lastX=out.x; m.lastY=out.y; m.lastTime=out.time;
m.collision=m.collision || out.collision; m.boundary=m.boundary || out.boundaryViolation;
m.minClearance=min(m.minClearance,out.minClearance);
m.maxSteeringDeg=max(m.maxSteeringDeg,abs(rad2deg(out.steeringAngle)));
brake=out.ax<0; m.brakingEvents=m.brakingEvents+double(brake && ~m.previousBrake);
% Same selected-name-change count used by validate_autonex_scenarios.
m.replans=m.replans+double(m.samples>0 && ~strcmp(m.previousName,out.selectedName));
m.previousBrake=brake; m.previousName=out.selectedName; m.samples=m.samples+1;
m.finite=m.finite && all(isfinite([out.x out.y out.speed out.egoYaw out.steeringAngle out.ax]));
end
