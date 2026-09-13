function app=autonex_demo_card(app,id,m,complete)
c=autonex_demo_catalog(); c=c(strcmp({c.id},id));
app.demoIntro.Value={c.title;['CHALLENGE: ' c.challenge];['PRIMARY RISK: ' c.risk];['CAPABILITY: ' c.capability]};
if isempty(m), app.demoResult.Value={'READY / no samples'}; return; end
status='RUNNING';
if complete
    status='REVIEW / acceptance not certified for this live run';
    if m.collision || m.boundary || ~m.finite, status='FAILED / observed safety or finite-state check'; end
end
app.demoResult.Value={status; sprintf('Collision: %d   Boundary: %d',m.collision,m.boundary); ...
    sprintf('Minimum clearance: %.2f m',m.minClearance);sprintf('Distance travelled: %.2f m',m.distance); ...
    sprintf('Maximum steering: %.2f deg',m.maxSteeringDeg);sprintf('Braking events: %d',m.brakingEvents); ...
    sprintf('Replans (maneuver changes): %d',m.replans); ...
    'PASS requires the existing scenario acceptance suite; a clean demo alone is not a pass.'};
end
