function test_rear_risk()
opts=autonex_options(); actors=create_highway_scenario(); ego=actors(1);
ego.x=0; ego.y=7; ego.vx=20; ego.vy=0;
track=struct('TrackID',42,'State',[-30;19;7;0], ...
    'StateCovariance',eye(4)*.01);
r=estimate_rear_risk(track,ego,0,opts);
assert(~r.rearRiskActive,'A non-closing rear track must not trigger on constant speed.');
r=estimate_rear_risk(track,ego,-6,opts);
assert(r.rearRiskActive && r.rearThreatActorId==42 && r.rearPredictedGap<0);
assert(~r.rearImpactUnavoidable,'Sampled predictions cannot prove inevitability.');
track.State(3)=0;
r=estimate_rear_risk(track,ego,-6,opts);
assert(~r.rearRiskActive,'A non-overlapping rear path must not trigger.');
track.State=[20;30;7;0];
r=estimate_rear_risk(track,ego,-6,opts);
assert(~r.rearRiskActive,'An ahead actor cannot be a rear threat.');
track.State=[-30;25;7;0];
r=estimate_rear_risk(track,ego,0,opts);
assert(r.rearRiskActive && abs(r.rearTTC-25.5/5)<1e-10);
ego.vy=4; track.State(3)=ego.y+2.35; track.State(4)=4;
r=estimate_rear_risk(track,ego,0,opts);
assert(r.rearRiskActive,'A rotated footprint must not clear a still-overlapping rear threat.');
ego.vy=0;
% Even severe rear pressure cannot authorize an immediate front conflict.
ego.vx=5; track.State=[3;0;7;0];
rear=track; rear.TrackID=43; rear.State=[-10;20;7;0];
fused=struct('trackID',42,'fusedConfidence',1,'fusedUncertainty',0);
rearFused=fused; rearFused.trackID=43;
drivable=struct('xValues',-5:70,'yValues',-1:.25:8);
safeSpace=struct('xResolution',1,'yResolution',.25, ...
    'safeFreeMask',true(numel(drivable.yValues),numel(drivable.xValues)));
corridor=struct('valid',true,'x',0:70,'referenceY',7*ones(1,71), ...
    'lowerY',zeros(1,71),'upperY',7*ones(1,71));
[choice,diagnostic]=evaluate_minimum_risk_action(ego,0,0,[track rear], ...
    [fused rearFused],corridor,drivable,safeSpace,[],opts);
assert(isempty(choice) && diagnostic.feasible==0, ...
    'Rear pressure must never bypass the hard front gate.');
% Unknown/occupied space must also block otherwise generated escape paths.
safeSpace.safeFreeMask(:)=false;
[choice,~]=evaluate_minimum_risk_action(ego,0,0,[],[], ...
    corridor,drivable,safeSpace,[],opts);
assert(isempty(choice),'Occupied/unknown escape space must be rejected.');
% A partially open corridor can reduce rear exposure before a full lateral
% escape becomes visible. Centered driving must not win a flat binary tie.
ego.vx=20; track.State=[-10;25;7;0];
safeSpace.safeFreeMask=repmat(drivable.yValues(:)>=5.25 & drivable.yValues(:)<=7,1,numel(drivable.xValues));
corridor.lowerY(:)=5.25;
[choice,~]=evaluate_minimum_risk_action(ego,0,0,track,fused, ...
    corridor,drivable,safeSpace,[],opts);
assert(~isempty(choice) && choice.targetY<6.8, ...
    'A verified partial reposition must receive credit for reducing rear overlap.');
fprintf('REAR_RISK_COMPONENTS_PASS\n');
end
