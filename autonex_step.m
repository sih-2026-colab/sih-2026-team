function [state,out] = autonex_step(state,t,opts)
% One shared closed-loop step for MATLAB, animation, validation and Simulink.
if isempty(state)
    rng(opts.seed); state.actors=create_highway_scenario();
    if strcmp(opts.scenario,'original_close_range')
        state.actors(2).x=-18;
        state.actors(3).x=4.1;
    elseif strcmp(opts.scenario,'close_rear_cut_in')
        state.actors(2).x=-18;
    elseif strcmp(opts.scenario,'stationary_obstacle')
        state.actors(3).x=55; state.actors(3).y=7; state.actors(3).vx=0; state.actors(3).vy=0;
        state.actors(2).x=-80;
    elseif strcmp(opts.scenario,'clear_road')
        for k=2:numel(state.actors), state.actors(k).x=200+k*30; state.actors(k).vy=0; end
    end
    state.actors=configure_autonex_scenario(state.actors,opts.scenario);
    state.egoYaw=atan2(state.actors(1).vy,state.actors(1).vx); state.steeringAngle=0; state.previousPath=[];
    state.tracker=create_autonex_gnn_tracker(opts.trackerModel); state.selection=[];
    state.lastDetectionTime=0; state.mapTime=-inf; state.mapMemory=[];
    state.corridorStab=struct('initialized',false,'previousReferenceY',NaN, ...
        'lastUpdateTime',-inf,'mode','RESET','rawReferenceY',NaN);
    state.trackerInitialized=false; state.rearManeuver=[];
    state.intentModel=[]; state.intentSource='HEURISTIC';
    modelFile=fullfile(fileparts(mfilename('fullpath')),'models','autonex_intent_model.mat');
    if opts.useLearnedIntent && isfile(modelFile)
        loaded=load(modelFile,'model'); state.intentModel=loaded.model;
        state.intentSource='HYBRID_SYNTHETIC';
    end
end
ego=state.actors(1);
ego.curvature=tan(state.steeringAngle)/opts.wheelbase;
environment=struct('lightLevel',1,'visibility',1,'radarQuality',.95,'thermalQuality',.85);
if strcmp(opts.scenario,'degraded_sensing')
    environment.lightLevel=.1; environment.visibility=.2;
    environment.radarQuality=.45; environment.thermalQuality=.5;
end
if strcmp(opts.scenario,'night_pedestrian')
    environment.lightLevel=.02; environment.visibility=.5;
end
world=struct([]);
if strcmp(opts.perceptionMode,'camera_radar_lidar')
    enabled=logical(opts.sensorAvailability);
    assert(numel(enabled)==3,'Expected Camera/Radar/LiDAR availability');
    streamAvailable=enabled(2) && ~(strcmp(opts.scenario,'tracking_loss') && t>=1 && t<=3);
    enabled(2)=streamAvailable;
    sensorConfig=autonex_sensor_config(opts.sensorConfig);
    sensorConfig.yawRate=hypot(ego.vx,ego.vy)*ego.curvature;
    frame=autonex_sensor_frame(state.actors,environment,t,enabled,sensorConfig);
    if ~isfield(state,'perceptionMemory'), state.perceptionMemory=[]; end
    [state.perceptionMemory,tracks,fused,world]=update_autonex_world_model(state.perceptionMemory,frame,opts.trackerModel);
    state.worldModel=world;
    if streamAvailable, state.lastDetectionTime=t; end
    radar=frame.radar; rgb=frame.camera; thermal=struct([]);
else
assert(strcmp(opts.perceptionMode,'legacy'),'Unknown perception mode');
radar=scan_autonex_radar_suite(state.actors);
streamAvailable=~(strcmp(opts.scenario,'tracking_loss') && t>=1 && t<=3);
if ~streamAvailable, radar=radar([]); end
if strcmp(opts.scenario,'degraded_sensing') && ~isempty(radar)
    radar=radar(rand(size(radar))>.5);
end
detections=radar_to_object_detections(radar,ego,t);
% An empty valid scan is different from a missing sensor frame.
if streamAvailable, state.lastDetectionTime=t; end
tracks=[];
if ~isempty(detections) || state.trackerInitialized
    [tracks,~,~]=state.tracker(detections,t);
    state.trackerInitialized=true;
end
rgb=simulate_rgb_camera(ego,state.actors,environment);
thermal=simulate_thermal_ir(ego,state.actors);
fused=build_fused_track_perception(tracks,rgb,thermal,environment);
end
intent=[]; referenceY=ego.y;
if opts.laneIndependent
    if t-state.mapTime>=.25-1e-8
        [state.drivable,~,~]=build_autonex_safe_space(state.actors);
        [state.drivable,state.mapMemory]=update_observed_free_space(state.drivable,state.mapMemory,tracks,t);
        state.safeSpace=inflate_drivable_space_for_vehicle(state.drivable,4.5,1.9);
        state.corridor=extract_safe_corridor(state.drivable,state.safeSpace,ego);
        state.mapTime=t;
    end
    corridor=state.corridor;
    if corridor.valid
        lookX=min(corridor.x(end),max(corridor.x(1),ego.x+10));
        rawReferenceY=interp1(corridor.x,corridor.referenceY,lookX);
        [referenceY,state.corridorStab]=stabilize_corridor_reference( ...
            rawReferenceY,state.safeSpace,state.drivable,ego,state.corridorStab,t,lookX);
        if ~isfinite(referenceY), referenceY=rawReferenceY; end
    else
        rawReferenceY=NaN;
    end
    if ~isfield(state,'corridorStab'), state.corridorStab=struct('initialized',false); end
    stabMode='NO_CORRIDOR';
    if isfield(state.corridorStab,'mode'), stabMode=state.corridorStab.mode; end
    rawRefOut=NaN; if isfield(state.corridorStab,'rawReferenceY'), rawRefOut=state.corridorStab.rawReferenceY; end
    if isfield(state.corridorStab,'rawReferenceY') && isfinite(rawReferenceY), rawRefOut=rawReferenceY; end
    expandedSearch=isfield(opts,'expandedSearch') && opts.expandedSearch;
    candidates=generate_corridor_candidates(ego,0:.1:3,corridor,state.drivable,state.safeSpace,expandedSearch);
else
    corridor=struct('valid',false,'length',0,'reason','LEGACY_LANE_REFERENCE');
    referenceY=7; candidates=generate_2d_candidate_set(ego,0:.1:3,referenceY);
end
if ~isempty(tracks)
    [value,found]=find_highest_cutin_risk(tracks,ego,referenceY,state.intentModel);
    if found, intent=value; end
end
selected=[]; index=NaN; mode='NO_SAFE_SPACE'; stability='RESET'; guardian='NO_SAFE_SPACE';
results=struct([]); guardianResults=struct([]);
if ~isempty(candidates)
    results=evaluate_2d_trajectories(candidates,tracks,fused,ego,intent,-1.75,8.75);
    [proposal,index,mode]=select_best_2d_trajectory(candidates,results);
    [selected,index,mode,state.selection,stability]=stabilize_trajectory_selection( ...
        candidates,results,proposal,index,mode,state.selection,t);
    [selected,index,guardian,guardianResults]=cat_reflex_2d_guardian(candidates,results,selected,index,tracks,-1.75,8.75);
    if strcmp(guardian,'OVERRIDE_MINIMUM_RISK')
        selected=[];
        index=NaN;
    end
    if ~isempty(selected) && ~strcmp(guardian,'APPROVED')
        state.selection.initialized=true; state.selection.name=selected.name;
        state.selection.targetSpeedKmh=selected.targetSpeedKmh; state.selection.lastSwitchTime=t;
    end
end
targetSpeed=0; targetY=ego.y; name='STOP'; emergency=isempty(selected);
if ~isempty(selected)
    targetSpeed=selected.targetSpeedKmh; targetY=selected.targetY; name=selected.name;
end
% A stale radar stream cannot authorize continued cruise from coasted tracks.
if t-state.lastDetectionTime>.4
    emergency=true; guardian='SENSOR_TIMEOUT';
end
% Explicit stop for a stationary lead obstacle inside braking distance.
% Uses tracks only; actor truth is reserved for evaluation below.
for k=1:numel(tracks)
    s=tracks(k).State; dx=s(1)-ego.x;
    closing=ego.vx-s(2);
    if dx>0 && abs(s(3)-ego.y)<2 && closing>0 && dx<closing^2/12+6
        emergency=true; guardian='BRAKING_DISTANCE';
    end
end
if emergency
    targetSpeed=0; targetY=ego.y; selected=[]; name='STOP';
    command='EMERGENCY_BRAKE'; state.selection=[];
else
    command=cat_reflex_guardian(hypot(ego.vx,ego.vy)*3.6,targetSpeed);
    if strcmp(command,'REPLAN') && isfield(opts,'recoverCruise') && opts.recoverCruise
        command='MICRO_ACCELERATE';
    end
end
ax=speed_tracking_controller(hypot(ego.vx,ego.vy)*3.6,targetSpeed,command,opts);
rearRisk=estimate_rear_risk(tracks,ego,min(ax,-opts.maxDeceleration),opts);
minimumRiskMode='NORMAL';
if ax<0, minimumRiskMode='FRONT_BRAKE'; end
if emergency, minimumRiskMode='EMERGENCY_STOP'; end
rearActionDiagnostics=struct('tested',0,'feasible',0,'bestRearCost',inf, ...
    'baselineRearConflict',NaN,'candidateRearConflict',NaN,'rearConflictReduction',NaN, ...
    'rearRiskCost',NaN,'comfortCost',NaN,'progressCost',NaN,'totalScore',NaN, ...
    'candidateScores',struct([]));
% Immediate front-braking and sensor-timeout authority remain unconditional.
if rearRisk.rearRiskActive && opts.laneIndependent && ...
        ~ismember(guardian,{'BRAKING_DISTANCE','SENSOR_TIMEOUT'}) && (ax<0 || emergency)
    [mitigation,rearActionDiagnostics]=evaluate_minimum_risk_action(ego,state.egoYaw, ...
        state.steeringAngle,tracks,fused,corridor,state.drivable,state.safeSpace,selected,opts,state.rearManeuver);
    if ~isempty(mitigation)
        selected=mitigation; targetSpeed=mitigation.targetSpeedKmh;
        targetY=mitigation.targetY; name=mitigation.name; emergency=false;
        command=cat_reflex_guardian(hypot(ego.vx,ego.vy)*3.6,targetSpeed);
        ax=speed_tracking_controller(hypot(ego.vx,ego.vy)*3.6,targetSpeed,command,opts);
        minimumRiskMode=mitigation.minimumRiskMode;
        guardian='REAR_MITIGATION_FRONT_VERIFIED'; state.selection=[];
    end
end
if strcmp(minimumRiskMode,'LATERAL_ESCAPE')
    state.rearManeuver=struct('targetY',selected.targetY, ...
        'remainingTime',max(0,selected.maneuverTime-opts.dt));
elseif ~isempty(state.rearManeuver)
    state.rearManeuver.remainingTime=max(0,state.rearManeuver.remainingTime-opts.dt);
    if state.rearManeuver.remainingTime==0, state.rearManeuver=[]; end
end
path=[]; if ~isempty(selected), path=selected.trajectory; end
control=path_following_controller(ego,state.egoYaw,state.steeringAngle,path,opts,state.previousPath,emergency || ~strcmp(guardian,'APPROVED'));
state.previousPath=path;
state.steeringAngle=control.steeringAngle;
ay=hypot(ego.vx,ego.vy)^2/opts.wheelbase*tan(control.steeringAngle);
if hypot(ego.vx,ego.vy)==0 && ax<0, ax=0; end
state.actors(1).ax=ax; state.actors(1).ay=ay;
out=struct('time',t,'speed',hypot(ego.vx,ego.vy)*3.6,'targetSpeed',targetSpeed,'targetY',targetY, ...
    'selectionMode',mode,'selectedName',name,'stabilityMode',stability,'guardianMode',guardian, ...
    'pCut',0,'x',ego.x,'y',ego.y,'ax',ax,'ay',ay,'trackCount',numel(tracks), ...
    'corridorLength',corridor.length,'candidateCount',numel(candidates),'emergency',emergency);
out.selectedPathY=control.selectedPathY; out.lateralError=control.lateralError;
out.targetHeading=control.targetHeading; out.steeringAngle=control.steeringAngle;
out.egoYaw=state.egoYaw;
out.longitudinalCommand=command;
out.motionMode='CRUISE';
if contains(stability,'SWITCH')
    out.motionMode='REPLAN';
end
if ax<0, out.motionMode='BRAKE'; end
if emergency && hypot(ego.vx,ego.vy)==0, out.motionMode='STOP'; end
rearFields=fieldnames(rearRisk);
for d=1:numel(rearFields), out.(rearFields{d})=rearRisk.(rearFields{d}); end
out.rearThreatIdSource='TRACK_ID'; out.minimumRiskMode=minimumRiskMode;
out.rearActionDiagnostics=rearActionDiagnostics;
scoreFields={'baselineRearConflict','candidateRearConflict','rearConflictReduction', ...
    'rearRiskCost','comfortCost','progressCost','totalScore'};
for d=1:numel(scoreFields), out.(scoreFields{d})=rearActionDiagnostics.(scoreFields{d}); end
if isfield(opts,'rearAuditTimes') && any(abs(t-opts.rearAuditTimes)<1e-8)
    out.rearCandidateAudit=audit_rear_candidate_gates(ego,tracks,fused,corridor, ...
        state.drivable,state.safeSpace,opts);
end
if isfield(opts,'validationTelemetry') && opts.validationTelemetry
    out.perceptionAudit=autonex_perception_audit(tracks,fused,ego,environment);
    out.longitudinalCommand=command;
end
out.rawSteeringAngle=control.rawSteeringAngle;
out.limitedSteeringAngle=control.steeringAngle; out.steeringRate=control.steeringRate;
diagnostics={'requestedSteeringAngle','appliedSteeringAngle','requestedSteeringRate', ...
    'appliedSteeringRate','steeringSaturation','steeringRateSaturation','guardianOverrideActive'};
for d=1:numel(diagnostics), out.(diagnostics{d})=control.(diagnostics{d}); end
if opts.laneIndependent
    out.rawCorridorReferenceY=rawRefOut;
    out.stabilizedCorridorReferenceY=referenceY;
    out.corridorStabilityMode=stabMode;
else
    out.rawCorridorReferenceY=NaN;
    out.stabilizedCorridorReferenceY=NaN;
    out.corridorStabilityMode='LEGACY';
end
if ~isempty(intent), out.pCut=intent.probability*100; end
out.rgbDetections=numel(rgb); out.thermalDetections=numel(thermal);
out.intentSource=state.intentSource;
out.surfaceHazardCells=0;
if opts.laneIndependent, out.surfaceHazardCells=state.drivable.surfaceHazards.hazardCells; end
out.actors=state.actors; out.candidates=candidates; out.selected=selected; out.tracks=tracks;
out.worldModel=world;
% Optional read-only judge telemetry: expose already computed results.
if isfield(opts,'explainabilityTelemetry') && opts.explainabilityTelemetry
    out.explainability=struct('plannerResults',results,'guardianResults',guardianResults, ...
        'fused',fused,'ego',ego,'environment',environment,'intent',intent, ...
        'sensorAvailability',opts.sensorAvailability,'sensorConfig',opts.sensorConfig);
end
if isfield(opts,'sensorTelemetry') && opts.sensorTelemetry && strcmp(opts.perceptionMode,'camera_radar_lidar')
    out.sensorFrame=frame;
end
out.collision=false; out.minClearance=inf;
if isfield(opts,'validationTelemetry') && opts.validationTelemetry
    out.collisionActorIDs=[];
end
% Conservative axis-aligned bounds of the yawed ego footprint.
egoHalfX=2.25*abs(cos(state.egoYaw))+.95*abs(sin(state.egoYaw));
egoHalfY=2.25*abs(sin(state.egoYaw))+.95*abs(cos(state.egoYaw));
for k=2:numel(state.actors)
    other=state.actors(k);
    [L,W]=autonex_actor_size(other);
    dx=abs(other.x-ego.x)-egoHalfX-L/2; dy=abs(other.y-ego.y)-egoHalfY-W/2;
    out.collision=out.collision || (dx<0 && dy<0);
    if dx<0 && dy<0 && isfield(out,'collisionActorIDs')
        out.collisionActorIDs(end+1)=other.id;
    end
    out.minClearance=min(out.minClearance,hypot(max(dx,0),max(dy,0)));
end
out.boundaryViolation=ego.y-egoHalfY < -1.75 || ego.y+egoHalfY > 8.75;
% Advance other actors and ego over the same interval. The generic actor
% integrator's ego result is discarded; only the bicycle model moves ego.
nextActors=update_highway_actors(state.actors,opts.dt);
[nextActors(1),state.egoYaw,state.ego]=update_ego_bicycle( ...
    rmfield(ego,'curvature'),state.egoYaw,ax,control.steeringAngle,opts);
state.actors=nextActors;
state.steeringAngle=state.ego.steering;
out.nextEgo=state.ego;
end
