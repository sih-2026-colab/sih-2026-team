function [state,out] = autonex_step(state,t,opts)
% One shared closed-loop step for MATLAB, animation, validation and Simulink.
if isempty(state)
    rng(opts.seed); state.actors=create_highway_scenario();
    if strcmp(opts.scenario,'stationary_obstacle')
        state.actors(3).x=55; state.actors(3).y=7; state.actors(3).vx=0; state.actors(3).vy=0;
        state.actors(2).x=-80;
    elseif strcmp(opts.scenario,'clear_road')
        for k=2:numel(state.actors), state.actors(k).x=200+k*30; state.actors(k).vy=0; end
    end
    state.tracker=create_autonex_gnn_tracker(); state.selection=[];
    state.lastDetectionTime=0; state.mapTime=-inf; state.mapMemory=[];
    state.trackerInitialized=false;
end
ego=state.actors(1);
environment=struct('lightLevel',1,'visibility',1,'radarQuality',.95,'thermalQuality',.85);
if strcmp(opts.scenario,'degraded_sensing')
    environment.lightLevel=.1; environment.visibility=.2;
    environment.radarQuality=.45; environment.thermalQuality=.5;
end
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
        referenceY=interp1(corridor.x,corridor.referenceY,min(corridor.x(end),max(corridor.x(1),ego.x+10)));
    end
    candidates=generate_corridor_candidates(ego,0:.1:3,corridor,state.drivable,state.safeSpace);
else
    corridor=struct('valid',false,'length',0,'reason','LEGACY_LANE_REFERENCE');
    referenceY=7; candidates=generate_2d_candidate_set(ego,0:.1:3,referenceY);
end
if ~isempty(tracks)
    [value,found]=find_highest_cutin_risk(tracks,ego,referenceY);
    if found, intent=value; end
end
selected=[]; index=NaN; mode='NO_SAFE_SPACE'; stability='RESET'; guardian='NO_SAFE_SPACE';
results=struct([]);
if ~isempty(candidates)
    results=evaluate_2d_trajectories(candidates,tracks,fused,ego,intent,-1.75,8.75);
    [proposal,index,mode]=select_best_2d_trajectory(candidates,results);
    [selected,index,mode,state.selection,stability]=stabilize_trajectory_selection( ...
        candidates,results,proposal,index,mode,state.selection,t);
    [selected,index,guardian]=cat_reflex_2d_guardian(candidates,results,selected,index,tracks,-1.75,8.75);
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
    command=cat_reflex_guardian(ego.vx*3.6,targetSpeed);
end
ax=speed_tracking_controller(ego.vx*3.6,targetSpeed,command);
ay=lateral_tracking_controller(ego.y,ego.vy,targetY);
if ego.vx<=0 && ax<0, ax=0; end
state.actors(1).ax=ax; state.actors(1).ay=ay;
out=struct('time',t,'speed',ego.vx*3.6,'targetSpeed',targetSpeed,'targetY',targetY, ...
    'selectionMode',mode,'selectedName',name,'stabilityMode',stability,'guardianMode',guardian, ...
    'pCut',0,'x',ego.x,'y',ego.y,'ax',ax,'ay',ay,'trackCount',numel(tracks), ...
    'corridorLength',corridor.length,'candidateCount',numel(candidates),'emergency',emergency);
if ~isempty(intent), out.pCut=intent.probability*100; end
out.actors=state.actors; out.candidates=candidates; out.selected=selected; out.tracks=tracks;
out.collision=false; out.minClearance=inf;
for k=2:numel(state.actors)
    other=state.actors(k);
    dx=abs(other.x-ego.x)-4.5; dy=abs(other.y-ego.y)-1.9;
    out.collision=out.collision || (dx<0 && dy<0);
    out.minClearance=min(out.minClearance,hypot(max(dx,0),max(dy,0)));
end
out.boundaryViolation=ego.y-.95 < -1.75 || ego.y+.95 > 8.75;
% Clamp velocity before integration; do not hide road departures by clamping position.
state.actors(1).ax=max(ax,-ego.vx/opts.dt);
state.actors=update_highway_actors(state.actors,opts.dt);
end
