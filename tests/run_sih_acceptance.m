function report=run_sih_acceptance(name)
% Criteria declared before execution; truth association is diagnostics ONLY.
opts=autonex_options(struct('scenario',name,'duration',10,'perceptionMode','camera_radar_lidar', ...
    'validationTelemetry',true,'sensorTelemetry',true,'showLaneMarkings',false));
state=[]; samples=struct([]); initialChecks=true; plannerValid=true; finite=true;
duplicates=false; roadSafe=true; multiTracked=false; riskSeen=false;
times=struct('firstGeometricVisibility',NaN,'firstSensorDetection',NaN, ...
    'firstFusedTrack',NaN,'firstPrediction',NaN,'firstSceneConflict',NaN, ...
    'firstRiskResponse',NaN,'firstBrakeOrReplan',NaN,'firstPostDetectionBrakeOrReplan',NaN);
for t=0:opts.dt:opts.duration
    [state,out]=autonex_step(state,t,opts); actors=out.actors; ego=actors(1);
    g=autonex_road_geometry(actors); frame=out.sensorFrame; world=out.worldModel;
    ped=find(strcmpi({actors.type},'pedestrian'),1);
    v=autonex_actor_visibility([ego.x ego.y],actors,ped,'camera');
    rv=autonex_actor_visibility([ego.x ego.y],actors,ped,'radar');
    savedRng=rng; thermal=simulate_thermal_ir(ego,actors); rng(savedRng);
    [p,pose]=simulate_autonex_depth_scan(actors); xy=p(:,1:2)+pose(1:2);
    depthSeen=any(abs(xy(:,1)-actors(ped).x)<=.31 & abs(xy(:,2)-actors(ped).y)<=.31);
    cameraSeen=~isempty(frame.camera) && any(strcmpi({frame.camera.type},'pedestrian'));
    thermalSeen=~isempty(thermal) && any(strcmpi({thermal.type},'pedestrian'));
    radarSeen=false;
    if ~isempty(frame.radar)
        radarSeen=any(hypot([frame.radar.x]-actors(ped).x,[frame.radar.y]-actors(ped).y)<1.5);
    end
    sensorSeen=cameraSeen || radarSeen || depthSeen;
    matched=[];
    if ~isempty(world)
        matched=find(hypot([world.x]-actors(ped).x,[world.y]-actors(ped).y)<2 & ...
            strcmp({world.class},'pedestrian'));
        finite=finite && all(isfinite([world.x world.y world.vx world.vy world.uncertainty]));
    end
    duplicates=duplicates || numel(matched)>1;
    assignedPedestrian=0;
    for q=1:numel(world)
        distances=hypot([actors(2:end).x]-world(q).x,[actors(2:end).y]-world(q).y);
        [distance,nearest]=min(distances);
        assignedPedestrian=assignedPedestrian+double(nearest+1==ped && distance<2);
    end
    duplicates=duplicates || assignedPedestrian>1;
    multiTracked=multiTracked || numel(out.tracks)>=2;
    risk=0;
    for q=1:numel(out.tracks)
        s=out.tracks(q).State; h=0:.1:5;
        dx=s(1)+s(2)*h-(ego.x+ego.vx*h); dy=s(3)+s(4)*h-(ego.y+ego.vy*h);
        risk=max(risk,max(0,1-min((dx/5.5).^2+(dy/2.2).^2)));
        finite=finite && all(isfinite([s(:);out.tracks(q).StateCovariance(:)]));
    end
    riskSeen=riskSeen || risk>0;
    response=out.ax<-.1 || strcmp(out.motionMode,'REPLAN') || out.emergency;
    if v.visible && isnan(times.firstGeometricVisibility), times.firstGeometricVisibility=t; end
    if sensorSeen && isnan(times.firstSensorDetection), times.firstSensorDetection=t; end
    if ~isempty(matched) && isnan(times.firstFusedTrack), times.firstFusedTrack=t; times.firstPrediction=t; end
    if risk>0 && isnan(times.firstSceneConflict), times.firstSceneConflict=t; end
    if risk>0 && response && isnan(times.firstRiskResponse), times.firstRiskResponse=t; end
    if response && isnan(times.firstBrakeOrReplan), times.firstBrakeOrReplan=t; end
    if response && sensorSeen && isnan(times.firstPostDetectionBrakeOrReplan), times.firstPostDetectionBrakeOrReplan=t; end
    if t==0 && strcmp(name,'occluded_pedestrian')
        parked=autonex_actor_visibility([ego.x ego.y],actors,2,'camera');
        parkedDetected=~isempty(frame.camera) && any(strcmpi({frame.camera.type},'car'));
        initialChecks=v.fullyOccluded && parked.visible && parkedDetected && ...
            ~cameraSeen && ~thermalSeen && ~depthSeen && ~radarSeen && assignedPedestrian==0;
    elseif t==0
        initialChecks=g.junction && autonex_road_contains(32,-5,g) && ...
            ~autonex_road_contains(20,-5,g) && ~opts.showLaneMarkings;
    end
    plannerValid=plannerValid && (~isempty(out.selected) || (out.emergency && out.targetSpeed==0));
    for q=1:numel(out.candidates)
        tr=out.candidates(q).trajectory;
        plannerValid=plannerValid && all(isfinite([tr.x tr.y tr.vx tr.vy tr.ay]));
        roadSafe=roadSafe && all(autonex_road_contains(tr.x,tr.y,g));
    end
    finite=finite && all(isfinite([out.x out.y out.speed out.ax out.egoYaw out.steeringAngle out.steeringRate]));
    row=struct('time',t,'x',out.x,'y',out.y,'speedKmh',out.speed,'ax',out.ax, ...
        'steering',out.steeringAngle,'steeringRate',out.steeringRate, ...
        'collision',out.collision,'collisionActorIDs',out.collisionActorIDs, ...
        'boundary',out.boundaryViolation,'clearance',out.minClearance, ...
        'visibleFraction',v.visibleFraction,'cameraOccluded',v.fullyOccluded, ...
        'thermalOccluded',v.fullyOccluded,'radarOccluded',rv.fullyOccluded, ...
        'depthPedestrianReturn',depthSeen,'cameraDetection',cameraSeen, ...
        'radarDetection',radarSeen,'thermalDetection',thermalSeen, ...
        'pedestrianTrackCount',numel(matched),'trackCount',numel(out.tracks), ...
        'risk',risk,'mode',out.motionMode,'guardian',out.guardianMode,'candidateCount',out.candidateCount);
    if isempty(samples), samples=row; else, samples(end+1)=row; end %#ok<AGROW>
end
physical=max(abs([samples.steering]))<=opts.maxSteeringAngle+1e-8 && ...
    max(abs([samples.steeringRate]))<=opts.maxSteeringRate+1e-8 && ...
    min([samples.ax])>=-6-1e-8 && max([samples.ax])<=1.2+1e-8;
timing=all(isfinite([times.firstGeometricVisibility times.firstSensorDetection times.firstFusedTrack times.firstBrakeOrReplan]));
if strcmp(name,'occluded_pedestrian')
    timing=timing && times.firstGeometricVisibility>0 && times.firstSensorDetection>0 && ...
        times.firstFusedTrack>=times.firstSensorDetection;
end
report=struct('scenario',name,'options',opts,'timings',times,'initialGeometryPass',initialChecks, ...
    'finite',finite,'plannerValid',plannerValid,'duplicateTracks',duplicates,'roadSafe',roadSafe, ...
    'multipleTracked',multiTracked,'riskSeen',riskSeen,'physical',physical, ...
    'collisionCount',sum([samples.collision]),'boundaryCount',sum([samples.boundary]), ...
    'minimumClearance',min([samples.clearance]),'samples',samples,'pass',false);
report.pass=initialChecks && timing && finite && plannerValid && ~duplicates && roadSafe && ...
    multiTracked && riskSeen && physical && report.collisionCount==0 && report.boundaryCount==0;
fid=fopen(fullfile('results',[name '_acceptance.json']),'w');
fprintf(fid,'%s',jsonencode(report,PrettyPrint=true)); fclose(fid);
fprintf('Pedestrian first geometrically visible: %.2f s\n',times.firstGeometricVisibility);
fprintf('First sensor detection: %.2f s\n',times.firstSensorDetection);
fprintf('First fused track/prediction: %.2f s\n',times.firstFusedTrack);
fprintf('First scene risk response: %.2f s\n',times.firstRiskResponse);
fprintf('First BRAKE/REPLAN: %.2f s; first while pedestrian detected: %.2f s\n', ...
    times.firstBrakeOrReplan,times.firstPostDetectionBrakeOrReplan);
fprintf('SIH_ACCEPTANCE %s PASS=%d collision=%d boundary=%d\n',name,report.pass,report.collisionCount,report.boundaryCount);
end
