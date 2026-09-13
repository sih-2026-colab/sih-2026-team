function d=autonex_judge_explain(out)
% Read-only adapter. Planner and guardian results are captured, never rerun.
d=struct('available',false,'rows',{cell(0,5)},'safe',false(1,numel(out.candidates)), ...
    'selectedIndex',[],'predictions',struct([]),'risk','Telemetry unavailable', ...
    'detail','Score N/A','sensors',{{'Sensor telemetry unavailable'}}, ...
    'transitions',struct,'chain','Awaiting pipeline telemetry');
if ~isfield(out,'explainability'), return; end
d.available=true; e=out.explainability; r=e.plannerResults; g=e.guardianResults;
reasons=cell(1,numel(r)); status=cell(1,numel(r));
for k=1:numel(r)
    parts={};
    if r(k).frontConflicts>0, parts{end+1}='front envelope'; end
    if r(k).rearConflicts>0, parts{end+1}='rear envelope'; end
    if r(k).sideConflicts>0, parts{end+1}='side envelope'; end
    if r(k).boundaryViolation, parts{end+1}='boundary'; end
    if ~r(k).comfortSafe, parts{end+1}='lateral comfort'; end
    if ~g(k).safe, parts{end+1}='guardian veto'; end
    d.safe(k)=r(k).safe && g(k).safe;
    status{k}='SAFE'; if ~d.safe(k), status{k}='REJECTED'; end
    reasons{k}=strjoin(parts,', ');
    if isempty(parts), reasons{k}='all evaluated gates pass'; end
    if ~isempty(out.selected) && isequaln(out.selected.trajectory,out.candidates(k).trajectory)
        d.selectedIndex=k; status{k}='SELECTED';
    end
end
if ~isempty(r)
    [~,order]=sort([r.score]); order=order(1:min(6,numel(order)));
    if ~isempty(d.selectedIndex) && ~ismember(d.selectedIndex,order), order(end)=d.selectedIndex; end
    for k=1:numel(order)
        i=order(k); d.rows(k,:)={sprintf('%s / %.0f',r(i).name,r(i).targetSpeedKmh), ...
            status{i},r(i).score,sprintf('F%d R%d S%d',r(i).frontConflicts,r(i).rearConflicts,r(i).sideConflicts),reasons{i}};
    end
end
d.detail=sprintf('%s | %.1f km/h | Y %.2f m\nScore N/A: no selected planner candidate', ...
    out.selectedName,out.targetSpeed,out.targetY);
if ~isempty(d.selectedIndex)
    q=r(d.selectedIndex); gg=g(d.selectedIndex);
    d.detail=sprintf(['%s | %.1f km/h | Y %.2f m\nPlanner total %.2f | proximity %.2f | intent %.2f\n' ...
        'Guardian risk %.2f | front TTC %s | rear TTC %s'], ...
        q.name,out.targetSpeed,out.targetY,q.score,q.proximityCost,q.intentCost,gg.risk,number(gg.minFrontTTC),number(gg.minRearTTC));
elseif ~isempty(out.selected) && strcmp(out.guardianMode,'REAR_MITIGATION_FRONT_VERIFIED')
    d.detail=sprintf('%s | %.1f km/h | Y %.2f m\nRear total %.2f | rear %.2f | comfort %.2f | progress %.2f', ...
        out.selectedName,out.targetSpeed,out.targetY,out.totalScore,out.rearRiskCost,out.comfortCost,out.progressCost);
end
% Same CV mean used by evaluate_2d_trajectories. Horizons are on its .1 s grid.
hs=[0 .5 1 1.5 2]; angles=linspace(0,2*pi,33); circle=[cos(angles);sin(angles)];
nearest=inf; threat='No confirmed track'; ttc=NaN; separation=NaN; closing=NaN; risk='NO TRACK';
for k=1:numel(out.tracks)
    tr=out.tracks(k); s=tr.State; confidence=.6; uncertainty=1.5;
    if ~isempty(e.fused)
        idx=find([e.fused.trackID]==tr.TrackID,1);
        if ~isempty(idx), confidence=e.fused(idx).fusedConfidence; uncertainty=e.fused(idx).fusedUncertainty; end
    end
    xy=[s(1)+s(2)*hs;s(3)+s(4)*hs]; ellipses=cell(1,4);
    for j=2:5
        h=hs(j); F=zeros(2,numel(s)); F(1,1:2)=[1 h]; F(2,3:4)=[1 h]; P=F*tr.StateCovariance*F';
        [V,D]=eig((P+P')/2); ellipses{j-1}=V*diag(sqrt(max(0,diag(D))))*circle+xy(:,j);
    end
    dx=s(1)-e.ego.x; dy=s(3)-e.ego.y;
    c=max(e.ego.vx-s(2),0); if dx<0, c=max(s(2)-e.ego.vx,0); end
    lat=s(4)-e.ego.vy; toward=abs(dy)>.05 && -sign(dy)*lat>.1;
    env=calculate_contextual_safety_envelope(uncertainty,confidence,c,lat,toward);
    bubble=diag([env.longitudinal env.lateral])*circle+xy(:,1);
    normSep=(dx/env.longitudinal)^2+(dy/env.lateral)^2;
    p=struct('id',tr.TrackID,'xy',xy,'ellipses',{ellipses},'bubble',bubble, ...
        'envelope',env,'conflict',normSep<=1);
    if isempty(d.predictions), d.predictions=p; else, d.predictions(end+1)=p; end %#ok<AGROW>
    % Relevant track = smallest current normalized envelope separation.
    if normSep<nearest
        nearest=normSep; separation=hypot(dx,dy); closing=c; ttc=NaN;
        % Canonical guardian longitudinal TTC eligibility at h=0.
        if abs(dy)<=2.2 && c>.10 && dx~=0, ttc=abs(dx)/c; end
        threat=sprintf('Track %d',tr.TrackID);
        if ~isempty(out.worldModel)
            i=find([out.worldModel.id]==tr.TrackID,1);
            if ~isempty(i), threat=sprintf('%s / %s',threat,out.worldModel(i).class); end
        end
        risk='OUTSIDE ENVELOPE'; if normSep<=1, risk='ENVELOPE CONFLICT'; end
    end
end
d.risk=sprintf('%s\nCentre distance %s m | closing %s m/s | TTC %s s\n%s | footprint clearance %s m\nCollision %d | boundary %d (simulation truth)', ...
    threat,number(separation),number(closing),number(ttc),risk,number(out.minClearance),out.collision,out.boundaryViolation);
d.transitions.risk=risk; d.transitions.action=out.motionMode; d.transitions.guardian=out.guardianMode;
d.transitions.rejections=sprintf('%d rejected / %d evaluated',sum(~d.safe),numel(r));
d.transitions.tracks=sprintf('%d confirmed tracks',numel(out.tracks));
d.sensors={'Sensor frame unavailable';'THERMAL: inactive in camera/radar/LiDAR mode'};
if isfield(out,'sensorFrame')
    f=out.sensorFrame; names={'camera','radar','lidar'}; d.sensors={};
    for k=1:3
        values=f.(names{k}); label='NO DETECTIONS';
        if ~e.sensorAvailability(k), label='DISABLED'; elseif ~isempty(values), label=sprintf('ACTIVE / %d detections',numel(values)); end
        d.sensors{end+1}=sprintf('%s: %s',upper(names{k}),label);
    end
    d.sensors{end+1}='THERMAL: inactive (not fused in this mode)';
    % Class-level evidence, not an invented actor/track association.
    pedDetected=~isempty(f.camera) && any(strcmpi({f.camera.type},'pedestrian'));
    pedFused=~isempty(out.worldModel) && any(strcmpi({out.worldModel.class},'pedestrian'));
    d.transitions.pedestrianDetection=sprintf('Pedestrian camera detection %d',pedDetected);
    d.transitions.pedestrianTrack=sprintf('Pedestrian fused class %d',pedFused);
    d.sensors{end+1}=sprintf('PEDESTRIAN: camera detection %d / fused class %d',pedDetected,pedFused);
    ped=find(strcmpi({out.actors.type},'pedestrian'),1);
    if ~isempty(ped)
        config=autonex_sensor_config(e.sensorConfig); yaw=e.ego.heading;
        R=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
        origin=[e.ego.x e.ego.y]+config.cameraPose(1:2)*R';
        v=autonex_actor_visibility(origin,out.actors,ped,'camera');
        label='VISIBLE'; if v.fullyOccluded, label='OCCLUDED'; elseif v.visibleFraction<1, label='PARTIAL'; end
        d.sensors{end+1}=sprintf('Truth LOS / %s: %s (%.0f%%)',out.actors(ped).name,label,100*v.visibleFraction);
        origin=[e.ego.x e.ego.y]+config.lidarPose(1:2)*R';
        v=autonex_actor_visibility(origin,out.actors,ped,'depth');
        d.sensors{end+1}=sprintf('Depth geometric LOS blocked: %d (not a return)',v.fullyOccluded);
        v=autonex_actor_visibility([e.ego.x e.ego.y],out.actors,ped,'thermal');
        d.sensors{end+1}=sprintf('Thermal geometric LOS blocked: %d (sensor inactive)',v.fullyOccluded);
        d.transitions.visibility=['Pedestrian camera LOS ' label];
    end
end
d.chain=sprintf('PERCEPTION %d RGB  >  TRACK %d  >  PREDICT %d  >  RISK %d rejected  >  PLAN %s  >  GUARDIAN %s  >  %s', ...
    out.rgbDetections,out.trackCount,numel(d.predictions),sum(~d.safe),out.selectedName,out.guardianMode,out.motionMode);
end
function s=number(x)
if isnan(x), s='N/A'; elseif isinf(x), s='Inf'; else, s=sprintf('%.2f',x); end
end
