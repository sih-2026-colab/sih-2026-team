function [choice,diagnostic]=evaluate_minimum_risk_action(ego,yaw,steering,tracks,fused, ...
    corridor,drivable,safeSpace,selected,opts,commitment)
% Front/occupancy/actuator feasibility first; rear harm is ranked only afterward.
if nargin<11, commitment=[]; end
choice=[]; diagnostic=struct('tested',0,'feasible',0,'bestRearCost',inf, ...
    'baselineRearConflict',NaN,'candidateRearConflict',NaN,'rearConflictReduction',NaN, ...
    'rearRiskCost',NaN,'comfortCost',NaN,'progressCost',NaN,'totalScore',NaN, ...
    'candidateScores',struct([]));
% Match the shared planner's observed-space action horizon. The longer rear
% stopping forecast is an alert, not permission to traverse unseen map cells.
h=0:.1:opts.rearActionHorizon;
pool=generate_corridor_candidates(ego,h,corridor,drivable,safeSpace,true);
if isempty(pool), return; end
% Reuse the candidate schema and quintic generator for explicit forward creep.
targets=unique([pool.targetY]);
for y=targets
    c=pool(1); c.targetY=y; c.targetSpeedKmh=opts.rearCreepSpeedKmh;
    c.name=sprintf('REAR_CREEP_%.2f',y);
    c.maneuverTime=max(2.5,sqrt(6*abs(y-ego.y)/2.5));
    c.trajectory=generate_2d_trajectory(ego,c.targetSpeedKmh,y,h,c.maneuverTime);
    if trajectory_in_safe_space(c.trajectory,drivable,safeSpace,ego), pool(end+1)=c; end %#ok<AGROW>
end
% Preserve the deadline of an ongoing escape instead of postponing its
% completion by a full maneuver duration at every 20 Hz replan. Regenerated
% paths still start at measured velocity/curvature and pass every gate below.
if ~isempty(commitment) && commitment.remainingTime>0
    keep=true(size(pool));
    for k=1:numel(pool)
        c=pool(k);
        if abs(c.targetY-commitment.targetY)<=.75
            c.maneuverTime=min(c.maneuverTime,max(.5,commitment.remainingTime));
            c.trajectory=generate_2d_trajectory(ego,c.targetSpeedKmh,c.targetY,h,c.maneuverTime);
            keep(k)=trajectory_in_safe_space(c.trajectory,drivable,safeSpace,ego);
            pool(k)=c;
        end
    end
    pool=pool(keep);
end
if isempty(pool), return; end
frontMask=arrayfun(@(q)q.State(1)>=ego.x-2.25,tracks);
front=tracks(frontMask); frontFused=fused(frontMask);
checks=evaluate_2d_trajectories(pool,front,frontFused,ego,[],-1.75,8.75);
[~,~,~,guardChecks]=cat_reflex_2d_guardian(pool,checks,[],NaN,front,-1.75,8.75);
% The existing guardian's front, side, comfort and geometric gates are retained.
baseline=selected;
if isempty(baseline)
    baseline=struct('targetSpeedKmh',0,'trajectory',[],'emergencyBaseline',true);
end
% Roll the action the loop would actually execute without mitigation. An
% unsafe baseline is a comparator only, never an admissible alternative.
[~,baselineConflict,~]=roll_candidate(baseline,ego,yaw,steering,tracks,fused,frontMask,drivable,safeSpace,opts,false);
diagnostic.baselineRearConflict=baselineConflict;
bestScore=inf;
for k=1:numel(pool)
    if ~checks(k).safe || ~guardChecks(k).safe, continue; end
    diagnostic.tested=diagnostic.tested+1;
    c=pool(k);
    [valid,rearCost,rollout]=roll_candidate(c,ego,yaw,steering,tracks,fused,frontMask,drivable,safeSpace,opts);
    if ~valid, continue; end
    diagnostic.feasible=diagnostic.feasible+1;
    costs=score_rear_conflict_reduction(baselineConflict,rearCost, ...
        .1*abs(c.targetY-ego.y),-.001*(rollout.x(end)-ego.x));
    audit=costs; audit.targetY=c.targetY; audit.targetSpeedKmh=c.targetSpeedKmh;
    if isempty(diagnostic.candidateScores), diagnostic.candidateScores=audit;
    else, diagnostic.candidateScores(end+1)=audit; end
    score=costs.totalScore;
    if score<bestScore
        bestScore=score; c.rearCost=rearCost; c.rollout=rollout;
        c.minimumRiskMode='REAR_AWARE_BRAKE';
        if abs(c.targetY-ego.y)>.75, c.minimumRiskMode='LATERAL_ESCAPE';
        elseif c.targetSpeedKmh<=opts.rearCreepSpeedKmh && c.targetSpeedKmh>0
            c.minimumRiskMode='FORWARD_CREEP';
        end
        choice=c; diagnostic.bestRearCost=rearCost;
        fields={'candidateRearConflict','rearConflictReduction','rearRiskCost','comfortCost','progressCost','totalScore'};
        for z=1:numel(fields), diagnostic.(fields{z})=costs.(fields{z}); end
    end
end
% Keep a presently safe plan if its physically rolled rear outcome is as good.
if ~isempty(selected) && ~isempty(choice)
    [valid,currentCost,currentRollout]=roll_candidate(selected,ego,yaw,steering,tracks,fused,frontMask,drivable,safeSpace,opts);
    currentScore=score_rear_conflict_reduction(baselineConflict,currentCost,0, ...
        -.001*(currentRollout.x(end)-ego.x));
    if valid && currentScore.totalScore<=bestScore, choice=[]; end
end
end
function [valid,rearCost,tr]=roll_candidate(c,ego,yaw,steering,tracks,fused,frontMask,drivable,safeSpace,opts,checkSafety)
if nargin<11, checkSafety=true; end
start=ego; valid=true; rearCost=0;
steps=0:opts.dt:opts.rearActionHorizon;
tr=struct('time',steps,'x',zeros(size(steps)),'y',zeros(size(steps)), ...
    'vx',zeros(size(steps)),'vy',zeros(size(steps)),'ay',zeros(size(steps)));
for j=1:numel(steps)
    t=steps(j); speed=hypot(ego.vx,ego.vy);
    command=cat_reflex_guardian(speed*3.6,c.targetSpeedKmh);
    if isfield(c,'emergencyBaseline') && c.emergencyBaseline, command='EMERGENCY_BRAKE'; end
    ax=speed_tracking_controller(speed*3.6,c.targetSpeedKmh,command,opts);
    ctrl=path_following_controller(ego,yaw,steering,c.trajectory,opts,[],true);
    halfX=2.25*abs(cos(yaw))+.95*abs(sin(yaw));
    halfY=2.25*abs(sin(yaw))+.95*abs(cos(yaw));
    if ego.y-halfY < -1.75 || ego.y+halfY>8.75 || ...
            abs(ctrl.steeringRate)>opts.maxSteeringRate+1e-8 || ...
            abs(ctrl.steeringAngle)>opts.maxSteeringAngle+1e-8 || ...
            abs(speed^2/opts.wheelbase*tan(ctrl.steeringAngle))>3.5
        valid=false; if checkSafety, return; end
    end
    tr.x(j)=ego.x; tr.y(j)=ego.y; tr.vx(j)=ego.vx; tr.vy(j)=ego.vy;
    tr.ay(j)=speed^2/opts.wheelbase*tan(ctrl.steeringAngle);
    for q=1:numel(tracks)
        s=tracks(q).State; dx=s(1)+s(2)*t-ego.x; dy=s(3)+s(4)*t-ego.y;
        f=fused(q); closing=max(ego.vx-s(2),0);
        envelope=calculate_contextual_safety_envelope(f.fusedUncertainty+.25*t, ...
            f.fusedConfidence,closing,s(4)-ego.vy,dy*(s(4)-ego.vy)<0);
        if frontMask(q)
            hard=abs(dx)<=max(5.5,halfX+2.25) && abs(dy)<=max(2,halfY+.95);
            bubble=(dx/envelope.longitudinal)^2+(dy/envelope.lateral)^2<1;
            stop=dx>0 && abs(dy)<2 && closing>0 && dx<closing^2/12+6;
            ttc=dx>0 && abs(dy)<=2.2 && closing>.1 && dx/closing<2;
            if hard || bubble || stop || ttc
                valid=false; if checkSafety, return; end
            end
        else
            % A turning ego occupies more lateral space than an aligned car.
            % Include track uncertainty; do not score a footprint overlap as
            % a successful escape merely because centerlines are separated.
            sigmaY=sqrt(max(tracks(q).StateCovariance(3,3),0));
            lateralRadius=max(envelope.lateral,halfY+.95)+min(sigmaY,1);
            % Reward useful partial repositioning in currently verified space.
            % A binary overlap flag made a narrow but helpful escape corridor
            % score identically to remaining centered in the rear actor's path.
            lateralOverlap=max(0,lateralRadius-abs(dy))/lateralRadius;
            gap=-dx-(halfX+2.25);
            rearCost=rearCost+lateralOverlap^2*max(0,6-gap)^2*opts.dt;
        end
    end
    [ego,yaw]=update_ego_bicycle(ego,yaw,ax,ctrl.steeringAngle,opts);
    steering=ctrl.steeringAngle;
end
valid=valid && trajectory_in_safe_space(tr,drivable,safeSpace,start);
end
