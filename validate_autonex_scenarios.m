function rows=validate_autonex_scenarios(scenarioNames)
% Unified full-pipeline regression. Metrics use SI units except labelled degrees/kmh.
% The legacy missing_lanes label is tested explicitly; it falls through to cut_in.
names={'missing_lane','cut_in','night_pedestrian','animal','degraded_sensing', ...
    'pothole','tracking_loss','stationary_obstacle','clear_road', ...
    'close_rear_cut_in','original_close_range','missing_lanes'};
rows=struct([]);
if nargin>0
    assert(all(ismember(scenarioNames,names)),'Unknown scenario identifier');
    names=scenarioNames;
    if isfile('results/multi_scenario_regression.json')
        rows=jsondecode(fileread('results/multi_scenario_regression.json'))';
    end
end
for k=1:numel(names)
    name=names{k}; duration=10;
    if strcmp(name,'missing_lane'), duration=25; end
    opts=struct('scenario',name,'duration',duration,'validationTelemetry',true, ...
        'showLaneMarkings',~ismember(name,{'missing_lane','missing_lanes'}), ...
        'reportFile',fullfile('results',['scenario_' name '.json']));
    r=run_autonex_simulation(opts); s=r.samples; dt=r.options.dt;
    v=[s.speed]/3.6; steering=[s.steeringAngle]; ax=[s.ax]; ay=[s.ay];
    limited=abs(steering)>=r.options.maxSteeringAngle-1e-8;
    intervals=[diff([s.time]) 0];
    turns=sign(steering(abs(steering)>deg2rad(2)));
    reversals=sum(diff(turns)~=0);
    yaw=[s.egoYaw]; x=[s.x]; y=[s.y];
    expectedV=max(0,v(1:end-1)+ax(1:end-1)*dt);
    maxSpeed=75/3.6; % Existing generator's highest candidate, not a new control limit.
    physical=max(v)<=max(maxSpeed,v(1))+1e-6 && min(v)>=0 && ...
        max(ax)<=r.options.maxAcceleration+1e-8 && min(ax)>=-r.options.maxDeceleration-1e-8 && ...
        max(abs(v(2:end)-expectedV))<1e-6;
    meanV=(v(1:end-1)+v(2:end))/2;
    movingDt=dt*ones(size(meanV));
    braking=ax(1:end-1)<0;
    movingDt(braking)=min(dt,v(find(braking))./(-ax(find(braking))));
    midYaw=yaw(1:end-1)+meanV/r.options.wheelbase.*tan(steering(1:end-1)).*movingDt/2;
    positionError=max(hypot(diff(x)-meanV.*cos(midYaw).*movingDt,diff(y)-meanV.*sin(midYaw).*movingDt));
    physical=physical && positionError<1e-6;
    emergency=[s.emergency]; authority=all([s(emergency).targetSpeed]==0) && all(ax(emergency)<=0);
    finite=all(isfinite([x y v ax ay steering yaw s.steeringRate s.lateralError]));
    active=emergency | ~strcmp({s.guardianMode},'APPROVED') | ...
        ismember({s.longitudinalCommand},{'CONTROLLED_BRAKE','MICRO_DECELERATE','EMERGENCY_BRAKE'});
    activationCount=sum(diff([false active])==1);
    audit=[s.perceptionAudit]; tracks=sum([audit.trackCount]);
    qualityDelta=sum([audit.qualityUncertaintyDelta])/max(tracks,1);
    marginDelta=sum([audit.qualityMarginDelta])/max(tracks,1);
    feature=true; featureCause='';
    if strcmp(name,'missing_lane')
        feature=x(end)>52.5 && range(y)>1 && r.options.laneIndependent;
        featureCause='Both staggered blockers must be passed';
    elseif ismember(name,{'cut_in','close_rear_cut_in','original_close_range','missing_lanes'})
        feature=max([s.pCut])>0 && any(ax<-.5); featureCause='Cut-in detection and braking required';
    elseif ismember(name,{'night_pedestrian','animal'})
        feature=any(ax<-.5) && sum([s.thermalDetections])>0 && max([audit.lateralMax])>2.2;
        featureCause='Thermal detection, expanded envelope and braking required';
    elseif strcmp(name,'degraded_sensing')
        feature=qualityDelta>1e-6 && marginDelta>=-1e-8;
        featureCause='Matched-observation degraded quality must increase uncertainty without shrinking margins';
    elseif strcmp(name,'pothole')
        feature=max([s.surfaceHazardCells])>0 && any(ax<-.5); featureCause='Surface hazard must trigger response';
    elseif strcmp(name,'tracking_loss')
        window=[s.time]>=1.5 & [s.time]<=3;
        feature=all(emergency(window)); featureCause='Missing stream must trigger emergency';
    end
    progress=r.distance>=min(10,.2*v(1)*duration) || (any(emergency) && v(end)<.1);
    rmsError=sqrt(mean([s.lateralError].^2));
    unstable=reversals>max(10,duration) || rmsError>.1;
    excessiveLateral=max(abs(ay))>3.5+1e-6;
    causes={};
    if r.collision
        first=find([s.collision],1);
        causes{end+1}=sprintf('Collision at %.2f s with actor IDs %s',s(first).time,mat2str(s(first).collisionActorIDs));
        if ismember(2,s(first).collisionActorIDs)
            causes{end+1}='Constant-speed rear actor reaches braking/stopped ego; rear-impact avoidance unresolved';
        end
    end
    if r.boundaryViolation, causes{end+1}='Road departure'; end
    if ~finite, causes{end+1}='Nonfinite vehicle state'; end
    if ~physical, causes{end+1}='Speed/acceleration/bicycle integration inconsistent'; end
    if ~authority, causes{end+1}='Emergency authority violated'; end
    if ~progress, causes{end+1}='Insufficient forward progress'; end
    if unstable, causes{end+1}='Sustained oscillation or tracking error'; end
    if excessiveLateral, causes{end+1}='Lateral acceleration exceeds existing guardian limit'; end
    if ~feature, causes{end+1}=featureCause; end
    saturationContext='NONE';
    if any(limited)
        if all(active(limited) | abs([s(limited).targetY]-[s(limited).y])>.1)
            saturationContext='AVOIDANCE_OR_REPOSITIONING';
        else, saturationContext='INCLUDES_CRUISE_REQUIRES_REVIEW'; end
    end
    row=struct('scenarioName',name,'duration',duration,'distance',r.distance, ...
        'averageSpeed',r.distance/duration,'finalSpeedKmh',r.finalSpeed,'maximumSpeedKmh',max(v)*3.6, ...
        'minimumClearance',r.minClearance,'collisionCount',sum([s.collision]), ...
        'boundaryViolations',sum([s.boundaryViolation]), ...
        'replanCount',sum(~strcmp({s(1:end-1).selectedName},{s(2:end).selectedName})), ...
        'CatReflexActivations',activationCount,'emergencySteps',r.emergencySteps, ...
        'maxSteering',rad2deg(max(abs(steering))),'samplesAtSteeringLimit',sum(limited), ...
        'timeAtSteeringLimit',sum(intervals(limited)), ...
        'steeringLimitPercentage',100*sum(intervals(limited))/duration, ...
        'saturationContext',saturationContext,'maxSteeringRate',rad2deg(max(abs([s.steeringRate]))), ...
        'significantSteeringReversals',reversals,'maxLateralError',max(abs([s.lateralError])), ...
        'RMSLateralError',rmsError,'NaNInfDetected',~finite,'physicalConsistency',physical, ...
        'positionIntegrationError',positionError,'maxLateralAcceleration',max(abs(ay)), ...
        'qualityUncertaintyDelta',qualityDelta,'qualityMarginDelta',marginDelta, ...
        'PASS_FAIL',isempty(causes),'failureCause',strjoin(causes,'; '));
    if isempty(rows), rows=row;
    else
        previous=find(strcmp({rows.scenarioName},name),1);
        if isempty(previous), rows(end+1)=row; else, rows(previous)=row; end %#ok<AGROW>
    end
    fid=fopen('results/multi_scenario_regression.json','w');
    fprintf(fid,'%s',jsonencode(rows,PrettyPrint=true)); fclose(fid);
    fprintf('REGRESSION %s PASS=%d | %s\n',name,row.PASS_FAIL,row.failureCause);
end
t=struct2table(rows);
disp(t(:,{'scenarioName','PASS_FAIL','distance','averageSpeed','minimumClearance', ...
    'collisionCount','boundaryViolations','replanCount','CatReflexActivations', ...
    'emergencySteps','maxSteering','steeringLimitPercentage','maxLateralError','RMSLateralError'}));
end
