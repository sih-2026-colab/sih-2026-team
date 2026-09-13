function rows=validate_rear_impact_mitigation(names)
allowed={'stationary_obstacle','close_rear_cut_in','original_close_range'};
if nargin==0, names=allowed; end
assert(iscellstr(names) && all(ismember(names,allowed)),'Unknown rear mitigation scenario');
rows=struct([]);
for k=1:numel(names)
    opts=autonex_options(struct('scenario',names{k},'duration',10,'validationTelemetry',true));
    state=[]; samples=struct([]); frontMinimum=inf; rearMinimum=inf;
    collisionTime=[]; partners=[];
    for t=0:opts.dt:opts.duration
        [state,out]=autonex_step(state,t,opts);
        ego=out.actors(1);
        for j=2:numel(out.actors)
            other=out.actors(j); [L,W]=autonex_actor_size(other);
            halfX=2.25*abs(cos(out.egoYaw))+.95*abs(sin(out.egoYaw));
            halfY=2.25*abs(sin(out.egoYaw))+.95*abs(cos(out.egoYaw));
            dx=other.x-ego.x; dy=other.y-ego.y;
            clearance=hypot(max(abs(dx)-halfX-L/2,0),max(abs(dy)-halfY-W/2,0));
            if dx>=0, frontMinimum=min(frontMinimum,clearance);
            else, rearMinimum=min(rearMinimum,clearance); end
        end
        if out.collision
            if isempty(collisionTime), collisionTime=t; end
            partners=unique([partners out.collisionActorIDs]);
        end
        s=rmfield(out,{'actors','candidates','selected','tracks'});
        if isempty(samples), samples=s; else, samples(end+1)=s; end %#ok<AGROW>
    end
    modes={samples.minimumRiskMode}; counts=struct;
    for m=unique(modes), counts.(m{1})=sum(strcmp(modes,m{1})); end
    active=[samples.rearRiskActive]; emergency=[samples.emergency];
    finite=all(isfinite([samples.x samples.y samples.speed samples.egoYaw samples.steeringAngle samples.steeringRate]));
    row=struct('scenarioName',names{k},'collisionCount',sum([samples.collision]), ...
        'collisionTime',collisionTime,'collisionPartner',partners, ...
        'minimumFrontClearance',frontMinimum,'minimumRearClearance',rearMinimum, ...
        'minimumRearTTC',min([samples.rearTTC]), ...
        'rearRiskActivations',sum(diff([false active])==1), ...
        'rearThreatActorId',unique([samples(active).rearThreatActorId]),'rearThreatIdSource','TRACK_ID', ...
        'minimumRiskModeCounts',counts,'forwardCreepActivations',sum(diff([false strcmp(modes,'FORWARD_CREEP')])==1), ...
        'lateralEscapeActivations',sum(diff([false strcmp(modes,'LATERAL_ESCAPE')])==1), ...
        'CatReflexActivations',sum(diff([false ~strcmp({samples.guardianMode},'APPROVED')])==1), ...
        'emergencySteps',sum(emergency),'maxSteering',rad2deg(max(abs([samples.steeringAngle]))), ...
        'maxSteeringRate',rad2deg(max(abs([samples.steeringRate]))), ...
        'distance',samples(end).x-samples(1).x,'finalSpeed',samples(end).speed, ...
        'averageSpeed',(samples(end).x-samples(1).x)/opts.duration, ...
        'boundaryViolations',sum([samples.boundaryViolation]),'NaNInfDetected',~finite, ...
        'rearImpactUnavoidable',any([samples.rearImpactUnavoidable]),'PASS_FAIL',false);
    row.PASS_FAIL=row.collisionCount==0 && row.boundaryViolations==0 && finite && ...
        row.maxSteeringRate<=rad2deg(opts.maxSteeringRate)+1e-8 && ...
        row.maxSteering<=rad2deg(opts.maxSteeringAngle)+1e-8;
    if isempty(rows), rows=row; else, rows(end+1)=row; end %#ok<AGROW>
    fid=fopen(fullfile('results',['rear_mitigation_' names{k} '.json']),'w');
    fprintf(fid,'%s',jsonencode(struct('options',opts,'metrics',row,'samples',samples),PrettyPrint=true)); fclose(fid);
    fid=fopen('results/rear_mitigation_summary.json','w');
    fprintf(fid,'%s',jsonencode(rows,PrettyPrint=true)); fclose(fid);
    disp(row);
end
end
