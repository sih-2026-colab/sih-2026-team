function rows=write_new_sih_observations(saved)
% Uniform observation gates. No scenario-specific control or acceptance branch.
rows=struct([]);
for k=1:numel(saved)
    r=saved(k); s=r.samples;
    row=struct('scenarioName',r.scenario,'duration',0,'distance',r.distance, ...
        'averageSpeed',0,'minimumClearance',r.minimumClearance,'collisionCount',r.collisionSteps, ...
        'collisionPartner',[],'boundaryViolations',r.boundarySteps,'replans',r.replans, ...
        'CatReflexActivations',0,'emergencySteps',r.emergencySteps,'maxSteering',0, ...
        'maxSteeringRate',0,'significantSteeringReversals',0,'maxLateralError',0, ...
        'RMSLateralError',0,'NaNInfDetected',true,'finalDecision','NO_OUTPUT', ...
        'smokePass',r.smokePass,'PASS_FAIL',false,'failureCause','','events',r.events, ...
        'steeringRateOverride',struct('steps',0,'firstTime',[], ...
        'previousState',struct([]),'currentState',struct([]),'likelySubsystem',''));
    causes={};
    if ~r.smokePass, causes{end+1}=['Smoke failure: ' r.errorIdentifier]; end
    if ~isempty(s)
        row.duration=s(end).time-s(1).time;
        row.averageSpeed=r.distance/max(row.duration,eps);
        row.collisionPartner=unique([s.collisionActorIDs]);
        active=[s.emergency] | ~strcmp({s.guardianMode},'APPROVED') | ...
            ismember({s.longitudinalCommand},{'CONTROLLED_BRAKE','MICRO_DECELERATE','EMERGENCY_BRAKE'});
        row.CatReflexActivations=sum(diff([false active])==1);
        steering=[s.steeringAngle]; turns=sign(steering(abs(steering)>deg2rad(2)));
        row.significantSteeringReversals=sum(diff(turns)~=0);
        row.maxSteering=rad2deg(max(abs(steering)));
        row.maxSteeringRate=rad2deg(max(abs([s.steeringRate])));
        config=autonex_options();
        override=abs([s.steeringRate])>config.maxSteeringRate+1e-8;
        row.steeringRateOverride.steps=sum(override);
        first=find(override,1);
        if ~isempty(first)
            row.steeringRateOverride.firstTime=s(first).time;
            row.steeringRateOverride.previousState=s(max(first-1,1));
            row.steeringRateOverride.currentState=s(first);
            row.steeringRateOverride.likelySubsystem= ...
                'Existing guardian bypass of comfort steering limiter; physical actuator feasibility not validated';
        end
        row.maxLateralError=max(abs([s.lateralError]));
        row.RMSLateralError=sqrt(mean([s.lateralError].^2));
        row.NaNInfDetected=~all(isfinite([s.x s.y s.speed s.ax s.ay s.egoYaw ...
            s.steeringAngle s.steeringRate s.lateralError s.targetHeading]));
        row.finalDecision=[s(end).selectedName ' / ' s(end).longitudinalCommand];
        if row.NaNInfDetected, causes{end+1}='Nonfinite vehicle/control state'; end
        if r.collisionSteps>0, causes{end+1}='Collision observed'; end
        if r.boundarySteps>0, causes{end+1}='Road departure observed'; end
        if r.minimumClearance<.5, causes{end+1}='Clearance below 0.5 m observation threshold'; end
        if row.RMSLateralError>.1, causes{end+1}='RMS path error above 0.10 m'; end
        emergency=[s.emergency];
        if any([s(emergency).targetSpeed]~=0) || any([s(emergency).ax]>0)
            causes{end+1}='Emergency authority violated';
        end
        if r.distance<5 && ~(any(emergency) && s(end).speed<.36)
            causes{end+1}='Insufficient progress without emergency stopping';
        end
        if row.significantSteeringReversals>max(10,row.duration)
            causes{end+1}='Sustained steering oscillation';
        end
    end
    row.PASS_FAIL=isempty(causes) && r.smokePass;
    row.failureCause=strjoin(causes,'; ');
    if isempty(rows), rows=row; else, rows(end+1)=row; end %#ok<AGROW>
end
fid=fopen(fullfile(fileparts(mfilename('fullpath')),'results','new_sih_safety_observations.json'),'w');
assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(rows,PrettyPrint=true));
t=struct2table(rows); disp(t(:,{'scenarioName','smokePass','PASS_FAIL','duration','distance', ...
    'averageSpeed','minimumClearance','collisionCount','boundaryViolations','emergencySteps','failureCause'}));
end
