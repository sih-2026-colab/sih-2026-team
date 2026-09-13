function snapshots=trace_guardian_steering_events()
% Deterministic baseline replay to recover geometry/tracks omitted from logs.
saved=jsondecode(fileread('results/guardian_steering_event_reconstruction.json')); snapshots=struct([]);
for name={'unsignalized_intersection','dense_market'}
    row=saved(strcmp({saved.scenarioName},name{1}));
    eventTimes=unique([row.time]);
    opts=autonex_options(struct('scenario',name{1},'enforcePhysicalSteeringRate',false,'validationTelemetry',true));
    state=[]; previousAngle=0;
    for t=0:opts.dt:max(eventTimes)
        [state,out]=autonex_step(state,t,opts);
        if any(abs(t-eventTimes)<1e-8)
            ego=out.actors(1); distances=arrayfun(@(a)hypot(a.x-ego.x,a.y-ego.y),out.actors(2:end));
            [~,nearest]=min(distances); actor=out.actors(nearest+1);
            frontClearance=inf; nearestTTC=inf; frontID=[];
            for j=2:numel(out.actors)
                a=out.actors(j); [L,W]=autonex_actor_size(a);
                gap=a.x-ego.x-2.25-L/2;
                if a.x>ego.x && abs(a.y-ego.y)<.95+W/2
                    if gap<frontClearance, frontClearance=gap; frontID=a.id; end
                    closing=ego.vx-a.vx;
                    if closing>0, nearestTTC=min(nearestTTC,max(0,gap)/closing); end
                end
            end
            curvature=[];
            if ~isempty(out.selected)
                p=out.selected.trajectory;
                [~,~,ax]=predict_ego_candidate_state(ego.x,ego.vx,out.selected.targetSpeedKmh,0);
                curvature=(p.vx(1)*p.ay(1)-p.vy(1)*ax)/max(hypot(p.vx(1),p.vy(1))^3,eps);
            end
            snap=struct('scenarioName',name{1},'time',t,'egoSpeedMps',out.speed/3.6, ...
                'egoYaw',out.egoYaw,'previousSteeringAngle',previousAngle, ...
                'requestedSteeringAngle',out.requestedSteeringAngle, ...
                'appliedSteeringAngle',out.appliedSteeringAngle,'dt',opts.dt, ...
                'normalRateLimit',opts.maxSteeringRate,'requestedSteeringRate',out.requestedSteeringRate, ...
                'appliedSteeringRate',out.appliedSteeringRate,'initialPathCurvature',curvature, ...
                'frontClearance',frontClearance,'frontActorID',frontID,'nearestActor',actor, ...
                'frontTTC',nearestTTC,'selectionMode',out.selectionMode,'guardianMode',out.guardianMode, ...
                'command',out.longitudinalCommand,'emergency',out.emergency,'minimumClearance',out.minClearance);
            expected=row(abs([row.time]-t)<1e-8);
            assert(abs(out.steeringAngle-expected.appliedSteeringAngle)<1e-8,'Baseline event did not reproduce');
            if isempty(snapshots), snapshots=snap; else, snapshots(end+1)=snap; end %#ok<AGROW>
        end
        previousAngle=out.steeringAngle;
    end
end
fid=fopen('results/guardian_steering_event_reconstruction.json','w');
fprintf(fid,'%s',jsonencode(snapshots,PrettyPrint=true)); fclose(fid);
fprintf('GUARDIAN_EVENTS_REPRODUCED\n');
end
