function rows = test_new_sih_scenarios(scenarioNames)
% Full-pipeline SMOKE test, not a safety acceptance or planner tuning test.
% 10 seconds covers the nominal 3-5 s crossing conflicts. Seed/dt unchanged.
% Optional subset supports terminals with short command time limits.
root=fileparts(mfilename('fullpath'));
names={'animal_crossing','occluded_pedestrian','unsignalized_intersection','dense_market'};
if nargin>0
    assert(iscellstr(scenarioNames) && ~isempty(scenarioNames) && ...
        all(ismember(scenarioNames,names)),'AutoNex:SmokeNames','Unknown scenario subset');
    names=scenarioNames;
end
rows=struct([]);
for k=1:numel(names)
    opts=autonex_options(struct('scenario',names{k},'duration',10, ...
        'showLaneMarkings',false,'validationTelemetry',true));
    row=struct('scenario',names{k},'seed',opts.seed,'dt',opts.dt, ...
        'duration',opts.duration,'smokePass',false,'steps',0,'actorCount',0, ...
        'minimumClearance',inf,'distance',0,'candidateSteps',0,'trackSteps',0, ...
        'emergencySteps',0,'replans',0,'collisionSteps',0,'boundarySteps',0, ...
        'initialRadarActorIDs',[],'errorTime',NaN,'errorIdentifier','', ...
        'errorMessage','','events',struct([]),'samples',struct([]));
    state=[]; t=0; previousName=''; wasCollision=false; wasBoundary=false;
    wasLowClearance=false; previousSnapshot=struct([]);
    try
        initial=configure_autonex_scenario(create_highway_scenario(),names{k});
        check_actors(initial); row.actorCount=numel(initial);
        rng(opts.seed); radar=scan_autonex_radar_suite(initial);
        if ~isempty(radar), row.initialRadarActorIDs=unique([radar.actorID]); end
        for t=0:opts.dt:opts.duration
            [state,out]=autonex_step(state,t,opts);
            if t==0
                fields={'id','name','type','x','y','vx','vy','heading','uncertainty'};
                assert(isequal(rmfield(out.actors,setdiff(fieldnames(out.actors),fields)), ...
                    rmfield(initial,setdiff(fieldnames(initial),fields))), ...
                    'AutoNex:SmokeDispatch','Pipeline did not use configured actors');
            end
            check_actors(state.actors); check_actors(out.actors);
            assert(all(isfinite([state.egoYaw state.steeringAngle out.x out.y ...
                out.speed out.ax out.ay out.targetSpeed out.targetY out.steeringAngle])), ...
                'AutoNex:SmokeEgo','Nonfinite ego/control output');
            check_planner(out);
            row.steps=row.steps+1;
            row.minimumClearance=min(row.minimumClearance,out.minClearance);
            row.distance=out.x-initial(1).x;
            row.candidateSteps=row.candidateSteps+(out.candidateCount>0);
            row.trackSteps=row.trackSteps+(out.trackCount>0);
            row.emergencySteps=row.emergencySteps+out.emergency;
            row.replans=row.replans+(~isempty(previousName) && ~strcmp(previousName,out.selectedName));
            previousName=out.selectedName;
            row.collisionSteps=row.collisionSteps+out.collision;
            row.boundarySteps=row.boundarySteps+out.boundaryViolation;
            if out.collision && ~wasCollision
                row.events=add_record(row.events,safety_event(out,'COLLISION',previousSnapshot));
            end
            if out.boundaryViolation && ~wasBoundary
                row.events=add_record(row.events,safety_event(out,'BOUNDARY_VIOLATION',previousSnapshot));
            end
            if out.minClearance<.5 && ~wasLowClearance
                row.events=add_record(row.events,safety_event(out,'CLEARANCE_BELOW_0_5_M',previousSnapshot));
            end
            wasLowClearance=out.minClearance<.5;
            wasCollision=out.collision; wasBoundary=out.boundaryViolation;
            row.samples=add_record(row.samples,rmfield(out,{'actors','candidates','selected','tracks'}));
            previousSnapshot=state_snapshot(out);
        end
        assert(row.candidateSteps>0 && row.trackSteps>0, ...
            'AutoNex:SmokeInactive','No candidates or tracks produced during run');
        row.smokePass=true;
    catch problem
        row.errorTime=t; row.errorIdentifier=problem.identifier;
        row.errorMessage=getReport(problem,'extended','hyperlinks','off');
    end
    rows=add_record(rows,row);
    fprintf('SMOKE %s PASS=%d steps=%d clearance=%.3f m distance=%.2f m collisions=%d boundary=%d\n', ...
        row.scenario,row.smokePass,row.steps,row.minimumClearance,row.distance, ...
        row.collisionSteps,row.boundarySteps);
    if ~row.smokePass, fprintf('%s\n',row.errorMessage); end
    for j=1:numel(row.events)
        e=row.events(j);
        fprintf('  %.2f s %s actors=%s mode=%s / %s clearance=%.3f m\n', ...
            e.time,e.type,mat2str(e.actorIDs),e.selectionMode,e.guardianMode,e.clearance);
    end
end
folder=fullfile(root,'results');
if ~isfolder(folder), mkdir(folder); end
reportPath=fullfile(folder,'new_sih_smoke_results.json');
saved=rows;
if nargin>0 && isfile(reportPath)
    previous=jsondecode(fileread(reportPath));
    for j=1:numel(previous)
        if ~ismember(previous(j).scenario,names)
            saved=add_record(saved,previous(j));
        end
    end
end
fid=fopen(reportPath,'w');
assert(fid>=0,'AutoNex:SmokeReport','Cannot write smoke results');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(saved,PrettyPrint=true));
write_new_sih_observations(saved);
assert(all([rows.smokePass]),'AutoNex:SmokeFailed', ...
    'Setup/runtime/output smoke failure; see results/new_sih_smoke_results.json');
fprintf('NEW_SIH_SCENARIOS_SMOKE_PASS (not safety qualification)\n');
end
function check_actors(actors)
fields={'id','name','type','x','y','vx','vy','ax','ay','heading','uncertainty'};
assert(numel(actors)>=3 && all(isfield(actors,fields)), ...
    'AutoNex:SmokeSchema','Missing required actor fields or actors');
assert(actors(1).id==1 && numel(unique([actors.id]))==numel(actors), ...
    'AutoNex:SmokeIDs','Invalid ego/actor IDs');
assert(all(isfinite([actors.id actors.x actors.y actors.vx actors.vy ...
    actors.ax actors.ay actors.heading actors.uncertainty])), ...
    'AutoNex:SmokeState','Nonfinite actor state');
assert(all([actors.uncertainty]>=0) && all([actors.id]>0) && ...
    all(mod([actors.id],1)==0),'AutoNex:SmokeMetadata','Invalid actor metadata');
assert(all(ismember({actors.type},{'car','pedestrian','bike','animal','pothole'})) && ...
    all(~cellfun(@isempty,{actors.name})),'AutoNex:SmokeType','Unsupported type/empty name');
end
function check_planner(out)
assert(all(isfield(out,{'candidates','selected','selectionMode','guardianMode', ...
    'selectedName','candidateCount','emergency'})), ...
    'AutoNex:SmokePlanner','Missing planner output');
assert(~isempty(out.selectionMode) && ~isempty(out.guardianMode) && ...
    ~isempty(out.selectedName) && out.candidateCount==numel(out.candidates), ...
    'AutoNex:SmokeDecision','Invalid planner decision');
% Empty selection is a valid emergency STOP, not a malformed scenario.
assert(~isempty(out.selected) || (out.emergency && out.targetSpeed==0), ...
    'AutoNex:SmokeStop','Empty selection without emergency stop');
for k=1:numel(out.candidates)
    path=out.candidates(k).trajectory;
    assert(~isempty(path.x) && numel(path.x)==numel(path.y) && ...
        all(isfinite([path.x(:);path.y(:);path.vx(:);path.vy(:);path.ay(:);path.time(:)])), ...
        'AutoNex:SmokePath','Malformed/nonfinite candidate path');
end
for k=1:numel(out.tracks)
    assert(all(isfinite(out.tracks(k).State)) && all(isfinite(out.tracks(k).StateCovariance),'all'), ...
        'AutoNex:SmokeTrack','Nonfinite tracking state/covariance');
end
end
function event=safety_event(out,type,previousSnapshot)
ids=out.collisionActorIDs;
partners={out.actors(ismember([out.actors.id],ids)).name};
likely='Candidate risk evaluation / prediction / emergency stopping; unconfirmed';
if strcmp(type,'BOUNDARY_VIOLATION')
    likely='Road-boundary evaluation / path following; unconfirmed';
end
event=struct('time',out.time,'type',type,'actorIDs',ids, ...
    'actorNames',{partners},'clearance',out.minClearance, ...
    'selectionMode',out.selectionMode,'guardianMode',out.guardianMode, ...
    'selectedName',out.selectedName,'command',out.longitudinalCommand, ...
    'likelySubsystem',likely,'previousState',previousSnapshot,'currentState',state_snapshot(out));
end
function snapshot=state_snapshot(out)
snapshot=struct('time',out.time,'ego',out.actors(1),'actors',out.actors, ...
    'yaw',out.egoYaw,'steering',out.steeringAngle,'selectedDecision',out.selectedName, ...
    'selectionMode',out.selectionMode,'guardianMode',out.guardianMode, ...
    'command',out.longitudinalCommand,'targetSpeed',out.targetSpeed);
end
function records=add_record(records,record)
if isempty(records), records=record; else, records(end+1)=record; end
end
