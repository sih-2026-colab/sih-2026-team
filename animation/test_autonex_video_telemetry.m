function reports=test_autonex_video_telemetry(duration)
% Independent no-telemetry baseline, disk round trips and event edge checks.
if nargin<1, duration=10; end
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,'-begin');
% Resume event/nonfinite checks without invoking exporter or simulator.
if isequal(duration,'saved')
    reports=testSavedExports(root);
    return;
end
names={'occluded_pedestrian','cut_in','animal_crossing','dense_market','missing_lane','intersection'};
reports=struct([]);
for i=1:numel(names)
    before=rng; data=export_autonex_video_telemetry(names{i},struct('duration',duration));
    assert(isequal(before,rng),'Exporter changed caller RNG state');
    opts=autonex_options(struct('scenario',data.coreScenario,'duration',duration, ...
        'perceptionMode','camera_radar_lidar'));
    state=[]; ts=cellfun(@(f)f.time,data.frames);
    assert(all(diff(ts)>0) && ts(1)==0,'Nonmonotonic frame timestamps');
    if ~isempty(data.events), assert(all(diff([data.events.time])>=0)); end
    observedCollision=false; observedBoundary=false; minimumClearance=inf;
    nonfiniteCount=0;
    for k=1:numel(ts)
        [state,out]=autonex_step(state,ts(k),opts);
        expected=out; expected.tracks=struct([]);
        for j=1:numel(out.tracks)
            tr=out.tracks(j);
            trackSnapshot=struct('TrackID',tr.TrackID,'State',tr.State, ...
                'StateCovariance',tr.StateCovariance,'ObjectAttributes',{tr.ObjectAttributes});
            if isempty(expected.tracks), expected.tracks=trackSnapshot; else, expected.tracks(j)=trackSnapshot; end
        end
        expected=autonex_video_sanitize(expected);
        f=data.frames{k}; actual=rmfield(f.source,{'sensorFrame','explainability'});
        assert(isequaln(actual,expected),'Baseline mismatch: %s frame %d',names{i},k);
        assert(isequal(f.ego.x,out.x) && isequal(f.ego.y,out.y) && ...
            isequal(f.ego.speedMps,out.speed/3.6) && strcmp(f.decision,out.motionMode));
        assert(numel(f.candidates)==numel(out.candidates));
        for j=1:numel(f.candidates)
            assert(isequaln(f.candidates(j).trajectory,out.candidates(j).trajectory), ...
                'Candidate path changed');
        end
        assert(isequaln(f.selected,autonex_video_sanitize(out.selected)));
        [~,left]=autonex_video_sanitize(f); assert(isempty(left),'Unreported nonfinite export');
        nonfiniteCount=nonfiniteCount+numel(f.nonfinite);
        observedCollision=observedCollision || out.collision;
        observedBoundary=observedBoundary || out.boundaryViolation;
        minimumClearance=min(minimumClearance,out.minClearance);
    end
    folder=fullfile(root,'results','animation',names{i});
    saved=load(fullfile(folder,'telemetry.mat'),'telemetry'); assert(isequaln(saved.telemetry,data));
    encoded=fileread(fullfile(folder,'telemetry.json'));
    assert(~contains(encoded,':NaN') && ~contains(encoded,':Inf') && ~contains(encoded,':-Inf'));
    decoded=jsondecode(encoded);
    assert(isequaln(decoded,jsondecode(jsonencode(data,ConvertInfAndNaN=false))));
    ego=readtable(fullfile(folder,'ego.csv'),'TextType','string');
    assert(height(ego)==numel(ts) && max(abs(ego.time-ts'))<1e-12);
    for k=1:numel(ts)
        f=data.frames{k};
        assert(abs(ego.x(k)-f.ego.x)<1e-12 && abs(ego.y(k)-f.ego.y)<1e-12);
        assert(strcmp(ego.decision(k),f.decision));
    end
    assert(height(readtable(fullfile(folder,'tracks.csv')))==sum(cellfun(@(f)numel(f.tracks),data.frames)));
    assert(height(readtable(fullfile(folder,'candidates.csv')))==sum(cellfun(@(f)numel(f.candidates),data.frames)));
    assert(isequaln(jsondecode(fileread(fullfile(folder,'events.json'))),jsondecode(jsonencode(data.events))));
    report=struct('scenario',names{i},'samples',numel(ts),'duration',duration, ...
        'exactBaselineMatch',true,'egoAndDecisionsMatch',true,'candidatePathsMatch',true, ...
        'monotonic',true,'diskRoundTrip',true,'callerRngPreserved',true, ...
        'nonfiniteValuesExplicitlyRecorded',nonfiniteCount,'events',numel(data.events), ...
        'observedCollision',observedCollision,'observedBoundaryViolation',observedBoundary, ...
        'minimumClearance',minimumClearance);
    if isempty(reports), reports=report; else, reports(i)=report; end
    fprintf('VALIDATION_PASS %s samples=%d collision=%d boundary=%d clearance=%.6f\n', ...
        names{i},numel(ts),observedCollision,observedBoundary,minimumClearance);
end
% Real first frame repeated: no duplicate event emission; no invented prediction.
f=data.frames{1}; [a,m]=autonex_video_events(f,[]); [b,~]=autonex_video_events(f,m);
assert(isempty(b) && ~any(strcmp({a.event},'PREDICTION_ACTIVE')));
[v,issues]=autonex_video_sanitize([1 NaN Inf -Inf]);
assert(isequal(v,{1 [] [] []}) && isequal({issues.kind},{'NaN','+Inf','-Inf'}));
target=fullfile(root,'results','animation','validation.json');
fid=fopen(target,'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(struct('scenarios',reports,'eventIdempotence',true,'nonfinitePolicyTest',true),PrettyPrint=true));
fprintf('ANIMATION_TELEMETRY_VALIDATION_PASS\n');
end

function reports=testSavedExports(root)
names={'occluded_pedestrian','cut_in','animal_crossing','dense_market','missing_lane','intersection'};
reports=struct([]);
for i=1:numel(names)
    saved=load(fullfile(root,'results','animation',names{i},'telemetry.mat'),'telemetry');
    data=saved.telemetry; memory=[]; replay=data.events([]); before=rng;
    for k=1:numel(data.frames)
        f=data.frames{k};
        [events,memory]=autonex_video_events(f,memory);
        [duplicate,~]=autonex_video_events(f,memory);
        assert(isempty(duplicate),'Repeated frame emitted duplicate events');
        assert(~any(strcmp({events.event},'PREDICTION_ACTIVE')),'Invented prediction event');
        replay=[replay events]; %#ok<AGROW>
    end
    assert(isequaln(replay,data.events),'Saved-event replay mismatch: %s',names{i});
    assert(isequal(before,rng),'Event replay changed RNG');
    row=struct('scenario',names{i},'frames',numel(data.frames),'eventReplayMatch',true,'idempotence',true);
    if isempty(reports), reports=row; else, reports(i)=row; end
end
% Synthetic TTC inputs exercise the animation adapter only; never exported.
f=data.frames{1}; assert(~isempty(f.candidates)); f.candidates=f.candidates(1);
cases={[],[],false; Inf,Inf,false; 2,1.5,false; [3 1],[],true; [],[2 1],true};
for k=1:size(cases,1)
    f.candidates.guardian.minFrontTTC=cases{k,1};
    f.candidates.guardian.minRearTTC=cases{k,2};
    [events,~]=autonex_video_events(f,[]);
    assert(any(strcmp({events.event},'TTC_CRITICAL'))==cases{k,3},'TTC scalar-safety regression');
end
[v,issues]=autonex_video_sanitize([1 NaN Inf -Inf]);
assert(isequal(v,{1 [] [] []}) && isequal({issues.kind},{'NaN','+Inf','-Inf'}));
fid=fopen(fullfile(root,'results','animation','targeted_validation.json'),'w'); assert(fid>=0);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(struct('scenarios',reports,'ttcScalarSafety',true, ...
    'nonfinitePolicyTest',true,'simulationRuns',0),PrettyPrint=true));
fprintf('test_autonex_video_telemetry SAVED_EXPORTS_PASS: 1206 frames, 0 simulation runs\n');
end
