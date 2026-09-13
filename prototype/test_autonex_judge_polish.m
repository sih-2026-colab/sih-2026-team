function report=test_autonex_judge_polish()
% Visual-only acceptance. Existing algorithm regressions remain unchanged.
startup; api=run_autonex_judge_prototype('occluded_pedestrian',struct('duration',4));
cleanup=onCleanup(@()api.close()); %#ok<NASGU>
report=struct([]); captures=struct; captured=struct('hidden',false,'detected',false,'decision',false);
for name={'occluded_pedestrian','unsignalized_intersection'}
    api.reset(name{1}); outputs=cell(1,61);
    for k=1:61
        if k>1, api.step(); end
        s=api.snapshot(); outputs{k}=s.out; a=s.app; d=a.explanation;
        assert(strcmp(a.decision.Text,s.out.motionMode)); assert(isequal(a.scores.Data,d.rows));
        before=rng; n=numel(findall(api.figure)); events=a.events;
        a=autonex_judge_update(a,s.state,s.out);
        assert(isequal(rng,before) && numel(findall(api.figure))==n && isequal(events,a.events));
        assert(numel(a.events)<=10);
        assert(isequal(a.top.DataAspectRatio,[1 1 1]));
        assert(max(abs(a.top.XLim-[s.out.x-15 s.out.x+55]))<1e-9);
        if strcmp(name{1},'occluded_pedestrian')
            if ~captured.hidden && isfield(d.transitions,'visibility') && contains(d.transitions.visibility,'OCCLUDED')
                captures.hidden=fixture(s); captured.hidden=true;
            end
            if ~captured.detected && isfield(d.transitions,'pedestrianDetection') && endsWith(d.transitions.pedestrianDetection,'1') && endsWith(d.transitions.pedestrianTrack,'1')
                captures.detected=fixture(s); captured.detected=true;
            end
            if ~captured.decision && ~isempty(d.selectedIndex) && numel(d.rows(:,1))>=4 && any(~d.safe) && s.out.trackCount>0
                captures.decision=fixture(s); captured.decision=true;
            end
        end
    end
    opts=autonex_options(struct('scenario',name{1},'perceptionMode','camera_radar_lidar','duration',4));
    baseline=[]; time=0;
    for k=1:61
        [baseline,expected]=autonex_step(baseline,time,opts); time=time+opts.dt;
        assert(isequaln(rmfield(outputs{k},{'explainability','sensorFrame'}),expected));
    end
    row=struct('scenario',name{1},'steps',61,'exactHeadlessMatch',true,'stableHandles',true,'rngPreserved',true);
    if isempty(report), report=row; else, report(end+1)=row; end %#ok<AGROW>
    fprintf('POLISH_EQUIVALENCE_PASS %s\n',name{1});
end
assert(all(structfun(@(v)v,captured)),'Required real event was not observed');
save('results/judge_polish_fixtures.mat','captures');
render_autonex_judge_polish;
test_actor_visibility; test_occluded_pedestrian_acceptance; test_unsignalized_intersection_acceptance;
test_steering_actuator; test_trajectory_continuity; test_closed_loop_motion;
fid=fopen('results/judge_polish_validation.json','w'); fprintf(fid,'%s',jsonencode(report,PrettyPrint=true)); fclose(fid);
fprintf('JUDGE_POLISH_VALIDATION_PASS\n');
end
function f=fixture(s)
% Keep only value data needed by rendering; no tracker/timer handles.
f=struct('out',s.out,'state',struct('drivable',s.state.drivable,'corridor',s.state.corridor),'events',{s.app.events});
end
