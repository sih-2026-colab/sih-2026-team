function report=test_autonex_judge_p2()
root=fileparts(fileparts(mfilename('fullpath'))); cd(root); startup;
api=run_autonex_judge_prototype('occluded_pedestrian',struct('duration',4));
cleanup=onCleanup(@()api.close()); %#ok<NASGU>
report=struct([]);
for name={'occluded_pedestrian','unsignalized_intersection'}
    api.reset(name{1}); snapshots=cell(1,61); times=nan(1,3); stable=true;
    for k=1:61
        if k>1, api.step(); end
        s=api.snapshot(); snapshots{k}=s.out; d=s.app.explanation;
        assert(d.available && numel(d.safe)==s.out.candidateCount);
        assert(isequal(d.safe,[s.out.explainability.plannerResults.safe] & [s.out.explainability.guardianResults.safe]));
        assert(strcmp(s.app.decision.Text,s.out.motionMode));
        assert(isequal(s.app.scores.Data,d.rows)); assert(numel(s.app.events)<=10);
        if isfield(d.transitions,'visibility') && contains(d.transitions.visibility,'OCCLUDED') && isnan(times(1)), times(1)=s.out.time; end
        if isfield(d.transitions,'pedestrianDetection') && endsWith(d.transitions.pedestrianDetection,'1') && isnan(times(2)), times(2)=s.out.time; end
        if isfield(d.transitions,'pedestrianTrack') && endsWith(d.transitions.pedestrianTrack,'1') && isnan(times(3)), times(3)=s.out.time; end
        n=numel(findall(api.figure)); before=rng; a=s.app;
        for j=1:2, a=autonex_judge_update(a,s.state,s.out); end
        stable=stable && n==numel(findall(api.figure)); assert(isequal(before,rng));
        assert(isequal(a.events,s.app.events),'Duplicate render added events');
    end
    assert(stable); assert(numel(d.predictions)>=2);
    for j=1:numel(d.predictions)
        tr=s.out.tracks(j); p=d.predictions(j);
        assert(norm(p.xy(:,end)-[tr.State(1)+2*tr.State(2);tr.State(3)+2*tr.State(4)])<1e-12);
        F=zeros(2,numel(tr.State)); F(1,1:2)=[1 2]; F(2,3:4)=[1 2]; P=F*tr.StateCovariance*F';
        ellipse=p.ellipses{end}-p.xy(:,end);
        assert(max(abs(sum(ellipse.*(pinv(P)*ellipse),1)-1))<1e-6);
    end
    % Full temporal equivalence with no dashboard and telemetry disabled.
    opts=autonex_options(struct('scenario',name{1},'perceptionMode','camera_radar_lidar','duration',4));
    baseline=[]; time=0;
    for k=1:61
        [baseline,expected]=autonex_step(baseline,time,opts); time=time+opts.dt;
        actual=rmfield(snapshots{k},{'explainability','sensorFrame'});
        assert(isequaln(actual,expected),'P2 changed a pipeline output');
    end
    if strcmp(name{1},'occluded_pedestrian'), assert(times(1)==0 && times(2)>0 && times(3)>=times(2)); end
    api.figure.Position=[5 5 1200 760]; api.figure.Scrollable='off';
    s.app.live.Text='SIMULATION SNAPSHOT'; s.app.tabs.SelectedTab=s.app.pathTab; drawnow;
    suffix='intersection'; if strcmp(name{1},'occluded_pedestrian'), suffix=name{1}; end
    exportapp(api.figure,fullfile('results',['judge_p2_' suffix '.png']));
    s.app.tabs.SelectedTab=s.app.sensorTab; drawnow;
    exportapp(api.figure,fullfile('results',['judge_p2_' suffix '_sensors.png']));
    row=struct('scenario',name{1},'sameAsCoreEveryStep',true,'graphicsStable',stable, ...
        'observedOcclusionDetectionFusionTimes',times,'tracks',s.out.trackCount,'candidates',s.out.candidateCount, ...
        'collision',s.out.collision,'boundary',s.out.boundaryViolation,'passed',true);
    if isempty(report), report=row; else, report(end+1)=row; end %#ok<AGROW>
    fprintf('P2_RENDER_PASS %s\n',name{1});
end
fid=fopen('results/judge_p2_validation.json','w'); fprintf(fid,'%s',jsonencode(report,PrettyPrint=true)); fclose(fid);
test_actor_visibility;
test_occluded_pedestrian_acceptance;
test_unsignalized_intersection_acceptance;
test_steering_actuator; test_trajectory_continuity; test_closed_loop_motion;
fprintf('JUDGE_P2_VALIDATION_PASS\n');
end
