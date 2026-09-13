function reports=test_autonex_judge_p3()
startup; addpath('roadrunner/integration'); inspect_autonex_roadrunner;
c=autonex_demo_catalog(); api=run_autonex_judge_prototype(c(1).id,struct('duration',3));
cleanup=onCleanup(@()api.close()); %#ok<NASGU>
reports=struct([]); fid=fopen('results/judge_p3_telemetry.jsonl','w');
fileCleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
for i=1:numel(c)
    api.reset(c(i).id); s=api.snapshot();
    assert(s.out.time==0 && s.metrics.samples==1 && s.metrics.distance==0 && ~s.running);
    assert(isequal(s.app.tabs.SelectedTab,s.app.demoTab));
    assert(isequal(s.app.scenario.ItemsData,{c.id}));
    assert(all(cellfun(@(line)startsWith(line,'00.00'),s.app.events)));
    outputs=cell(1,61);
    for k=1:61
        if k>1, api.step(); end
        s=api.snapshot(); outputs{k}=s.out;
        assert(s.metrics.samples==k);
        before=rng; n=numel(findall(api.figure)); events=s.app.events;
        a=autonex_judge_update(s.app,s.state,s.out);
        fprintf('P3_HANDLE %s %d before=%d after=%d rng=%d events=%d\n',c(i).id,k,n,numel(findall(api.figure)),isequal(rng,before),isequal(events,a.events));
        assert(isequal(rng,before),'Renderer consumed random numbers');
        assert(isequal(events,a.events),'Duplicate rendering changed events');
        assert(numel(findall(api.figure))==n,'Graphics handle count grew');
        packet=autonex_rr_telemetry(s.out);
        assert(packet.egoXMetres==s.out.x && packet.egoSpeedMetresPerSecond==s.out.speed/3.6);
        fprintf(fid,'%s\n',jsonencode(struct('scenario',c(i).id,'telemetry',packet)));
    end
    assert(isequal(s.app.tabs.SelectedTab,s.app.demoTab));
    samples=[outputs{:}];
    assert(abs(s.metrics.distance-sum(hypot(diff([samples.x]),diff([samples.y]))))<1e-9);
    assert(s.metrics.brakingEvents==sum(diff([false [samples.ax]<0])==1));
    assert(s.metrics.replans==sum(~strcmp({samples(1:end-1).selectedName},{samples(2:end).selectedName})));
    assert(s.metrics.minClearance==min([samples.minClearance]));
    assert(s.metrics.maxSteeringDeg==max(abs(rad2deg([samples.steeringAngle]))));
    opts=autonex_options(struct('scenario',c(i).id,'perceptionMode','camera_radar_lidar','duration',3));
    baseline=[]; time=0;
    for k=1:61
        [baseline,expected]=autonex_step(baseline,time,opts); time=time+opts.dt;
        assert(isequaln(rmfield(outputs{k},{'explainability','sensorFrame'}),expected),'P3 changed core output');
    end
    drawnow; pause(.3);
    exportapp(api.figure,fullfile('results',['judge_p3_' c(i).id '_result.png']));
    s.app.tabs.SelectedTab=s.app.pathTab; drawnow; pause(.3);
    exportapp(api.figure,fullfile('results',['judge_p3_' c(i).id '.png']));
    row=struct('scenario',c(i).id,'exactCoreMatch',true,'stableHandles',true,'metricsVerified',true,'resetVerified',true,'demoMetrics',s.metrics);
    if isempty(reports), reports=row; else, reports(end+1)=row; end %#ok<AGROW>
    api.next(); reset=api.snapshot(); assert(strcmp(reset.app.scenario.Value,c(mod(i,numel(c))+1).id));
    assert(reset.metrics.samples==1 && reset.metrics.distance==0 && reset.out.time==0 && ~reset.running);
    fprintf('P3_UI_PASS %s\n',c(i).id);
end
% Timer lifecycle: automatic stop/result, slow-motion interval, reset and no stale state.
api.close(); clear cleanup;
short=run_autonex_judge_prototype(c(1).id,struct('duration',.1));
timerCleanup=onCleanup(@()short.close()); %#ok<NASGU>
short.reset(c(1).id); st=short.snapshot(); st.app.rate.Value='0.5x'; st.app.rate.ValueChangedFcn([],[]);
short.start(); start=tic;
while toc(start)<30
    pause(.1); st=short.snapshot(); if ~st.running, break; end
end
assert(~st.running && st.out.time>=.1-1e-8 && isequal(st.app.tabs.SelectedTab,st.app.demoTab));
% Full ten-second smoke for all six; report observed safety without retuning.
smoke=struct([]);
for i=1:numel(c)
    opts=autonex_options(struct('scenario',c(i).id,'perceptionMode','camera_radar_lidar','duration',10));
    state=[]; m=[];
    for time=0:opts.dt:10, [state,out]=autonex_step(state,time,opts); m=autonex_demo_metrics(m,out); end
    row=struct('scenario',c(i).id,'finite',m.finite,'collision',m.collision,'boundary',m.boundary,'minimumClearance',m.minClearance,'distance',m.distance);
    if isempty(smoke), smoke=row; else, smoke(end+1)=row; end %#ok<AGROW>
    assert(m.finite,'Nonfinite full-run state');
    fprintf('P3_SMOKE %s collision=%d boundary=%d clearance=%.3f\n',c(i).id,m.collision,m.boundary,m.minClearance);
end
test_actor_visibility; test_occluded_pedestrian_acceptance; test_unsignalized_intersection_acceptance;
test_steering_actuator; test_trajectory_continuity; test_closed_loop_motion;
f=fopen('results/judge_p3_validation.json','w'); fprintf(f,'%s',jsonencode(struct('ui',reports,'smoke',smoke,'timerPassed',true,'regressionsPassed',true),PrettyPrint=true)); fclose(f);
fprintf('JUDGE_P3_VALIDATION_PASS\n');
end
