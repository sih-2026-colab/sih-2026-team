function results=test_autonex_judge_prototype()
root=fileparts(fileparts(mfilename('fullpath'))); cd(root); startup;
api=run_autonex_judge_prototype('occluded_pedestrian',struct('uiVisible','on','duration',4));
cleanup=onCleanup(@()api.close()); %#ok<NASGU>
results=struct([]);
for name={'occluded_pedestrian','unsignalized_intersection'}
    api.reset(name{1}); s=api.snapshot(); initialX=s.out.x; initialTracks=s.out.trackCount;
    for k=1:60, api.step(); end
    s=api.snapshot(); out=s.out; a=s.app;
    assert(out.x>initialX && out.time>=3-1e-8);
    assert(strcmp(a.values.trackCount.Text,sprintf('%d',out.trackCount)));
    assert(strcmp(a.values.candidateCount.Text,sprintf('%d',out.candidateCount)));
    assert(strcmp(a.values.selectedName.Text,out.selectedName));
    assert(strcmp(a.values.collision.Text,string(out.collision)));
    assert(strcmp(a.values.boundaryViolation.Text,string(out.boundaryViolation)));
    assert(out.trackCount~=initialTracks,'Expected confirmed tracks to initialize');
    % Updating graphics cannot change state, output, or random-number stream.
    beforeState=s.state; beforeOut=out; beforeRng=rng;
    objectCount=numel(findall(api.figure));
    for k=1:5, a=autonex_judge_update(a,s.state,out); end
    assert(isequaln(beforeState,s.state) && isequaln(beforeOut,out) && isequal(rng,beforeRng));
    assert(numel(findall(api.figure))==objectCount,'Graphics objects grew on repeated updates');
    % Compare the whole run to the original pipeline with no renderer.
    opts=autonex_options(struct('scenario',name{1},'perceptionMode','camera_radar_lidar','duration',4));
    baseline=[]; t=0;
    for step=0:60
        [baseline,expected]=autonex_step(baseline,t,opts); t=t+opts.dt;
    end
    assert(isequaln(out,expected),'Dashboard changed the real pipeline output');
    api.figure.Position=[30 30 1600 900]; drawnow;
    exportapp(api.figure,fullfile(root,'results',['judge_' name{1} '.png']));
    row=struct('scenario',name{1},'time',out.time,'distance',out.x-initialX, ...
        'tracks',out.trackCount,'candidates',out.candidateCount,'collision',out.collision, ...
        'boundary',out.boundaryViolation,'sameAsCore',true,'graphicsStable',true,'passed',true);
    if isempty(results), results=row; else, results(end+1)=row; end %#ok<AGROW>
    fprintf('JUDGE_DASHBOARD_PASS %s\n',name{1});
end
api.reset('occluded_pedestrian'); api.start(); pause(.5); api.pause();
s=api.snapshot(); assert(~s.running && s.out.time>0,'Start/pause timer failed');
pausedTime=s.out.time; pause(.15); s=api.snapshot(); assert(s.out.time==pausedTime);
api.pause(); pause(.2); api.pause(); s=api.snapshot(); assert(s.out.time>pausedTime,'Resume failed');
api.reset('unsignalized_intersection'); s=api.snapshot(); assert(s.out.time==0 && ~s.running);
api.figure.Position=[30 30 1280 720]; drawnow;
exportapp(api.figure,fullfile(root,'results','judge_laptop_layout.png'));
fid=fopen(fullfile(root,'results','judge_prototype_validation.json'),'w');
fprintf(fid,'%s',jsonencode(results,PrettyPrint=true)); fclose(fid);
test_steering_actuator; test_trajectory_continuity; test_closed_loop_motion;
fprintf('JUDGE_P1_VALIDATION_PASS\n');
end
