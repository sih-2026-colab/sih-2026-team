function report=run_autonex_simulation(opts)
% Reproducible shared runner; the graphics never supply control decisions.
if nargin==0, opts=struct; end
opts=autonex_options(opts); state=[]; samples=struct([]); scene=[];
if opts.exportVideo
    opts.visualize=true; video=VideoWriter(opts.videoFile,'MPEG-4');
    video.FrameRate=1/opts.dt; open(video); cleanup=onCleanup(@()close(video)); %#ok<NASGU>
end
for t=0:opts.dt:opts.duration
    [state,out]=autonex_step(state,t,opts);
    if opts.visualize
        if isempty(scene), scene=autonex_animation_scene('create',out.actors,-1.75,8.75); end
        scene=autonex_animation_scene('update',scene,out.actors,out.candidates,out.selected,out.tracks,out);
        drawnow;
        if opts.exportVideo, writeVideo(video,getframe(scene.figure)); end
    end
    sample=rmfield(out,{'actors','candidates','selected','tracks'});
    if isempty(samples), samples=sample; else, samples(end+1)=sample; end %#ok<AGROW>
end
report=struct('options',opts,'samples',samples,'collision',any([samples.collision]), ...
    'boundaryViolation',any([samples.boundaryViolation]), ...
    'minClearance',min([samples.minClearance]),'finalSpeed',samples(end).speed, ...
    'distance',samples(end).x-samples(1).x,'emergencySteps',sum([samples.emergency]));
if ~isempty(opts.reportFile)
    file=fopen(opts.reportFile,'w'); assert(file>=0,'Cannot write report');
    cleanupReport=onCleanup(@()fclose(file)); %#ok<NASGU>
    fprintf(file,'%s',jsonencode(report,PrettyPrint=true));
end
fprintf('%s | collision=%d | boundary=%d | clearance=%.3f m | distance=%.2f m\n', ...
    opts.scenario,report.collision,report.boundaryViolation,report.minClearance,report.distance);
end
