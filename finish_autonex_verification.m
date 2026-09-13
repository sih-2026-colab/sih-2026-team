function finish_autonex_verification()
% Run the current nominal suite, preserve the original hard case, then verify integration/video.
root=fileparts(mfilename('fullpath')); previous=pwd; cleanup=onCleanup(@()cd(previous)); %#ok<NASGU>
cd(root);
summaries=validate_autonex();
stress=run_autonex_simulation(struct('scenario','close_rear_cut_in', ...
    'reportFile','results/close_rear_stress.json'));
noLanes=run_autonex_simulation(struct('scenario','missing_lanes','showLaneMarkings',false, ...
    'reportFile','results/missing_lanes_validation.json'));
baseline=jsondecode(fileread('results/cut_in_validation.json'));
assert(max(abs([noLanes.samples.x]-[baseline.samples.x]))<1e-8);
assert(max(abs([noLanes.samples.y]-[baseline.samples.y]))<1e-8);
fprintf('MISSING_LANES_EQUIVALENCE_PASS\n');
verify_autonex_simulink;
animation=run_autonex_animated(root,true,struct('scenario','missing_lanes','showLaneMarkings',false, ...
    'reportFile','results/animation_run.json','videoFile',fullfile(root,'autonex_safe_space_demo.mp4')));
v=VideoReader('autonex_safe_space_demo.mp4');
frames=0;
while hasFrame(v)
    frame=readFrame(v); frames=frames+1;
    if frames==61, imwrite(frame,'results/safe_space_demo_preview.png'); end
end
assert(frames==121,'Incomplete animation export');
verification=struct('nominalPassed',all([summaries.passed]), ...
    'nominalCount',numel(summaries),'stressCollision',stress.collision, ...
    'missingLaneEquivalence',true,'videoFrames',frames, ...
    'videoSeconds',v.Duration,'videoWidth',v.Width,'videoHeight',v.Height, ...
    'videoCollision',animation.collision,'simulinkMatch',true);
fid=fopen('results/completion_verification.json','w'); closeFile=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(verification,PrettyPrint=true));
disp(verification);
end
