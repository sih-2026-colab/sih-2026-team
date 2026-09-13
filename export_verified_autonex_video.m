function verification=export_verified_autonex_video()
root=fileparts(mfilename('fullpath'));
animation=run_autonex_animated(root,true,struct('scenario','missing_lanes','showLaneMarkings',false, ...
    'reportFile',fullfile(root,'results','animation_run.json'), ...
    'videoFile',fullfile(root,'autonex_safe_space_demo.mp4')));
v=VideoReader(fullfile(root,'autonex_safe_space_demo.mp4')); frames=0;
while hasFrame(v)
    frame=readFrame(v); frames=frames+1;
    if frames==61, imwrite(frame,fullfile(root,'results','safe_space_demo_preview.png')); end
end
assert(frames==121 && v.Width==1280 && v.Height==720,'Incomplete or incorrectly sized video');
verification=struct('frames',frames,'seconds',v.Duration,'width',v.Width,'height',v.Height, ...
    'collision',animation.collision,'boundaryViolation',animation.boundaryViolation);
fid=fopen(fullfile(root,'results','video_verification.json'),'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(verification,PrettyPrint=true)); disp(verification);
end
