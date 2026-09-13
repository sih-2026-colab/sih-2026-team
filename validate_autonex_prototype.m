function summaries=validate_autonex_prototype()
% Five SIH capabilities plus explicit robustness/adversarial cases. No video work.
train_autonex_intent_model;
test_prototype_components;
cases={'cut_in','missing_lanes','pothole','night_pedestrian','animal', ...
    'tracking_loss','degraded_sensing','stationary_obstacle','clear_road','original_close_range'};
summaries=struct([]);
for k=1:numel(cases)
    opts=struct('scenario',cases{k},'visualize',false,'exportVideo',false, ...
        'showLaneMarkings',~strcmp(cases{k},'missing_lanes'), ...
        'reportFile',fullfile('results',['prototype_' cases{k} '.json']));
    r=run_autonex_simulation(opts);
    feature=true;
    if strcmp(cases{k},'pothole')
        feature=max([r.samples.surfaceHazardCells])>0 && min([r.samples.ax])<-.5;
    elseif ismember(cases{k},{'night_pedestrian','animal'})
        feature=sum([r.samples.thermalDetections])>0 && min([r.samples.ax])<-.5;
    elseif strcmp(cases{k},'clear_road')
        feature=r.distance>50 && ~any([r.samples.ax]<-.1);
    elseif strcmp(cases{k},'tracking_loss')
        affected=[r.samples.time]>=1.5 & [r.samples.time]<=3;
        feature=all([r.samples(affected).emergency]);
    end
    brake=find([r.samples.ax]<-.5,1); onset=NaN;
    if ~isempty(brake), onset=r.samples(brake).time; end
    finite=all(isfinite([r.samples.x r.samples.y r.samples.speed r.samples.ax r.samples.ay]));
    row=struct('scenario',cases{k},'coreSIHCase',k<=5, ...
        'passed',finite && ~r.collision && ~r.boundaryViolation && feature, ...
        'collision',r.collision,'featurePassed',feature,'boundaryViolation',r.boundaryViolation, ...
        'minimumClearanceMetres',r.minClearance,'distanceMetres',r.distance, ...
        'finalSpeedKmh',r.finalSpeed,'brakingOnsetSeconds',onset, ...
        'maximumJerk',max(abs(diff([r.samples.ax])))/r.options.dt, ...
        'tracker',r.options.trackerModel,'intentSource',r.samples(1).intentSource);
    if isempty(summaries), summaries=row; else, summaries(end+1)=row; end %#ok<AGROW>
end
fid=fopen('results/prototype_benchmark.json','w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(summaries,PrettyPrint=true)); disp(struct2table(summaries));
end
