function summaries=validate_autonex()
% Deterministic simulation evidence, not a claim of real-world certification.
test_safe_corridor_extraction;
cases={'cut_in','stationary_obstacle','tracking_loss','degraded_sensing','clear_road'};
summaries=struct([]);
for k=1:numel(cases)
    opts=struct('scenario',cases{k},'reportFile',fullfile('results',[cases{k} '_validation.json']));
    r=run_autonex_simulation(opts);
    finite=all(isfinite([r.samples.x r.samples.y r.samples.speed r.samples.ax r.samples.ay]));
    response=true;
    if strcmp(cases{k},'tracking_loss')
        affected=[r.samples.time]>=1.5 & [r.samples.time]<=3;
        response=all([r.samples(affected).emergency]);
    elseif strcmp(cases{k},'clear_road')
        response=r.distance>50;
    elseif strcmp(cases{k},'stationary_obstacle')
        response=r.emergencySteps>0 || r.finalSpeed<30;
    end
    summary=struct('scenario',cases{k},'passed',finite && ~r.collision && ~r.boundaryViolation && response, ...
        'collision',r.collision,'boundaryViolation',r.boundaryViolation,'responsePassed',response, ...
        'minClearance',r.minClearance,'distance',r.distance,'finalSpeed',r.finalSpeed, ...
        'emergencySteps',r.emergencySteps);
    if isempty(summaries), summaries=summary; else, summaries(end+1)=summary; end %#ok<AGROW>
end
fid=fopen('results/validation_summary.json','w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(summaries,PrettyPrint=true));
disp(struct2table(summaries));
if ~all([summaries.passed]), warning('AutoNex:Validation','Some scenarios failed; inspect saved evidence.'); end
end
