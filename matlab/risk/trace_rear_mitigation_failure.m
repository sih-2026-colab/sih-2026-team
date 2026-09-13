function trace=trace_rear_mitigation_failure()
% Replay the original failure and compare held-path rollout with actual replanning.
opts=autonex_options(struct('scenario','original_close_range','validationTelemetry',true));
record=jsondecode(fileread('results/rear_mitigation_original_close_range.json'));
state=[]; trace=struct([]);
for t=0:opts.dt:4
    [state,out]=autonex_step(state,t,opts);
    if any(abs(t-[3 3.5 4])<1e-8) && ~isempty(out.selected) && isfield(out.selected,'rollout')
        predicted=out.selected.rollout;
        actual=record.samples([record.samples.time]>=t & [record.samples.time]<=t+1);
        row=struct('time',t,'candidate',out.selected,'rear',out.rearActionDiagnostics, ...
            'actualTime',[actual.time]-t,'actualX',[actual.x],'actualY',[actual.y], ...
            'predictedYAtOneSecond',interp1(predicted.time,predicted.y,1));
        if isempty(trace), trace=row; else, trace(end+1)=row; end %#ok<AGROW>
        fprintf('REAR_TRACE t=%.2f predictedY(1s)=%.3f actualY(1s)=%.3f\n', ...
            t,row.predictedYAtOneSecond,row.actualY(end));
    end
end
fid=fopen('results/rear_mitigation_failure_trace.json','w');
fprintf(fid,'%s',jsonencode(trace,PrettyPrint=true)); fclose(fid);
end
