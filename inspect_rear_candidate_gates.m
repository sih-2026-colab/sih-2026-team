function records=inspect_rear_candidate_gates()
opts=autonex_options(struct('scenario','original_close_range','rearAuditTimes',[2 2.5 2.9 3]));
state=[]; records=struct([]);
for t=0:opts.dt:3
    [state,out]=autonex_step(state,t,opts);
    if isfield(out,'rearCandidateAudit')
        row=struct('time',t,'ego',out.actors(1),'tracks',out.tracks, ...
            'selected',out.selected,'gates',out.rearCandidateAudit);
        if isempty(records), records=row; else, records(end+1)=row; end %#ok<AGROW>
        fprintf('CANDIDATE_GATE_AUDIT t=%.2f\n',t);
    end
end
fid=fopen('results/rear_candidate_gate_audit.json','w');
fprintf(fid,'%s',jsonencode(records,PrettyPrint=true)); fclose(fid);
end
