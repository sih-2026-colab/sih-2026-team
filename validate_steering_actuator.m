function rows=validate_steering_actuator(phase)
% Baseline uses saved full runs; counterfactual is opt-in, default unchanged.
if nargin==0, phase='after'; end
if strcmp(phase,'before') && isfile('results/actuator_before_summary.json')
    % Preserve the pre-change evidence after smoke outputs are refreshed.
    rows=jsondecode(fileread('results/actuator_before_summary.json'));
    disp(struct2table(rows)); return;
end
if strcmp(phase,'before')
    baselineConfig=autonex_options();
    assert(~baselineConfig.enforcePhysicalSteeringRate, ...
        'Baseline archive missing: do not relabel current physical runs as before evidence');
end
names={'unsignalized_intersection','dense_market','animal_crossing','missing_lane','night_pedestrian','cut_in'};
rows=struct([]);
if strcmp(phase,'counterfactual'), names=names(1:2); end
if strcmp(phase,'before'), saved=jsondecode(fileread('results/new_sih_smoke_results.json')); end
for k=1:numel(names)
    name=names{k}; duration=10;
    if strcmp(name,'missing_lane'), duration=25; end
    opts=autonex_options(struct('scenario',name,'duration',duration,'validationTelemetry',true));
    if strcmp(phase,'before')
        if ismember(name,{'unsignalized_intersection','dense_market','animal_crossing'})
            match=find(strcmp({saved.scenario},name),1);
            r=struct('samples',saved(match).samples,'options',opts);
        else
            r=jsondecode(fileread(fullfile('results',['scenario_' name '.json'])));
        end
    else
        opts.enforcePhysicalSteeringRate=true;
        opts.reportFile=fullfile('results',['actuator_' phase '_' name '.json']);
        r=run_autonex_simulation(opts);
    end
    row=steering_actuator_metrics(r);
    if isempty(rows), rows=row; else, rows(end+1)=row; end %#ok<AGROW>
    fid=fopen(fullfile('results',['actuator_' phase '_summary.json']),'w');
    fprintf(fid,'%s',jsonencode(rows,PrettyPrint=true)); fclose(fid);
    fprintf('ACTUATOR %s %s PASS=%d applied=%.3f deg/s clearance=%.3f m\n', ...
        phase,name,row.PASS,row.maxAppliedSteeringRate,row.minimumClearance);
end
disp(struct2table(rows));
if ~strcmp(phase,'before')
    assert(all([rows.PASS]),'AutoNex:ActuatorFeasibility','Physical-actuator safety regression; inspect reports');
end
end
