function coverage = report_sih_scenario_coverage()
% Evidence-aware coverage; a smoke pass is NOT a safety qualification.
root=fileparts(mfilename('fullpath'));
requirements={'Missing/unmarked lane';'Highway cut-in';'Pedestrian (night)'; ...
    'Sudden occluded pedestrian';'Animal crossing';'Degraded sensing'; ...
    'Unsignalized intersection';'Dense market'};
names={'missing_lane';'cut_in';'night_pedestrian';'occluded_pedestrian'; ...
    'animal_crossing';'degraded_sensing';'unsignalized_intersection';'dense_market'};
status=repmat({'NOT YET VALIDATED'},numel(names),1);
tested=false(numel(names),1);
files={'multi_scenario_regression.json','new_sih_smoke_results.json'};
for f=1:numel(files)
    path=fullfile(root,'results',files{f});
    if ~isfile(path), continue; end
    saved=jsondecode(fileread(path));
    for k=1:numel(names)
        if f==1
            idx=find(strcmp({saved.scenarioName},names{k}),1);
            if isempty(idx), continue; end
            tested(k)=true;
            if saved(idx).PASS_FAIL, status{k}='Saved regression PASS';
            else, status{k}='Saved regression FAIL'; end
        else
            idx=find(strcmp({saved.scenario},names{k}),1);
            if isempty(idx), continue; end
            r=saved(idx);
            tested(k)=r.steps>0;
            if ~r.smokePass, status{k}='Smoke FAIL (setup/runtime/output)';
            elseif r.collisionSteps>0 || r.boundarySteps>0
                status{k}='Smoke PASS; safety failure observed';
            else, status{k}='Smoke PASS; safety not qualified'; end
        end
    end
end
observationPath=fullfile(root,'results','new_sih_safety_observations.json');
if isfile(observationPath)
    observed=jsondecode(fileread(observationPath));
    for k=1:numel(names)
        idx=find(strcmp({observed.scenarioName},names{k}),1);
        if isempty(idx), continue; end
        if observed(idx).PASS_FAIL, status{k}='PASS (bounded safety observation)';
        else, status{k}=['FAIL: ' observed(idx).failureCause]; end
    end
end
limitations={'Two-blocker straight road';'Scripted highway traffic'; ...
    'Synthetic night sensing';'Depth occlusion only; radar/RGB/thermal see hidden actors'; ...
    'Constant-velocity animal; actor uncertainty metadata is not a motion model'; ...
    'Synthetic sensor-health degradation';'Straight-road crossing; no junction topology/right-of-way'; ...
    'Scripted traffic; bike uses car-sized footprint'};
coverage=table(requirements,names,true(numel(names),1),tested,status,limitations, ...
    'VariableNames',{'SIHRequirement','ScenarioIdentifier','Implemented', ...
    'FullPipelineTested','ValidationStatus','Limitations'});
disp(coverage);
fid=fopen(fullfile(root,'results','sih_scenario_coverage.json'),'w');
assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(table2struct(coverage),PrettyPrint=true));
fprintf(['FullPipeline means shared autonex_step execution, not feature completeness.\n' ...
    'occluded_pedestrian: geometry only; radar/RGB/thermal lack actor occlusion.\n' ...
    'unsignalized_intersection: crossing conflict on straight strip; no junction topology.\n' ...
    'dense_market: bike uses existing car-sized footprint; no auto/pushcart type added.\n' ...
    'animal remains slow-ahead; animal_crossing is separate.\n' ...
    'Legacy missing_lanes falls through to cut_in; it is NOT missing_lane coverage.\n' ...
    'Saved results may predate source changes; rerun tests for current evidence.\n']);
end
