function checks=validate_sih_finalization()
% Run every requested suite and retain failures; never relabel smoke as safety.
addpath('tests'); checks=struct([]);
jobs={'test_actor_visibility','test_occluded_pedestrian_acceptance', ...
    'test_unsignalized_intersection_acceptance','affected_smoke', ...
    'validate_multisensor_acceptance','run_motion_regressions', ...
    'validate_autonex_scenarios'};
for k=1:numel(jobs)
    name=jobs{k}; detail=''; passed=false;
    try
        switch name
            case 'affected_smoke'
                r=test_new_sih_scenarios({'occluded_pedestrian','unsignalized_intersection'});
                passed=all([r.smokePass]) && ~any([r.collisionSteps]) && ~any([r.boundarySteps]);
            case 'validate_multisensor_acceptance'
                r=validate_multisensor_acceptance(); passed=all([r.passed]);
                detail=strjoin({r(~[r.passed]).name},', ');
            case 'run_motion_regressions'
                r=run_motion_regressions(); passed=all([r.passed]);
                detail=strjoin({r(~[r.passed]).name},', ');
            case 'validate_autonex_scenarios'
                r=validate_autonex_scenarios(); passed=all([r.PASS_FAIL]);
                detail=strjoin({r(~[r.PASS_FAIL]).scenarioName},', ');
            otherwise
                feval(name); passed=true; r=[];
        end
        save(fullfile('results',['sih_' name '.mat']),'r');
    catch err
        detail=getReport(err,'extended','hyperlinks','off');
    end
    row=struct('test',name,'passed',passed,'detail',detail);
    if isempty(checks), checks=row; else, checks(end+1)=row; end %#ok<AGROW>
    fid=fopen('results/sih_finalization_checks.json','w');
    fprintf(fid,'%s',jsonencode(checks,PrettyPrint=true)); fclose(fid);
    fprintf('SIH_FINALIZATION %s PASS=%d %s\n',name,passed,detail);
end
end
