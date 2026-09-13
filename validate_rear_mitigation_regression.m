function checks=validate_rear_mitigation_regression()
% Run all requested regressions without hiding a later test behind an earlier failure.
checks=struct([]);
jobs={'test_rear_conflict_reduction','test_steering_actuator','test_trajectory_continuity', ...
    'test_closed_loop_path_following','test_prototype_components', ...
    'test_degraded_fusion_safety','validate_corridor_stability', ...
    'validate_autonex_scenarios','test_new_sih_scenarios'};
for k=1:numel(jobs)
    name=jobs{k}; passed=false; detail='';
    try
        switch name
            case 'validate_autonex_scenarios'
                rows=validate_autonex_scenarios();
                passed=all([rows.PASS_FAIL]) && all([rows.maxSteering]<=30+1e-8) && ...
                    all([rows.maxSteeringRate]<=45+1e-8);
                detail=strjoin({rows(~[rows.PASS_FAIL]).scenarioName},', ');
            case 'test_new_sih_scenarios'
                rows=test_new_sih_scenarios();
                passed=all([rows.smokePass]) && ~any([rows.collisionSteps]) && ...
                    ~any([rows.boundarySteps]);
                for q=1:numel(rows)
                    s=rows(q).samples;
                    passed=passed && all(abs([s.steeringAngle])<=deg2rad(30)+1e-8) && ...
                        all(abs([s.steeringRate])<=deg2rad(45)+1e-8);
                end
                detail='Smoke execution plus zero collisions and road departures';
            case 'validate_corridor_stability'
                r=validate_corridor_stability();
                passed=~r.collision && ~r.boundaryViolation && ...
                    r.jitter.stabilizedTotalVariation<=r.jitter.rawTotalVariation+1e-8;
            otherwise
                feval(name); passed=true;
        end
    catch err
        detail=getReport(err,'extended','hyperlinks','off');
    end
    row=struct('test',name,'passed',passed,'detail',detail);
    if isempty(checks), checks=row; else, checks(end+1)=row; end %#ok<AGROW>
    fid=fopen('results/rear_mitigation_regression_checks.json','w');
    fprintf(fid,'%s',jsonencode(checks,PrettyPrint=true)); fclose(fid);
    fprintf('REAR_MILESTONE_CHECK %s PASS=%d %s\n',name,passed,detail);
end
end
