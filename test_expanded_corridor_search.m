function report=test_expanded_corridor_search()
% Isolated experiment: do not change default planner behavior without evidence.
report=run_autonex_simulation(struct('scenario','original_close_range', ...
    'expandedSearch',true,'recoverCruise',true, ...
    'reportFile','results/expanded_corridor_experiment.json'));
fprintf('EXPANDED_SEARCH collision=%d boundary=%d\n',report.collision,report.boundaryViolation);
end
