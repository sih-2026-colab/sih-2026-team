function report=verify_original_close_range()
% Preserve the initial hard scenario independently of newer demonstration spacing.
report=run_autonex_simulation(struct('scenario','original_close_range', ...
    'reportFile','results/original_close_range_validation.json'));
fprintf('ORIGINAL_CASE: rear=-18m cut-in=4.1m collision=%d\n',report.collision);
end
