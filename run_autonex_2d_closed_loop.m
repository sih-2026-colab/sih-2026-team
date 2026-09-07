function report = run_autonex_2d_closed_loop(options)
% Main entry point: observed free-space planning with shared closed-loop control.
% Use run_autonex_2d_legacy for the original lane-based baseline.
if nargin == 0, options = struct; end
if ~isfield(options,'visualize'), options.visualize = true; end
report = run_autonex_simulation(options);
end
