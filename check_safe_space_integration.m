test_safe_corridor_extraction;
actors=create_highway_scenario();
[d,s,c]=build_autonex_safe_space(actors);
disp(c);
fprintf('Safe cells %d\n',nnz(s.safeFreeMask));
save('results/corridor_diagnostic.mat','d','s','c');
fig=figure('Visible','off'); imagesc(d.xValues,d.yValues,s.safeFreeMask); axis xy; hold on;
plot(actors(1).x,actors(1).y,'rx');
if c.valid, plot(c.x,c.referenceY,'r-','LineWidth',2); end
saveas(fig,'results/corridor_diagnostic.png'); close(fig);
report=run_autonex_simulation(struct('duration',.5,'reportFile','results/corridor_smoke.json'));
assert(all(isfinite([report.samples.x])));
disp('SAFE_SPACE_INTEGRATION_PASS');
