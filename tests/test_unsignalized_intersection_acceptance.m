function report=test_unsignalized_intersection_acceptance()
report=run_sih_acceptance('unsignalized_intersection');
assert(report.pass,'AutoNex:JunctionAcceptance','See results/unsignalized_intersection_acceptance.json');
end
