function report=test_occluded_pedestrian_acceptance()
report=run_sih_acceptance('occluded_pedestrian');
assert(report.pass,'AutoNex:OcclusionAcceptance','See results/occluded_pedestrian_acceptance.json');
end
