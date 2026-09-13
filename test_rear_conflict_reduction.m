function test_rear_conflict_reduction()
center=score_rear_conflict_reduction(20,20,0,0);
partial=score_rear_conflict_reduction(20,15,.175,0);
assert(partial.rearConflictReduction==5 && partial.improvementBand==1 && ...
    partial.totalScore<center.totalScore,'A: Partial safety improvement must beat moderate comfort cost.');
noGain=score_rear_conflict_reduction(20,20,.175,0);
assert(noGain.rearRiskCost==0 && noGain.totalScore>center.totalScore, ...
    'B: Lateral displacement alone must receive no safety advantage.');
negligible=score_rear_conflict_reduction(20,19.99,.175,0);
assert(negligible.improvementBand==0 && center.totalScore<negligible.totalScore, ...
    'D: Centered motion must win when safety improvement is negligible.');
large=score_rear_conflict_reduction(20,5,.7,0);
assert(large.improvementBand==2 && large.totalScore<partial.totalScore);
% C and physical A: Exercise the actual evaluator against a hard front
% obstacle, unavailable occupancy, and a partially open lateral corridor.
test_rear_risk();
fprintf('REAR_RELATIVE_CONFLICT_A_B_C_D_PASS\n');
end
