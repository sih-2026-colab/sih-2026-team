function s=score_rear_conflict_reduction(baseline,candidate,comfort,progress)
% Dimensionless improvement bands protect safety gains from moderate comfort
% costs. No lateral displacement reward and no change to the conflict model.
reduction=baseline-candidate;
tolerance=max(1e-3,.01*baseline);
fraction=reduction/max(baseline,1e-3);
band=0;
if reduction>tolerance
    band=1; if fraction>=.5, band=2; end
elseif reduction < -tolerance
    band=-1;
end
risk=0;
if band>0, risk=-band-.5*min(1,fraction);
elseif band<0, risk=1+min(1,-fraction); end
% Bounded secondary cost cannot cancel a meaningful safety-improvement band.
total=risk+.25*(2/pi)*atan(comfort+progress);
s=struct('baselineRearConflict',baseline,'candidateRearConflict',candidate, ...
    'rearConflictReduction',reduction,'rearRiskCost',risk,'comfortCost',comfort, ...
    'progressCost',progress,'totalScore',total,'improvementBand',band);
end
