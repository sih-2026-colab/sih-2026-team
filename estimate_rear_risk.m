function risk=estimate_rear_risk(tracks,ego,acceleration,opts)
% Confirmed track IDs, not scenario actor IDs. Anticipate the effect of braking.
risk=struct('rearRiskActive',false,'rearRiskLevel','NONE','rearThreatActorId',0, ...
    'rearDistance',inf,'rearLateralOffset',inf,'rearClosingSpeed',0,'rearTTC',inf, ...
    'rearPredictedGap',inf,'rearImpactUnavoidable',false);
h=0:.1:opts.rearRiskHorizon;
v=max(0,ego.vx); a=min(acceleration,0);
yaw=atan2(ego.vy,ego.vx);
lateralFootprint=2.25*abs(sin(yaw))+.95*abs(cos(yaw))+.95;
if a<0, travel=v*min(h,-v/a)+.5*a*min(h,-v/a).^2; else, travel=v*h; end
for k=1:numel(tracks)
    s=tracks(k).State; gap=ego.x-s(1)-4.5;
    if s(1)>=ego.x-2.25, continue; end
    sigma=sqrt(max(tracks(k).StateCovariance(3,3),0));
    offset=s(3)+s(4)*h-(ego.y+ego.vy*h);
    overlap=abs(offset)<max(2.2,lateralFootprint)+min(sigma,1);
    gaps=gap+travel-s(2)*h;
    futureClosing=s(2)-max(0,v+a*h);
    threat=overlap & futureClosing>.1 & gaps<6;
    closing=s(2)-ego.vx; ttc=inf;
    if closing>.1 && overlap(1), ttc=max(gap,0)/closing; end
    predicted=inf; if any(overlap), predicted=min(gaps(overlap)); end
    if any(threat) && (~risk.rearRiskActive || predicted<risk.rearPredictedGap)
        risk.rearRiskActive=true; risk.rearRiskLevel='PREDICTED_BRAKING_CONFLICT';
        if ttc<2, risk.rearRiskLevel='HIGH'; end
        risk.rearThreatActorId=tracks(k).TrackID; risk.rearDistance=gap;
        risk.rearLateralOffset=s(3)-ego.y; risk.rearClosingSpeed=closing;
        risk.rearTTC=ttc; risk.rearPredictedGap=predicted;
    elseif ~risk.rearRiskActive && predicted<risk.rearPredictedGap
        risk.rearThreatActorId=tracks(k).TrackID; risk.rearDistance=gap;
        risk.rearLateralOffset=s(3)-ego.y; risk.rearClosingSpeed=closing;
        risk.rearTTC=ttc; risk.rearPredictedGap=predicted;
    end
end
% No claim of inevitability: finite sampled candidates do not prove that
% every feasible ego action fails, nor that the rear actor cannot brake.
end
