function audit=autonex_perception_audit(tracks,fused,ego,environment)
% Read-only diagnostics: compare identical tracks/observations under nominal quality.
audit=struct('trackCount',numel(tracks),'uncertaintySum',0,'longitudinalMax',0, ...
    'lateralMax',0,'qualityUncertaintyDelta',0,'qualityMarginDelta',0);
nominal=struct('lightLevel',1,'visibility',1,'radarQuality',.95,'thermalQuality',.85);
for k=1:numel(tracks)
    f=fused(k); s=tracks(k).State;
    reference=adaptive_multimodal_fusion(tracks(k),f.rgbConfidence,f.thermalConfidence,nominal);
    closing=ego.vx-s(2); lateral=s(4)-ego.vy;
    toward=(ego.y-s(3))*lateral>0;
    actual=calculate_contextual_safety_envelope(f.fusedUncertainty,f.fusedConfidence,closing,lateral,toward);
    base=calculate_contextual_safety_envelope(reference.fusedUncertainty,reference.fusedConfidence,closing,lateral,toward);
    audit.uncertaintySum=audit.uncertaintySum+f.fusedUncertainty;
    audit.longitudinalMax=max(audit.longitudinalMax,actual.longitudinal);
    audit.lateralMax=max(audit.lateralMax,actual.lateral);
    audit.qualityUncertaintyDelta=audit.qualityUncertaintyDelta+f.fusedUncertainty-reference.fusedUncertainty;
    audit.qualityMarginDelta=audit.qualityMarginDelta+actual.longitudinal-base.longitudinal;
end
end
