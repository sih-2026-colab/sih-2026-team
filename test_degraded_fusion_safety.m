function test_degraded_fusion_safety()
% Matched-state regression for the observed health-normalization defect.
track=struct('StateCovariance',.1*eye(6));
nominal=struct('lightLevel',1,'visibility',1,'radarQuality',.95,'thermalQuality',.85);
bad=struct('lightLevel',.1,'visibility',.2,'radarQuality',.45,'thermalQuality',.5);
night=nominal; night.lightLevel=.02; night.visibility=.5;
failed=struct('lightLevel',0,'visibility',0,'radarQuality',0,'thermalQuality',0);
for rgb=[.1 .8 NaN]
    for thermal=[.8 NaN]
        baseline=adaptive_multimodal_fusion(track,rgb,thermal,nominal);
        for env={bad,night,failed}
            degraded=adaptive_multimodal_fusion(track,rgb,thermal,env{1});
            assert(degraded.fusedConfidence<=baseline.fusedConfidence+1e-12,'Lower health increased confidence');
            assert(degraded.fusedUncertainty>=baseline.fusedUncertainty,'Lower health reduced uncertainty');
            if ~isnan(rgb) || isequal(env{1},bad)
                assert(degraded.fusedUncertainty>baseline.fusedUncertainty,'Quality loss was ignored');
            end
            a=calculate_contextual_safety_envelope(baseline.fusedUncertainty,baseline.fusedConfidence,5,.5,true);
            b=calculate_contextual_safety_envelope(degraded.fusedUncertainty,degraded.fusedConfidence,5,.5,true);
            assert(b.longitudinal>=a.longitudinal && b.lateral>=a.lateral,'Quality loss shrank safety envelope');
        end
    end
end
fprintf('DEGRADED_FUSION_SAFETY_PASS\n');
end
