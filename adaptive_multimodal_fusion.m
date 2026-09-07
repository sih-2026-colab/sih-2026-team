function fused = adaptive_multimodal_fusion( ...
    track, ...
    rgbConfidence, ...
    thermalConfidence, ...
    environment)

    %% =====================================================
    % AUTONEX ROBUST ADAPTIVE MULTIMODAL FUSION
    %
    % Supports:
    % - radar/GNN
    % - RGB
    % - thermal IR
    % - missing sensor measurements
    %% =====================================================


    %% =====================================================
    % 1. RADAR TRACK CONFIDENCE
    %% =====================================================

    P = track.StateCovariance;


    positionUncertainty = ...
        sqrt( ...
            max(P(1,1),0) + ...
            max(P(3,3),0));


    radarConfidence = ...
        exp(-positionUncertainty / 3.0);


    radarConfidence = ...
        max(0,min(radarConfidence,1));


    %% =====================================================
    % 2. SENSOR HEALTH
    %% =====================================================

    health = ...
        estimate_sensor_health(environment);


    %% =====================================================
    % 3. CONFIDENCES
    %% =====================================================

    confidences = [
        radarConfidence
        rgbConfidence
        thermalConfidence
    ];


    healthWeights = [
        health.radar
        health.rgb
        health.thermal
    ];


    %% =====================================================
    % 4. AVAILABLE SENSOR MASK
    %% =====================================================

    available = ...
        ~isnan(confidences);


    numSensors = ...
        sum(available);


    %% =====================================================
    % 5. FAILURE FALLBACK
    %% =====================================================

    if numSensors == 0

        fused.radarConfidence = NaN;

        fused.rgbConfidence = NaN;

        fused.thermalConfidence = NaN;


        fused.radarWeight = 0;

        fused.rgbWeight = 0;

        fused.thermalWeight = 0;


        fused.fusedConfidence = 0;

        fused.sensorDisagreement = 1;

        fused.positionUncertainty = ...
            positionUncertainty;

        fused.fusedUncertainty = 10;

        fused.numSensors = 0;

        return;

    end


    %% =====================================================
    % 6. DISABLE WEIGHT FOR MISSING SENSORS
    %% =====================================================

    healthWeights(~available) = 0;


    totalWeight = ...
        sum(healthWeights);


    if totalWeight <= 0

        healthWeights(available) = 1;

        totalWeight = ...
            sum(healthWeights);

    end


    normalizedWeights = ...
        healthWeights / totalWeight;


    %% =====================================================
    % 7. FUSED CONFIDENCE
    %% =====================================================

    confidenceForFusion = ...
        confidences;


    confidenceForFusion(~available) = 0;


    fusedConfidence = ...
        sum( ...
            normalizedWeights .* ...
            confidenceForFusion);


    fusedConfidence = ...
        max(0,min(fusedConfidence,1));


    %% =====================================================
    % 8. SENSOR DISAGREEMENT
    %% =====================================================

    availableConfidences = ...
        confidences(available);


    if numSensors >= 2

        sensorDisagreement = ...
            std(availableConfidences);

    else

        %% Only one sensing modality currently supporting
        % this track -> deliberately increase uncertainty.

        sensorDisagreement = 0.20;

    end


    %% =====================================================
    % 9. MISSING-SENSOR PENALTY
    %% =====================================================

    missingSensorPenalty = ...
        (3 - numSensors) * 0.30;


    %% =====================================================
    % 10. FINAL UNCERTAINTY
    %% =====================================================

    fusedUncertainty = ...
        positionUncertainty + ...
        2.0 * sensorDisagreement + ...
        missingSensorPenalty;


    %% =====================================================
    % OUTPUT
    %% =====================================================

    fused.radarConfidence = ...
        radarConfidence;


    fused.rgbConfidence = ...
        rgbConfidence;


    fused.thermalConfidence = ...
        thermalConfidence;


    fused.radarWeight = ...
        normalizedWeights(1);


    fused.rgbWeight = ...
        normalizedWeights(2);


    fused.thermalWeight = ...
        normalizedWeights(3);


    fused.fusedConfidence = ...
        fusedConfidence;


    fused.sensorDisagreement = ...
        sensorDisagreement;


    fused.positionUncertainty = ...
        positionUncertainty;


    fused.fusedUncertainty = ...
        fusedUncertainty;


    fused.numSensors = ...
        numSensors;

end