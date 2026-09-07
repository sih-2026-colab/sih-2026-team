function fused = ...
    fuse_sensor_confidence(track, thermalConfidence)

    %% =====================================================
    % AUTONEX RADAR + THERMAL CONFIDENCE FUSION
    %% =====================================================


    %% Get GNN/Kalman covariance

    P = track.StateCovariance;


    %% -----------------------------------------------------
    % POSITION UNCERTAINTY
    %% -----------------------------------------------------

    positionUncertainty = ...
        sqrt( ...
            max(P(1,1),0) + ...
            max(P(3,3),0));


    %% -----------------------------------------------------
    % CONVERT TRACK UNCERTAINTY INTO CONFIDENCE
    %% -----------------------------------------------------

    radarConfidence = ...
        exp(-positionUncertainty / 3.0);


    radarConfidence = ...
        max(0, min(radarConfidence,1));


    %% -----------------------------------------------------
    % SENSOR FUSION
    %
    % Radar has more weight because it currently provides
    % our motion/position estimate.
    %% -----------------------------------------------------

    fusedConfidence = ...
        0.60 * radarConfidence + ...
        0.40 * thermalConfidence;


    fusedConfidence = ...
        max(0, min(fusedConfidence,1));


    %% -----------------------------------------------------
    % DISAGREEMENT / UNCERTAINTY
    %% -----------------------------------------------------

    confidenceDifference = ...
        abs(radarConfidence - thermalConfidence);


    fusedUncertainty = ...
        positionUncertainty + ...
        confidenceDifference;


    %% OUTPUT

    fused.radarConfidence = ...
        radarConfidence;

    fused.thermalConfidence = ...
        thermalConfidence;

    fused.fusedConfidence = ...
        fusedConfidence;

    fused.positionUncertainty = ...
        positionUncertainty;

    fused.fusedUncertainty = ...
        fusedUncertainty;

end