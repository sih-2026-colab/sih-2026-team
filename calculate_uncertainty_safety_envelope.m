function envelope = ...
    calculate_uncertainty_safety_envelope( ...
        fusedUncertainty, ...
        fusedConfidence, ...
        relativeSpeed)

    %% =====================================================
    % AUTONEX UNCERTAINTY-AWARE SAFETY ENVELOPE
    %
    % Safety envelope depends on:
    %
    % 1. sensor / tracking uncertainty
    % 2. fused confidence
    % 3. relative closing speed
    %% =====================================================


    %% =====================================================
    % BASE SAFETY ENVELOPE
    %% =====================================================

    baseLongitudinal = 6.0;

    baseLateral = 2.2;


    %% =====================================================
    % UNCERTAINTY EXPANSION
    %% =====================================================

    uncertaintyLongitudinal = ...
        2.0 * fusedUncertainty;


    uncertaintyLateral = ...
        0.8 * fusedUncertainty;


    %% =====================================================
    % LOW-CONFIDENCE EXPANSION
    %% =====================================================

    confidencePenalty = ...
        1 - fusedConfidence;


    confidenceLongitudinal = ...
        4.0 * confidencePenalty;


    confidenceLateral = ...
        1.5 * confidencePenalty;


    %% =====================================================
    % CLOSING SPEED EXPANSION
    %
    % relativeSpeed > 0 means approaching/closing.
    %% =====================================================

    closingSpeed = ...
        max(relativeSpeed,0);


    speedLongitudinal = ...
        0.35 * closingSpeed;


    %% =====================================================
    % FINAL SAFETY ENVELOPE
    %% =====================================================

    longitudinal = ...
        baseLongitudinal + ...
        uncertaintyLongitudinal + ...
        confidenceLongitudinal + ...
        speedLongitudinal;


    lateral = ...
        baseLateral + ...
        uncertaintyLateral + ...
        confidenceLateral;


    %% Limit extreme prototype values

    longitudinal = ...
        min(max(longitudinal,6),18);


    lateral = ...
        min(max(lateral,2.2),5);


    %% =====================================================
    % OUTPUT
    %% =====================================================

    envelope.longitudinal = ...
        longitudinal;

    envelope.lateral = ...
        lateral;

    envelope.uncertainty = ...
        fusedUncertainty;

    envelope.confidence = ...
        fusedConfidence;

    envelope.relativeSpeed = ...
        relativeSpeed;

end