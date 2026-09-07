function envelope = ...
    calculate_contextual_safety_envelope( ...
        fusedUncertainty, ...
        fusedConfidence, ...
        relativeClosingSpeed, ...
        lateralSpeed, ...
        movingTowardEgoPath)

    %% =====================================================
    % AUTONEX CONTEXT-AWARE SAFETY ENVELOPE
    %
    % Important improvement:
    %
    % An adjacent-lane vehicle moving straight should NOT
    % receive the same lateral bubble as a vehicle actively
    % cutting toward the ego path.
    %% =====================================================


    %% Clamp inputs

    fusedConfidence = ...
        max(0,min(fusedConfidence,1));


    fusedUncertainty = ...
        max(fusedUncertainty,0);


    closingSpeed = ...
        max(relativeClosingSpeed,0);


    %% =====================================================
    % BASE VEHICLE SAFETY REGION
    %% =====================================================

    baseLongitudinal = 6.0;

    baseLateral = 2.2;


    %% =====================================================
    % LONGITUDINAL EXPANSION
    %% =====================================================

    uncertaintyLongitudinal = ...
        0.80 * fusedUncertainty;


    confidenceLongitudinal = ...
        1.50 * ...
        (1 - fusedConfidence);


    closingSpeedLongitudinal = ...
        0.15 * closingSpeed;


    longitudinal = ...
        baseLongitudinal + ...
        uncertaintyLongitudinal + ...
        confidenceLongitudinal + ...
        closingSpeedLongitudinal;


    %% =====================================================
    % LATERAL EXPANSION
    %% =====================================================

    uncertaintyLateral = ...
        0.25 * fusedUncertainty;


    confidenceLateral = ...
        0.50 * ...
        (1 - fusedConfidence);


    lateral = ...
        baseLateral + ...
        uncertaintyLateral + ...
        confidenceLateral;


    %% =====================================================
    % MANEUVER-AWARE EXPANSION
    %
    % Only significantly expand sideways when the tracked
    % object is actually moving toward the ego trajectory.
    %% =====================================================

    if movingTowardEgoPath

        maneuverExpansion = ...
            min( ...
                0.90, ...
                0.35 * abs(lateralSpeed));


        lateral = ...
            lateral + ...
            maneuverExpansion;

    end


    %% =====================================================
    % PRACTICAL LIMITS
    %
    % Avoid allowing uncertainty alone to engulf an entire
    % neighboring lane.
    %% =====================================================

    longitudinal = ...
        min(max(longitudinal,6.0),11.0);


    lateral = ...
        min(max(lateral,2.2),3.25);


    %% =====================================================
    % OUTPUT
    %% =====================================================

    envelope.longitudinal = ...
        longitudinal;


    envelope.lateral = ...
        lateral;


    envelope.confidence = ...
        fusedConfidence;


    envelope.uncertainty = ...
        fusedUncertainty;


    envelope.relativeSpeed = ...
        closingSpeed;


    envelope.lateralSpeed = ...
        lateralSpeed;


    envelope.movingTowardEgoPath = ...
        movingTowardEgoPath;

end