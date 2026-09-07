function modifier = intent_reflex_modifier(intent)

    %% =====================================================
    % AUTONEX INTENT-AWARE REFLEX MODIFIER
    %
    % Converts cut-in probability into an early
    % preventive speed recommendation.
    %% =====================================================

    if isempty(intent)

        modifier.targetSpeedReduction = 0;
        modifier.riskLevel = 'NONE';

        return;

    end


    probability = intent.probability;


    %% LOW RISK
    if probability < 0.35

        modifier.targetSpeedReduction = 0;
        modifier.riskLevel = 'LOW';


    %% MEDIUM RISK
    elseif probability < 0.55

        modifier.targetSpeedReduction = 0.5;
        modifier.riskLevel = 'MEDIUM';


    %% HIGH RISK
    elseif probability < 0.75

        modifier.targetSpeedReduction = 1.0;
        modifier.riskLevel = 'HIGH';


    %% CRITICAL RISK
    else

        modifier.targetSpeedReduction = 2.0;
        modifier.riskLevel = 'CRITICAL';

    end

end