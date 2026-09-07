function [futureY, uncertainty] = predict_pedestrian(pedY, pedVy, horizon)

    % Predict pedestrian future lateral position
    futureY = pedY + pedVy * horizon;

    % Simple uncertainty model
    % More prediction time = more uncertainty
    baseUncertainty = 0.3;
    uncertaintyRate = 0.4;

    uncertainty = baseUncertainty + uncertaintyRate * horizon;

end