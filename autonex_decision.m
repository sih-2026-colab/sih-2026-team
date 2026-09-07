function command = autonex_decision(risk, safe)

    % 0 = CRUISE
    % 1 = BRAKE
    % 2 = REPLAN

    % Highest priority:
    % No safe path exists
    if ~any(safe)

        command = 1;       % BRAKE
        return;

    end

    % Immediate collision risk
    if risk == 3

        command = 1;

    % Something dangerous but alternate path exists
    elseif risk == 2

        command = 2;

    else

        command = 0;

    end

end