function command = decision_manager(risk)

    % Commands:
    % 0 = CRUISE
    % 1 = BRAKE
    % 2 = REPLAN

    if risk == 3
        command = 1;

    elseif risk == 2
        command = 2;

    else
        command = 0;
    end

end