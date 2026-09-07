function selectedSpeed = select_safe_speed(results)

    selectedSpeed = NaN;

    % Results are already ordered:
    % 75, 74, 73, 72, 70
    %
    % Therefore choose the first safe speed.
    % This gives minimum intervention.

    for i = 1:length(results)

        if results(i).safe

            selectedSpeed = ...
                results(i).speedKmh;

            return;

        end

    end

end