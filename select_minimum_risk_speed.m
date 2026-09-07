function [selectedSpeed, bestIndex] = ...
    select_minimum_risk_speed(results)

    selectedSpeed = NaN;
    bestIndex = NaN;

    bestScore = inf;

    for i = 1:length(results)

        if results(i).safe

            if results(i).score < bestScore

                bestScore = results(i).score;

                selectedSpeed = ...
                    results(i).speedKmh;

                bestIndex = i;

            end

        end

    end

end