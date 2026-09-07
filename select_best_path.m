function [bestPath, bestIndex] = select_best_path(paths, scores)

    [bestScore, bestIndex] = min(scores);

    if isinf(bestScore)
        bestPath = NaN;
    else
        bestPath = paths(bestIndex);
    end

end