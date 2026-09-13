function [referenceY, isValid, usedLookahead] = ...
    select_corridor_reference(corridor, ego, desiredLookahead)

    referenceY = ego.y;
    isValid = false;
    usedLookahead = NaN;

    if isempty(corridor) || ...
       ~isfield(corridor,'valid') || ...
       ~isfield(corridor,'centerY') || ...
       ~isfield(corridor,'x')

        return;
    end

    validIndices = ...
        find(corridor.valid & isfinite(corridor.centerY));

    if isempty(validIndices)
        return;
    end

    aheadMask = ...
        corridor.x(validIndices) > ego.x + 2.0;

    validIndices = ...
        validIndices(aheadMask);

    if isempty(validIndices)
        return;
    end

    targetX = ...
        ego.x + desiredLookahead;

    [~,localIndex] = ...
        min(abs(corridor.x(validIndices) - targetX));

    selectedIndex = ...
        validIndices(localIndex);

    referenceY = ...
        corridor.centerY(selectedIndex);

    usedLookahead = ...
        corridor.x(selectedIndex) - ego.x;

    isValid = true;

end