function corridor = extract_safe_corridor(drivable, safeSpace, ego)
% Find a forward-connected path through observed, footprint-safe grid cells.
% No lane centers are supplied. Unknown space and disconnected islands are excluded.
x = drivable.xValues; y = drivable.yValues;
mask = safeSpace.safeFreeMask;
corridor = struct('valid',false,'x',[],'referenceY',[], ...
    'lowerY',[],'upperY',[],'length',0,'reason','EGO_NOT_IN_SAFE_SPACE');
[~, startX] = min(abs(x-ego.x)); [~, startY] = min(abs(y-ego.y));
if ~mask(startY,startX), return; end
cost = inf(size(mask)); parent = zeros(size(mask));
cost(startY,startX) = 0; lastX = startX;
for col = startX+1:numel(x)
    for row = find(mask(:,col))'
        % Four-connected forward moves: a diagonal must not cut a blocked corner.
        prevRows = max(1,row-1):min(numel(y),row+1);
        allowed = mask(prevRows,col-1) & mask(row,col-1) & mask(prevRows,col);
        transition = cost(prevRows,col-1) + 2*abs(y(prevRows)'-y(row));
        transition(~allowed) = inf;
        [best,k] = min(transition);
        if isfinite(best)
            cost(row,col) = best + 0.01*(y(row)-ego.y)^2;
            parent(row,col) = prevRows(k);
        end
    end
    if ~any(isfinite(cost(:,col))), break; end
    lastX = col;
end
if lastX == startX, corridor.reason = 'NO_FORWARD_CONNECTION'; return; end
[~,row] = min(cost(:,lastX)); cols = startX:lastX;
rows = zeros(size(cols)); lower = rows; upper = rows;
for col = lastX:-1:startX
    k = col-startX+1; rows(k) = row;
    lo = row; hi = row;
    while lo>1 && mask(lo-1,col), lo=lo-1; end
    while hi<numel(y) && mask(hi+1,col), hi=hi+1; end
    lower(k)=y(lo); upper(k)=y(hi);
    if col>startX, row=parent(row,col); end
end
corridor.valid = true; corridor.x = x(cols); corridor.referenceY = y(rows);
corridor.lowerY = lower; corridor.upperY = upper;
corridor.length = x(lastX)-x(startX); corridor.reason = 'CONNECTED';
end
