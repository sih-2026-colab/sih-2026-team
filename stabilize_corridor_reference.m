function [stabilizedY, stab] = stabilize_corridor_reference(rawY, safeSpace, grid, ego, stab, t, lookX)
% Temporal stabilization of the safe-corridor lateral reference.
% Safety-gated: smoothing and rate limiting are only ever applied when the
% previous reference lies inside the SAME contiguous observed-free segment
% that contains the raw corridor reference. Unknown space is never free.
% No lane centers, no hard-coded Y values.
if nargin < 6 || isempty(lookX)
    lookX = ego.x + 10;
end
defaults = struct('initialized',false,'previousReferenceY',NaN, ...
    'lastUpdateTime',-inf,'mode','RESET','rawReferenceY',NaN);
if ~isfield(stab,'initialized'), stab = defaults; end
if ~stab.initialized
    stab.previousReferenceY = rawY;
end
stab.rawReferenceY = rawY;
x = grid.xValues; y = grid.yValues;
mask = safeSpace.safeFreeMask;

[~,col] = min(abs(x-lookX));
if ~any(mask(:,col))
    % No valid safe segment at the lookahead column: invalid so that the
    % existing no-safe-space / emergency behavior remains authoritative.
    stab.initialized = true; stab.lastUpdateTime = t; stab.mode = 'NO_SAFE_SEGMENT';
    stabilizedY = NaN;
    return
end

% Contiguous safe segment containing the RAW corridor reference.
[~,rawRow] = min(abs(y-rawY));
if ~mask(rawRow,col)
    safeRows = find(mask(:,col));
    [~,k] = min(abs(y(safeRows)-rawY)); rawRow = safeRows(k);
end
lo = rawRow; hi = rawRow;
while lo>1 && mask(lo-1,col), lo=lo-1; end
while hi<numel(y) && mask(hi+1,col), hi=hi+1; end
segLow = y(lo); segHigh = y(hi);

prevY = stab.previousReferenceY;
sameSegment = isfinite(prevY) && prevY >= segLow-1e-9 && prevY <= segHigh+1e-9;

if ~sameSegment
    % Safety switch: previous reference is outside the current contiguous
    % safe segment. Never smooth or rate-limit through blocked/unknown space.
    stabilizedY = min(max(rawY,segLow),segHigh);
    stab.mode = 'SAFETY_SWITCH';
else
    prevIn = (t - stab.lastUpdateTime) <= 0.3 + 1e-8;
    if ~isfinite(prevY) || ~prevIn
        stabilizedY = min(max(rawY,segLow),segHigh);
        stab.mode = 'RESET';
    else
        % Continuity: deadband suppresses sub-grid chatter, rate limit caps
        % lateral reference speed, alpha smooths toward the accepted target.
        deadband = 0.17; alpha = 0.4; maxStep = 0.50;
        d = rawY - prevY;
        if abs(d) <= deadband
            target = prevY;
            stab.mode = 'HOLD_DEADBAND';
        else
            target = prevY + sign(d)*(abs(d)-deadband);
            target = prevY + max(min(target-prevY,maxStep),-maxStep);
            stab.mode = 'STABILIZED';
        end
        target = prevY + alpha*(target - prevY);
        stabilizedY = min(max(target,segLow),segHigh); % clamp strictly inside segment
    end
end
stab.initialized = true;
stab.previousReferenceY = stabilizedY;
stab.lastUpdateTime = t;
end
