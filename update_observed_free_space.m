function [drivable,memory] = update_observed_free_space(drivable,memory,tracks,t)
% Retain recently observed road behind moving objects. Never invent unseen free cells.
% Confirmed moving objects remain in the independent time-dependent track checks.
prior=false(size(drivable.freeMask)); age=inf(size(prior));
if ~isempty(memory)
    stamp=interp2(memory.x,memory.y,memory.stamp,drivable.X,drivable.Y,'nearest',-inf);
    age=t-stamp; prior=age<=10;
end
dynamic=false(size(prior));
for k=1:numel(tracks)
    s=tracks(k).State;
    if hypot(s(2),s(4))>.5
        dynamic=dynamic | (abs(drivable.X-s(1))<=3 & abs(drivable.Y-s(3))<=1.5);
    end
end
staticOccupied=drivable.occupiedMask & ~dynamic;
observed=drivable.freeMask;
drivable.freeMask=(observed | prior) & ~staticOccupied;
drivable.occupiedMask=staticOccupied;
drivable.unknownMask=~drivable.freeMask & ~staticOccupied;
drivable.classification=-ones(size(prior));
drivable.classification(drivable.freeMask)=0; drivable.classification(staticOccupied)=1;
stamp=t-age; stamp(observed)=t; stamp(staticOccupied)=-inf;
memory=struct('x',drivable.xValues,'y',drivable.yValues,'stamp',stamp);
end
