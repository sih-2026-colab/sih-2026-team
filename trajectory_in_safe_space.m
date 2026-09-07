function safe = trajectory_in_safe_space(trajectory,drivable,safeSpace,ego)
% Densify segments so a trajectory cannot jump over a blocked grid cell.
x=[ego.x trajectory.x]; y=[ego.y trajectory.y]; safe=true;
step=min(safeSpace.xResolution,safeSpace.yResolution)/3;
for k=2:numel(x)
    n=max(2,ceil(hypot(x(k)-x(k-1),y(k)-y(k-1))/step)+1);
    qx=linspace(x(k-1),x(k),n); qy=linspace(y(k-1),y(k),n);
    col=round((qx-drivable.xValues(1))/safeSpace.xResolution)+1;
    row=round((qy-drivable.yValues(1))/safeSpace.yResolution)+1;
    inside=col>=1 & col<=numel(drivable.xValues) & row>=1 & row<=numel(drivable.yValues);
    if ~all(inside), safe=false; return; end
    if ~all(safeSpace.safeFreeMask(sub2ind(size(safeSpace.safeFreeMask),row,col)))
        safe=false; return;
    end
end
end
