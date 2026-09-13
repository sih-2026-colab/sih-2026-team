function hazard=detect_road_surface_hazards(points,xValues,yValues)
% Gridded depth input in world coordinates. A flat-road prototype baseline.
% Mark depressions below the robust ground level; reject isolated depth noise.
z=nan(numel(yValues),numel(xValues));
dx=mean(diff(xValues)); dy=mean(diff(yValues));
col=round((points(:,1)-xValues(1))/dx)+1;
row=round((points(:,2)-yValues(1))/dy)+1;
valid=all(isfinite(points),2) & col>=1 & col<=numel(xValues) & row>=1 & row<=numel(yValues);
z(sub2ind(size(z),row(valid),col(valid)))=points(valid,3);
ground=median(z(isfinite(z)));
if isempty(ground) || ~isfinite(ground), ground=0; end
depth=ground-z;
candidate=depth>.12;
mask=candidate & conv2(double(candidate),ones(3),'same')>=3;
hazard=struct('mask',mask,'depth',depth,'observed',isfinite(z), ...
    'groundHeight',ground,'thresholdMetres',.12,'hazardCells',nnz(mask));
end
