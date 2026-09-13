function [detections,cloud]=simulate_autonex_lidar_objects(actors,config)
% Planar ray-cast depth -> pointCloud -> Euclidean clusters. No actor IDs,
% classes or ground-truth centres are used in clustering/measurements.
assert(exist('pointCloud','file')==2 && exist('pcsegdist','file')==2, ...
    'AutoNex:LidarUnavailable','pointCloud and pcsegdist are required');
if nargin<2, config=struct; end
config=autonex_sensor_config(config);
[points,pose]=autonex_planar_returns(actors,config.lidarPose);
R=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
xy=points*R'+pose(1:2);
% Remove max-range no-returns and known synthetic road-edge surfaces.
valid=hypot(points(:,1),points(:,2))<79.9 & ...
    abs(xy(:,2)+1.75)>.08 & abs(xy(:,2)-8.75)>.08;
g=autonex_road_geometry(actors);
if g.junction
    valid=hypot(points(:,1),points(:,2))<79.9;
    for k=1:size(g.segments,1)
        a=g.segments(k,1:2); d=g.segments(k,3:4)-a;
        fraction=max(0,min(1,((xy-a)*d')/sum(d.^2)));
        valid=valid & vecnorm(xy-(a+fraction.*d),2,2)>.08;
    end
end
xy=xy(valid,:);
if config.lidarNoise>0, xy=xy+config.lidarNoise*randn(size(xy)); end
cloud=pointCloud([xy zeros(size(xy,1),1)]);
detections=struct([]);
if cloud.Count<3, return; end
[labels,n]=pcsegdist(cloud,.8);
for k=1:n
    p=cloud.Location(labels==k,1:2);
    if size(p,1)<3, continue; end
    center=(min(p,[],1)+max(p,[],1))/2;
    % Only exposed surfaces are observed; do not claim a precise actor centre.
    row=struct('x',center(1),'y',center(2),'covariance',(9+config.lidarNoise^2)*eye(2), ...
        'confidence',.8,'type','unknown','pointCount',size(p,1));
    if isempty(detections), detections=row; else, detections(end+1)=row; end %#ok<AGROW>
end
end
