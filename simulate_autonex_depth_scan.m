function [points,pose] = simulate_autonex_depth_scan(actors)
% Synthetic 360-degree planar depth scan at vehicle height.
% Ray/box first returns preserve occlusion. This is simulated sensing, not real LiDAR.
ego=actors(1); origin=[ego.x ego.y];
angle=(0:.1:359.9)'*pi/180; direction=[cos(angle) sin(angle)];
range=80*ones(size(angle));
geometry=autonex_road_geometry(actors);
% Road edges are physical scene geometry; lane paint is not sensed or used.
if geometry.junction
    range=autonex_road_ray_range(origin,direction,geometry);
else
for edge=[-1.75 8.75]
    hit=(edge-origin(2))./direction(:,2);
    hit(hit<=0)=inf; range=min(range,hit);
end
end
for k=2:numel(actors)
    if strcmpi(actors(k).type,'pothole'), continue; end
    [L,W]=autonex_actor_size(actors(k));
    low=[actors(k).x-L/2 actors(k).y-W/2];
    high=[actors(k).x+L/2 actors(k).y+W/2];
    a=(low-origin)./direction; b=(high-origin)./direction;
    near=max(min(a,b),[],2); far=min(max(a,b),[],2);
    hit=near; hit(far<max(near,0) | near<=0)=inf;
    range=min(range,hit);
end
points=[direction.*range zeros(size(angle))];
pose=[origin .60 1 0 0 0];
end
