function v=autonex_actor_visibility(origin,actors,target,sensor,egoIndex)
% Synthetic 2-D opaque-box LOS. Indices exclude self/target, never identify
% special scenario actors. Optical sensors sample extent; direct-path radar
% uses the centre ray and opaque vehicle/obstacle blockers (no multipath).
if nargin<5, egoIndex=1; end
a=actors(target); [L,W]=autonex_perception_dimensions(a);
h=a.heading; R=[cos(h) -sin(h);sin(h) cos(h)];
if strcmpi(sensor,'radar'), local=[0 0];
else, [x,y]=meshgrid([-1 0 1]); local=[x(:)*L/2 y(:)*W/2]; end
points=local*R'+[a.x a.y]; blocked=false(size(points,1),1);
for k=1:numel(actors)
    if k==target || k==egoIndex || strcmpi(actors(k).type,'pothole'), continue; end
    if strcmpi(sensor,'radar') && ismember(autonex_object_class(actors(k).type),{'pedestrian','animal'}), continue; end
    [l,w]=autonex_perception_dimensions(actors(k)); h=actors(k).heading;
    B=[cos(h) -sin(h);sin(h) cos(h)];
    o=(origin-[actors(k).x actors(k).y])*B;
    for j=1:size(points,1)
        d=(points(j,:)-origin)*B; enter=0; leave=1;
        for axis=1:2
            half=[l w]/2;
            if abs(d(axis))<1e-12
                if abs(o(axis))>half(axis), enter=inf; end
            else
                u=([-half(axis) half(axis)]-o(axis))/d(axis);
                enter=max(enter,min(u)); leave=min(leave,max(u));
            end
        end
        blocked(j)=blocked(j) || (enter<=leave && leave>1e-7 && enter<1-1e-7);
    end
end
v=struct('visible',any(~blocked),'visibleFraction',mean(~blocked), ...
    'fullyOccluded',all(blocked),'blockedRays',sum(blocked),'rayCount',numel(blocked));
end
