function [points,pose]=autonex_planar_returns(actors,mount)
% First-return ray/box sensing, extended for new perception ONLY.
ego=actors(1); E=[cos(ego.heading) -sin(ego.heading);sin(ego.heading) cos(ego.heading)];
origin=[ego.x ego.y]+(E*mount(1:2)')'; yaw=ego.heading+mount(3);
angle=(0:.1:359.9)'*pi/180; local=[cos(angle) sin(angle)];
R=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)]; direction=local*R';
range=80*ones(size(angle));
for edge=[-1.75 8.75]
    hit=(edge-origin(2))./direction(:,2); hit(hit<=0)=inf; range=min(range,hit);
end
for k=2:numel(actors)
    if strcmpi(actors(k).type,'pothole'), continue; end
    [L,W]=autonex_perception_dimensions(actors(k)); h=actors(k).heading;
    B=[cos(h) -sin(h);sin(h) cos(h)];
    o=(origin-[actors(k).x actors(k).y])*B; d=direction*B;
    a=([-L/2 -W/2]-o)./d; b=([L/2 W/2]-o)./d;
    near=max(min(a,b),[],2); far=min(max(a,b),[],2);
    hit=near; hit(far<max(near,0) | near<=0)=inf; range=min(range,hit);
end
points=local.*range; pose=[origin yaw];
end
