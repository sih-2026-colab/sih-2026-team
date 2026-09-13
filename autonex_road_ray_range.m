function range=autonex_road_ray_range(origin,direction,g)
range=80*ones(size(direction,1),1);
for k=1:size(g.segments,1)
    s=g.segments(k,:); a=s(1:2); b=s(3:4); edge=b-a;
    denom=direction(:,1)*edge(2)-direction(:,2)*edge(1);
    offset=a-origin;
    t=(offset(1)*edge(2)-offset(2)*edge(1))./denom;
    u=(offset(1)*direction(:,2)-offset(2)*direction(:,1))./denom;
    t(abs(denom)<1e-12 | t<=0 | u<0 | u>1)=inf;
    range=min(range,t);
end
end
