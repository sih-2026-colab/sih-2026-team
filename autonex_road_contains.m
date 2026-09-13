function inside=autonex_road_contains(x,y,g)
inside=false(size(x));
for k=1:size(g.rectangles,1)
    r=g.rectangles(k,:);
    inside=inside | (x>=r(1) & x<=r(2) & y>=r(3) & y<=r(4));
end
end
