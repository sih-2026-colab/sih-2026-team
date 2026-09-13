function points=simulate_road_surface_cloud(actors,drivable)
% Synthetic ground-depth samples. Potholes are negative geometry, not radar boxes.
X=drivable.X; Y=drivable.Y;
% Deterministic bounded depth noise keeps other sensor RNG streams unchanged.
Z=.012*sin(3.1*X+2.3*Y);
for k=2:numel(actors)
    if strcmpi(actors(k).type,'pothole')
        [L,W]=autonex_actor_size(actors(k));
        inside=abs(X-actors(k).x)<=L/2 & abs(Y-actors(k).y)<=W/2;
        Z(inside)=Z(inside)-.25;
    end
end
Z(~drivable.freeMask)=NaN; % Do not invent ground observations behind occlusions.
points=[X(:) Y(:) Z(:)];
end
