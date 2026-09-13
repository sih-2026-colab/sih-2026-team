function X=cutin_model_features(dx,dy,vy,relativeVx,sigma)
X=[abs(dy(:)) dy(:).*vy(:) abs(vy(:)) ...
    max(abs(dy(:))-1.9,0)./(abs(vy(:))+.2) dx(:) relativeVx(:) sigma(:)];
end
