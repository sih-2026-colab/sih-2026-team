function c=autonex_sensor_config(c)
% Extrinsics: [forward metres, left metres, yaw radians] in ego frame.
if nargin==0, c=struct; end
defaults=struct('cameraPose',[0 0 0],'radarPoses',[0 0 0;0 0 pi;0 0 pi/3;0 0 -pi/3], ...
    'lidarPose',[0 0 0],'yawRate',0,'lidarNoise',0);
for name=fieldnames(defaults)'
    if ~isfield(c,name{1}), c.(name{1})=defaults.(name{1}); end
end
assert(isequal(size(c.radarPoses),[4 3]) && numel(c.cameraPose)==3 && numel(c.lidarPose)==3);
assert(all(isfinite([c.cameraPose(:);c.radarPoses(:);c.lidarPose(:);c.yawRate;c.lidarNoise])));
assert(c.lidarNoise>=0);
end
