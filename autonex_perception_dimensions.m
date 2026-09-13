function [L,W,source]=autonex_perception_dimensions(actor)
% Synthetic priors, not universal real dimensions. Frozen planner unchanged.
if isfield(actor,'length') && isfield(actor,'width') && ...
        isscalar(actor.length) && isscalar(actor.width) && ...
        all(isfinite([actor.length actor.width])) && actor.length>0 && actor.width>0
    L=actor.length; W=actor.width; source='explicit'; return;
end
source='class prior';
switch autonex_object_class(actor.type)
    case 'car', L=4.5; W=1.9;
    case 'bus', L=10; W=2.5;
    case 'two-wheeler', L=2; W=.8;
    case 'auto-rickshaw', L=2.6; W=1.4;
    case 'pedestrian', L=.6; W=.6;
    case 'animal', L=2; W=.8;
    case 'pushcart', L=1.8; W=1;
    otherwise, L=1; W=1; source='unknown prior';
end
end
