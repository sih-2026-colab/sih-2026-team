function [ttc, threat] = aeb_logic(ego, objects, hit_dist_m, horizon_s, dt)
%AEB_LOGIC Constant-velocity time of first close approach.
%   ego: struct with x,y,vx,vy
%   objects: struct array with x,y,vx,vy,label
    if nargin < 3, hit_dist_m = 2.4; end
    if nargin < 4, horizon_s = 3.0; end
    if nargin < 5, dt = 0.05; end

    ttc = NaN;
    threat = '';
    steps = max(1, round(horizon_s / dt));

    for i = 1:numel(objects)
        obj = objects(i);
        for k = 1:steps
            t = k * dt;
            ex = ego.x + ego.vx * t;
            ey = ego.y + ego.vy * t;
            ox = obj.x + obj.vx * t;
            oy = obj.y + obj.vy * t;
            d = hypot(ex - ox, ey - oy);
            if d <= hit_dist_m
                if isnan(ttc) || t < ttc
                    ttc = t;
                    if isfield(obj, 'label')
                        threat = obj.label;
                    else
                        threat = num2str(obj.id);
                    end
                end
                break
            end
        end
    end
end
