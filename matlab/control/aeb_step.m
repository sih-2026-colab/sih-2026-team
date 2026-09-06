function [ax, mode] = aeb_step(ego, objects, mode, ttc_brake_s, a_brake, hit_dist_m)
%AEB_STEP One control step: CRUISE or BRAKE.
    if nargin < 3 || isempty(mode), mode = 'CRUISE'; end
    if nargin < 4, ttc_brake_s = 2.0; end
    if nargin < 5, a_brake = 6.0; end
    if nargin < 6, hit_dist_m = 2.4; end

    [ttc, ~] = aeb_logic(ego, objects, hit_dist_m);
    if ~isnan(ttc) && ttc < ttc_brake_s
        mode = 'BRAKE';
    end
    if strcmp(mode, 'BRAKE')
        ax = -a_brake;
    else
        ax = 0.0;
    end
end
