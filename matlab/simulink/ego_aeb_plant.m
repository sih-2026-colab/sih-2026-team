function y = ego_aeb_plant(u)
%EGO_AEB_PLANT Interpreted MATLAB Fcn for ego_aeb.slx.
% Input u is simulation time. Output y is ego x (m).
    persistent cache
    if isempty(cache) || u < cache.t
        thisDir = fileparts(mfilename('fullpath'));
        addpath(fullfile(thisDir, '..', 'main'));
        addpath(fullfile(thisDir, '..', 'control'));
        addpath(fullfile(thisDir, '..', 'scenarios'));
        cache.result = simulate_aeb(true);
        cache.t = 0;
    end
    cache.t = u;
    logt = [cache.result.log.t];
    logx = [cache.result.log.ego_x];
    y = interp1(logt, logx, u, 'linear', 'extrap');
end
