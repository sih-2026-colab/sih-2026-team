function values=autonex_simulink_step(t)
% Interpreted simulation adapter. Reset by model InitFcn before every run.
persistent state lastTime cached
if isempty(lastTime) || t<lastTime
    state=[]; lastTime=-inf;
end
if t>lastTime
    [state,out]=autonex_step(state,t,autonex_options());
    cached=[out.x out.y out.speed out.ax out.ay out.trackCount out.emergency out.collision];
    lastTime=t;
end
values=cached;
end
