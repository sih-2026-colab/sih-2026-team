function api=run_autonex_judge_prototype(scenario,options)
% Timer changes display pacing only. Each callback advances exactly opts.dt.
if nargin<1, scenario='occluded_pedestrian'; end
if nargin<2, options=struct; end
visible='on'; if isfield(options,'uiVisible'), visible=options.uiVisible; options=rmfield(options,'uiVisible'); end
options.scenario=scenario;
if ~isfield(options,'perceptionMode'), options.perceptionMode='camera_radar_lidar'; end
if ~isfield(options,'duration'), options.duration=10; end
opts=autonex_options(options); state=[]; out=[]; nextTime=0; running=false;
app=autonex_judge_dashboard(visible); app.scenario.Value=scenario;
app.dt=opts.dt; app.perceptionMode=opts.perceptionMode;
timerObject=timer('ExecutionMode','fixedSpacing','Period',opts.dt,'BusyMode','drop', ...
    'TimerFcn',@tick,'ErrorFcn',@onError);
app.start.ButtonPushedFcn=@(~,~)startRun(); app.pause.ButtonPushedFcn=@(~,~)pauseRun();
app.reset.ButtonPushedFcn=@(~,~)resetRun(app.scenario.Value);
app.scenario.ValueChangedFcn=@(~,~)resetRun(app.scenario.Value);
app.rate.ValueChangedFcn=@(~,~)setRate(); app.figure.CloseRequestFcn=@(~,~)closeRun();
api=struct('figure',app.figure,'start',@startRun,'pause',@pauseRun,'reset',@resetRun, ...
    'step',@stepOnce,'snapshot',@snapshot,'close',@closeRun);
app.figure.UserData=api;
    function startRun()
        if nextTime>opts.duration, resetRun(app.scenario.Value); end
        if ~running, running=true; app.live.Text='●  LIVE SIMULATION'; app.pause.Text='PAUSE'; start(timerObject); end
    end
    function pauseRun()
        if running
            running=false; stop(timerObject); app.live.Text='●  PAUSED'; app.pause.Text='RESUME';
        else, startRun(); end
    end
    function setRate()
        wasRunning=running; if running, stop(timerObject); end
        scale=1; if strcmp(app.rate.Value,'0.5x'), scale=.5; end
        timerObject.Period=opts.dt/scale;
        if wasRunning, start(timerObject); end
    end
    function resetRun(name)
        running=false; stop(timerObject); state=[]; out=[]; nextTime=0;
        opts.scenario=name; app.scenario.Value=name;
        for j=1:2, app.scenes{j}.followX=NaN; end
        stepOnce(); app.live.Text='●  READY'; app.pause.Text='PAUSE';
    end
    function stepOnce()
        if nextTime>opts.duration, return; end
        [state,out]=autonex_step(state,nextTime,opts);
        app=autonex_judge_update(app,state,out); nextTime=nextTime+opts.dt;
        drawnow limitrate;
    end
    function tick(~,~)
        if ~running, return; end
        try
            stepOnce();
            if nextTime>opts.duration
                running=false; stop(timerObject); app.live.Text='●  COMPLETE';
            end
        catch err
            running=false; stop(timerObject); app.live.Text='●  ERROR';
            app.footer.Text=err.message; app.figure.UserData.lastError=getReport(err);
            warning('AutoNex:Dashboard','%s',getReport(err));
        end
    end
    function onError(~,~)
        running=false; app.live.Text='●  ERROR';
    end
    function s=snapshot()
        s=struct('state',state,'out',out,'app',app,'running',running,'nextTime',nextTime);
    end
    function closeRun()
        if isvalid(timerObject), stop(timerObject); delete(timerObject); end
        if isvalid(app.figure), delete(app.figure); end
    end
end
