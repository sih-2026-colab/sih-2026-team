function app=autonex_judge_update(app,state,out)
% Read-only projection of real output. Does not advance or modify simulation.
for k=1:2, app.scenes{k}=autonex_judge_scene('update',app.scenes{k},state,out); end
d=autonex_judge_explain(out); app.explanation=d;
app.overlay=autonex_judge_overlay('update',app.overlay,d,out);
app.scenes{2}.candidates.Visible='off';
app.risk.Value=splitlines(string(d.risk)); app.pathDetail.Value=splitlines(string(d.detail));
app.scores.Data=d.rows; app.sensorText.Value=d.sensors;
app.chain.Text=d.chain; app.chain.Tooltip=d.chain; app.chain.FontSize=10;
if out.time<app.eventTime, app.events={}; app.eventState=struct; end
if out.time~=app.eventTime
    for key=fieldnames(d.transitions)'
        name=key{1}; value=d.transitions.(name);
        if ~isfield(app.eventState,name) || ~isequal(app.eventState.(name),value)
            app.events{end+1}=sprintf('%05.2f s  %s',out.time,value);
        end
    end
    app.events=app.events(max(1,end-9):end); app.eventState=d.transitions; app.eventTime=out.time;
end
if isempty(app.events), app.eventText.Value={'Awaiting state transitions'};
else, app.eventText.Value=app.events; end
for f=fieldnames(app.values)'
    key=f{1}; value='N/A';
    if isfield(out,key)
        v=out.(key);
        if islogical(v), value=string(v);
        elseif isnumeric(v)
            if isinf(v), value='No finite clearance';
            elseif strcmp(key,'steeringAngle'), value=sprintf('%+.1f°',rad2deg(v));
            elseif ismember(key,{'speed','targetSpeed'}), value=sprintf('%.1f km/h',v);
            elseif strcmp(key,'ax'), value=sprintf('%+.2f m/s²',v);
            elseif ismember(key,{'trackCount','candidateCount'}), value=sprintf('%d',v);
            else, value=sprintf('%.2f m',v); end
        else, value=char(v); end
    end
    app.values.(key).Text=value; app.values.(key).Tooltip=key+": "+string(value);
end
app.time.Text=sprintf('%05.2f s',out.time);
mode='N/A'; if isfield(out,'motionMode'), mode=out.motionMode; end
app.decision.Text=mode; color=app.theme.muted;
switch mode
    case 'CRUISE', color=app.theme.green;
    case 'REPLAN', color=app.theme.amber;
    case {'BRAKE','STOP'}, color=app.theme.red;
end
app.decision.FontColor=color;
for key={'collision','boundaryViolation','emergency'}
    app.values.(key{1}).FontColor=app.theme.text;
    if out.(key{1}), app.values.(key{1}).FontColor=app.theme.red; end
end
app.footer.Text=sprintf('dt %.2f s  /  %s',app.dt,app.perceptionMode);
end
