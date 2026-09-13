function app=autonex_judge_update(app,state,out)
% Read-only projection of real output. Does not advance or modify simulation.
for k=1:2, app.scenes{k}=autonex_judge_scene('update',app.scenes{k},state,out); end
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
