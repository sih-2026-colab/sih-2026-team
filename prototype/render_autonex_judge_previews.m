function render_autonex_judge_previews()
% Render actual pipeline snapshots at two window sizes, including resize QA.
app=autonex_judge_dashboard('on'); cleanup=onCleanup(@()delete(app.figure)); %#ok<NASGU>
app.dt=.05; app.perceptionMode='camera_radar_lidar';
for name={'occluded_pedestrian','unsignalized_intersection'}
    opts=autonex_options(struct('scenario',name{1},'perceptionMode',app.perceptionMode));
    state=[];
    for t=0:opts.dt:3, [state,out]=autonex_step(state,t,opts); end
    app.scenario.Value=name{1}; app.live.Text='●  SIMULATION SNAPSHOT';
    for j=1:2, app.scenes{j}.followX=NaN; end
    app=autonex_judge_update(app,state,out);
    app.figure.Position=[5 5 1280 720]; app.figure.Scrollable='off'; drawnow;
    exportapp(app.figure,fullfile('results',['judge_' name{1} '.png']));
    if strcmp(name{1},'unsignalized_intersection')
        app.figure.Position=[5 5 1100 650]; drawnow;
        exportapp(app.figure,'results/judge_laptop_layout.png');
    end
end
fprintf('JUDGE_PREVIEWS_PASS\n');
end
