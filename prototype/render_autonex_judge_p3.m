function render_autonex_judge_p3()
% Real pipeline snapshots, with metrics accumulated over the sampled run.
startup; c=autonex_demo_catalog(); a=autonex_judge_dashboard('on');
cleanup=onCleanup(@()delete(a.figure)); %#ok<NASGU>
a.dt=.05; a.perceptionMode='camera_radar_lidar';
for i=1:numel(c)
    opts=autonex_options(struct('scenario',c(i).id,'perceptionMode',a.perceptionMode, ...
        'explainabilityTelemetry',true,'sensorTelemetry',true)); state=[]; m=[];
    for time=0:opts.dt:3
        [state,out]=autonex_step(state,time,opts); m=autonex_demo_metrics(m,out);
    end
    a.scenario.Value=c(i).id; a.eventTime=out.time;
    a.events={'Recorded snapshot / live transition history not replayed'};
    a=autonex_judge_update(a,state,out); a=autonex_demo_card(a,c(i).id,m,true);
    a.live.Text='RECORDED SIMULATION'; a.tabs.SelectedTab=a.demoTab;
    drawnow; pause(.3); autonex_judge_fit_axes(a.main); autonex_judge_fit_axes(a.top); drawnow;
    for control=[a.next a.demoMode]
        r=getpixelposition(control,true); f=a.figure.Position;
        assert(r(2)>=0 && r(2)+r(4)<=f(4),'Scenario control clipped');
        parent=getpixelposition(control.Parent,true);
        assert(r(2)>=parent(2) && r(2)+r(4)<=parent(2)+parent(4)+1,'Scenario control outside tab');
    end
    exportapp(a.figure,fullfile('results',['judge_p3_' c(i).id '_result.png']));
    a.tabs.SelectedTab=a.pathTab; drawnow; pause(.2);
    exportapp(a.figure,fullfile('results',['judge_p3_' c(i).id '.png']));
    fprintf('P3_SCREENSHOT_PASS %s\n',c(i).id);
end
end
