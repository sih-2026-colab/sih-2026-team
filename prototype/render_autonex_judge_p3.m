function render_autonex_judge_p3(scenarioIDs)
% Real pipeline snapshots, with metrics accumulated over the sampled run.
startup; c=autonex_demo_catalog(); a=autonex_judge_dashboard('on');
if nargin>0, c=c(ismember({c.id},scenarioIDs)); assert(~isempty(c)); end
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
    if strcmp(c(i).id,'unsignalized_intersection')
        exportapp(a.figure,fullfile('results','judge_p3_intersection.png'));
    end
    fprintf('P3_SCREENSHOT_PASS %s\n',c(i).id);
end
screen=get(groot,'ScreenSize'); sizes=[min(1250,screen(3)-30) min(600,screen(4)-120);min(1100,screen(3)-40) min(580,screen(4)-120)];
for k=1:size(sizes,1)
    a.figure.Position=[10 70 sizes(k,:)]; a.tabs.SelectedTab=a.demoTab;
    drawnow; pause(.5); autonex_judge_fit_axes(a.main); autonex_judge_fit_axes(a.top); drawnow;
    for control=[a.next a.demoMode a.start a.pause a.reset a.rate a.scenario]
        r=getpixelposition(control,true); p=getpixelposition(control.Parent,true);
        assert(r(1)>=p(1)-1 && r(2)>=p(2)-1 && r(1)+r(3)<=p(1)+p(3)+1 && r(2)+r(4)<=p(2)+p(4)+1, ...
            'Scaled %s outside parent: control=%s parent=%s',class(control),mat2str(r),mat2str(p));
    end
    exportapp(a.figure,sprintf('results/judge_p3_scaled_%d.png',k));
end
fprintf('P3_SCALED_LAYOUT_PASS logicalDesktop=%g x %g\n',screen(3),screen(4));
end
