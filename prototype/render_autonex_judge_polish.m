function render_autonex_judge_polish()
% Re-render recorded real event frames, without advancing or retiming physics.
loaded=load('results/judge_polish_fixtures.mat'); captures=loaded.captures;
a=autonex_judge_dashboard('on'); cleanup=onCleanup(@()delete(a.figure)); %#ok<NASGU>
a.dt=.05; a.perceptionMode='camera_radar_lidar';
for name={'hidden','detected','decision'}
    f=captures.(name{1}); a.events=f.events; a.eventTime=f.out.time;
    a=autonex_judge_update(a,f.state,f.out); a.live.Text='RECORDED SIMULATION';
    a.tabs.SelectedTab=a.pathTab; drawnow; autonex_judge_viewport(a.top); drawnow;
    suffix=['occluded_' name{1}]; if strcmp(name{1},'decision'), suffix='decision'; end
    exportapp(a.figure,fullfile('results',['judge_p2_' suffix '.png']));
    fprintf('POLISH_CAPTURE %s t=%.2f\n',name{1},f.out.time);
end
% The logical desktop is DPI-scaled. Verify physical desktop and laptop sizes.
screen=get(groot,'ScreenSize'); desktop=a.figure.Position;
sizes=[desktop(3:4);min(1100,screen(3)-40) min(650,screen(4)-120)];
for k=1:2
    a.figure.Position=[screen(1)+10 screen(2)+70 sizes(k,:)]; drawnow;
    autonex_judge_viewport(a.top); drawnow;
    assert(strcmp(a.figure.Scrollable,'off'));
    for h=[a.start a.pause a.reset a.rate a.scenario]
        r=getpixelposition(h,true); assert(r(1)>=0 && r(2)>=0 && r(1)+r(3)<=sizes(k,1)+1 && r(2)+r(4)<=sizes(k,2)+1);
    end
    p=getpixelposition(a.top); ratio=diff(a.top.XLim)/diff(a.top.YLim);
    assert(abs(ratio-p(3)/p(4))<1e-8,'Equal-scale plot box does not fill axes');
    exportapp(a.figure,sprintf('results/judge_p2_scaling_%d.png',k));
end
fprintf('POLISH_SCALING_PASS logicalDesktop=%g x %g\n',screen(3),screen(4));
end
