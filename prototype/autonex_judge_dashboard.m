function app=autonex_judge_dashboard(visible)
if nargin==0, visible='on'; end
t=autonex_judge_theme();
f=uifigure('Name','AUTONEX — INDIADRIVE AI','Color',t.background, ...
    'Position',[20 20 1200 675],'Visible',visible,'Scrollable','off');
f.Scrollable='off';
g=uigridlayout(f,[4 2]); g.RowHeight={76,'1.2x','1x',52}; g.ColumnWidth={'1x',380};
g.Padding=[12 10 12 10]; g.RowSpacing=8; g.ColumnSpacing=10; g.BackgroundColor=t.background;
header=uigridlayout(g,[2 2]); header.Layout.Row=1; header.Layout.Column=[1 2];
header.ColumnWidth={'1x',260}; header.RowHeight={40,26}; header.Padding=[0 0 0 0]; header.BackgroundColor=t.background;
uilabel(header,'Text','AUTONEX  /  INDIADRIVE AI','FontName',t.font,'FontSize',28,'FontWeight','bold','FontColor',t.text);
app.live=uilabel(header,'Text','●  READY','HorizontalAlignment','right','FontName',t.font,'FontSize',16,'FontColor',t.cyan);
app.chain=uilabel(header,'Text','Uncertainty-Aware Autonomous Driving for Unstructured Indian Roads', ...
    'FontName',t.font,'FontSize',14,'FontColor',t.muted);
app.time=uilabel(header,'Text','00.00 s','HorizontalAlignment','right','FontName',t.font,'FontSize',20,'FontColor',t.text);
app.main=makeView(g,'01  /  LIVE DRIVING SCENE',2,t);
app.top=makeView(g,'02  /  WORLD MODEL    •    confirmed tracks + candidate paths',3,t);
app.top.Tag='WorldModel';
right=uigridlayout(g,[2 1]); right.Layout.Row=[2 3]; right.Layout.Column=2;
right.RowHeight={50,'1x'}; right.Padding=[0 0 0 0]; right.RowSpacing=6; right.BackgroundColor=t.background;
app.decision=uilabel(right,'Text','READY','FontName',t.font,'FontWeight','bold', ...
    'FontSize',30,'FontColor',t.cyan,'BackgroundColor',t.panel,'HorizontalAlignment','center');
app.tabs=uitabgroup(right); app.tabs.Layout.Row=2;
liveTab=uitab(app.tabs,'Title','Live','BackgroundColor',t.panel);
liveGrid=uigridlayout(liveTab,[3 1]); liveGrid.Padding=[0 0 0 0]; liveGrid.BackgroundColor=t.panel;
app.values=struct;
app.values=card(liveGrid,'SYSTEM STATE',{'Speed','Target speed','Acceleration','Steering','Target Y'}, ...
    {'speed','targetSpeed','ax','steeringAngle','targetY'},app.values,t);
app.values=card(liveGrid,'DECISION',{'Planner','Maneuver','Guardian','Command','Emergency'}, ...
    {'selectionMode','selectedName','guardianMode','longitudinalCommand','emergency'},app.values,t);
app.values=card(liveGrid,'PERCEPTION',{'Confirmed tracks','Candidates','Minimum clearance','Collision','Boundary violation'}, ...
    {'trackCount','candidateCount','minClearance','collision','boundaryViolation'},app.values,t);
app.pathTab=uitab(app.tabs,'Title','Paths / AI decision','BackgroundColor',t.panel);
pg=uigridlayout(app.pathTab,[3 1]); pg.RowHeight={80,80,'1x'}; pg.Padding=[5 5 5 5]; pg.RowSpacing=5; pg.BackgroundColor=t.panel;
app.risk=uitextarea(pg,'Editable','off','BackgroundColor',t.panel,'FontColor',t.text,'FontSize',12);
app.scores=uitable(pg,'Data',cell(0,5),'ColumnName',{'Path / km/h','Status','Score','Conflicts','Reason'}, ...
    'ColumnWidth',{155,82,67,85,200},'RowName',{},'BackgroundColor',t.panel,'ForegroundColor',t.text,'FontSize',12);
app.pathDetail=uitextarea(pg,'Editable','off','BackgroundColor',t.panel,'FontColor',t.green,'FontSize',12); app.pathDetail.Layout.Row=1; app.risk.Layout.Row=2; app.scores.Layout.Row=3;
app.sensorTab=uitab(app.tabs,'Title','Sensors + log','BackgroundColor',t.panel);
sg=uigridlayout(app.sensorTab,[3 1]); sg.RowHeight={'1x',30,'1x'}; sg.Padding=[5 5 5 5]; sg.BackgroundColor=t.panel;
app.sensorText=uitextarea(sg,'Editable','off','BackgroundColor',t.panel,'FontColor',t.text,'FontSize',12);
uilabel(sg,'Text','LIVE TRANSITIONS / latest 10','FontColor',t.cyan,'FontWeight','bold');
app.eventText=uitextarea(sg,'Editable','off','BackgroundColor',t.panel,'FontColor',t.text,'FontSize',12);
app.events={}; app.eventState=struct; app.eventTime=-inf; app.tabs.SelectedTab=app.pathTab;
app.demoTab=uitab(app.tabs,'Title','Scenario','BackgroundColor',t.panel);
dg=uigridlayout(app.demoTab,[4 1]); dg.RowHeight={'1x','1x',30,30}; dg.BackgroundColor=t.panel; dg.Padding=[5 5 5 5];
app.demoIntro=uitextarea(dg,'Editable','off','BackgroundColor',t.panel,'FontColor',t.cyan,'FontSize',12);
app.demoResult=uitextarea(dg,'Editable','off','BackgroundColor',t.panel,'FontColor',t.text,'FontSize',12);
app.demoMode=uicheckbox(dg,'Text','DEMO MODE / show result at finish','Value',true,'FontColor',t.text);
app.next=uibutton(dg,'Text','NEXT SCENARIO','BackgroundColor',t.border,'FontColor',t.text);
bar=uigridlayout(g,[1 7]); bar.Layout.Row=4; bar.Layout.Column=[1 2];
bar.ColumnWidth={48,'1x',80,80,80,65,125}; bar.Padding=[8 6 8 6]; bar.BackgroundColor=t.panel;
uilabel(bar,'Text','SCENE','FontColor',t.muted,'FontWeight','bold');
catalog=autonex_demo_catalog();
app.scenario=uidropdown(bar,'Items',{catalog.title},'ItemsData',{catalog.id},'BackgroundColor',t.background,'FontColor',t.text);
app.start=uibutton(bar,'Text','START','BackgroundColor',t.cyan,'FontWeight','bold');
app.pause=uibutton(bar,'Text','PAUSE','BackgroundColor',t.border,'FontColor',t.text);
app.reset=uibutton(bar,'Text','RESET','BackgroundColor',t.border,'FontColor',t.text);
app.rate=uidropdown(bar,'Items',{'1x','0.5x'},'BackgroundColor',t.background,'FontColor',t.text);
app.footer=uilabel(bar,'Text','REAL PIPELINE  /  awaiting start','FontColor',t.muted,'HorizontalAlignment','right');
app.figure=f; app.theme=t;
app.scenes={autonex_judge_scene('create',app.main,t),autonex_judge_scene('create',app.top,t)};
view(app.main,[-20 55]); view(app.top,2);

app.overlay=autonex_judge_overlay('create',app.top,t);
screen=get(groot,'ScreenSize');
f.Position=[screen(1)+10 screen(2)+70 min(1280,screen(3)-30) min(800,screen(4)-120)];
end
function ax=makeView(parent,titleText,row,t)
p=uipanel(parent,'Title',titleText,'FontName',t.font,'FontSize',13,'FontWeight','bold', ...
    'ForegroundColor',t.muted,'BackgroundColor',t.panel,'BorderColor',t.border);
p.Layout.Row=row; p.Layout.Column=1;

ax=uiaxes(p,'Color',t.background,'XColor',t.muted,'YColor',t.muted,'ZColor',t.muted, ...
    'FontName',t.font,'FontSize',11,'Box','off');
ax.Toolbar.Visible='off'; ax.Interactions=[]; hold(ax,'on'); axis(ax,'equal');
xlabel(ax,'X / m'); ylabel(ax,'Y / m'); ax.ZTick=[];
p.AutoResizeChildren='off'; p.SizeChangedFcn=@(~,~)autonex_judge_fit_axes(ax);
autonex_judge_fit_axes(ax);
end
function values=card(parent,titleText,labels,fields,values,t)
p=uipanel(parent,'Title',titleText,'ForegroundColor',t.cyan,'BackgroundColor',t.panel, ...
    'BorderColor',t.border,'FontName',t.font,'FontWeight','bold','FontSize',13);
g=uigridlayout(p,[numel(labels) 2]); g.ColumnWidth={'1x','1.6x'};
g.RowHeight=repmat({'1x'},1,numel(labels)); g.Padding=[12 5 12 5]; g.RowSpacing=2; g.BackgroundColor=t.panel;
for k=1:numel(labels)
    uilabel(g,'Text',labels{k},'FontName',t.font,'FontSize',12,'FontColor',t.muted);
    values.(fields{k})=uilabel(g,'Text','—','FontName',t.font,'FontSize',13, ...
        'FontWeight','bold','FontColor',t.text,'HorizontalAlignment','right','Tooltip',fields{k});
end
end

