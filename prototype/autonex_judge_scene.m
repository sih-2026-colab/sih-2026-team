function scene=autonex_judge_scene(action,varargin)
switch action
case 'create'
    ax=varargin{1}; t=varargin{2}; scene=struct('axes',ax,'theme',t,'followX',NaN);
    scene.road=patch(ax,nan,nan,nan,t.road,'EdgeColor',t.border,'FaceAlpha',1);
    scene.free=plot3(ax,nan,nan,nan,'.','Color',[.15 .28 .30],'MarkerSize',3);
    scene.corridor=plot3(ax,nan,nan,nan,'--','Color',[.26 .45 .48],'LineWidth',1);
    scene.candidates=plot3(ax,nan,nan,nan,'Color',[.34 .43 .49],'LineWidth',.6);
    scene.selected=plot3(ax,nan,nan,nan,'Color',t.green,'LineWidth',4);
    scene.tracks=plot3(ax,nan,nan,nan,'s','Color',t.amber,'MarkerSize',9,'LineWidth',1.5,'LineStyle','none');
    scene.actors=gobjects(0); scene.labels=gobjects(0);
case 'update'
    scene=varargin{1}; state=varargin{2}; out=varargin{3}; ax=scene.axes; t=scene.theme;
    if isnan(scene.followX), scene.followX=out.x; else, scene.followX=.85*scene.followX+.15*out.x; end
    low=scene.followX-8; high=scene.followX+43;
    top=strcmp(ax.Tag,'WorldModel');
    if top, low=out.x-15; high=out.x+55; end
    geometry=autonex_road_geometry(out.actors); vertices=[]; faces=[];
    for k=1:size(geometry.rectangles,1)
        r=geometry.rectangles(k,:); a=max(low,r(1)); b=min(high,r(2));
        if b<a, continue; end
        faces(end+1,:)=size(vertices,1)+(1:4); %#ok<AGROW>
        vertices=[vertices;a r(3) 0;b r(3) 0;b r(4) 0;a r(4) 0]; %#ok<AGROW>
    end
    set(scene.road,'Vertices',vertices,'Faces',faces);
    if isfield(state,'drivable')
        mask=state.drivable.freeMask; [row,col]=find(mask);
        idx=1:3:numel(row); x=state.drivable.xValues(col(idx)); y=state.drivable.yValues(row(idx));
        set(scene.free,'XData',x,'YData',y,'ZData',.01*ones(size(x)));
    end
    if isfield(state,'corridor') && state.corridor.valid
        set(scene.corridor,'XData',state.corridor.x,'YData',state.corridor.referenceY,'ZData',.025*ones(size(state.corridor.x)));
    else, set(scene.corridor,'XData',nan,'YData',nan,'ZData',nan); end
    x=[]; y=[];
    for k=1:numel(out.candidates)
        x=[x out.candidates(k).trajectory.x NaN]; y=[y out.candidates(k).trajectory.y NaN]; %#ok<AGROW>
    end
    set(scene.candidates,'XData',x,'YData',y,'ZData',.05*ones(size(x)));
    if isempty(out.selected), x=nan; y=nan;
    else, x=out.selected.trajectory.x; y=out.selected.trajectory.y; end
    set(scene.selected,'XData',x,'YData',y,'ZData',.10*ones(size(x)));
    x=[]; y=[];
    for k=1:numel(out.tracks), x(end+1)=out.tracks(k).State(1); y(end+1)=out.tracks(k).State(3); end %#ok<AGROW>
    set(scene.tracks,'XData',x,'YData',y,'ZData',1.7*ones(size(x)));
    for k=1:numel(out.actors)
        if k>numel(scene.actors)
            scene.actors(k)=patch(ax,'Vertices',zeros(8,3),'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8], ...
                'FaceColor',t.muted,'EdgeColor',t.border);
            scene.labels(k)=text(ax,0,0,0,'','Color',t.text,'FontSize',10,'Interpreter','none','Clipping','on');
        end
        a=out.actors(k); [L,W]=autonex_perception_dimensions(a); h=a.heading;
        if k==1, h=out.egoYaw; end
        R=[cos(h) -sin(h);sin(h) cos(h)]; xy=[-L -W;L -W;L W;-L W]/2*R'+[a.x a.y];
        height=1.2; if strcmpi(a.type,'pedestrian'), height=1.7; end
        set(scene.actors(k),'Vertices',[xy .15*ones(4,1);xy height*ones(4,1)],'Visible','on');
        color=t.muted; if k==1, color=t.cyan; end
        set(scene.actors(k),'FaceColor',color);
        label=a.name; if k==1, label='EGO'; end
        if top
            label=sprintf('%s %d',upper(a.type),a.id); if k==1, label='EGO'; end
            set(scene.actors(k),'EdgeColor',color,'LineWidth',1.5);
        end
        set(scene.labels(k),'Position',[a.x a.y+W/2+.5 height+.3],'String',label,'Visible','on');
        if k>1
            direction=1; if mod(k,2)==0, direction=-1; end
            set(scene.labels(k),'Position',[a.x a.y+direction*(W/2+1.2) height+.4], ...
                'VerticalAlignment','middle','FontSize',9);
        end
    end
    for k=numel(out.actors)+1:numel(scene.actors)
        scene.actors(k).Visible='off'; scene.labels(k).Visible='off';
    end
    xlim(ax,[low high]); ylim(ax,[-5 15]); zlim(ax,[-.1 4]);
    if ~top, ax.CameraViewAngleMode='auto'; camzoom(ax,1.4); end
end
end

