function p=autonex_judge_overlay(action,varargin)
% Fixed line handles; pooled patches grow only to the track high-water mark.
if strcmp(action,'create')
    ax=varargin{1}; t=varargin{2}; p=struct('axes',ax,'theme',t);
    p.safe=plot3(ax,nan,nan,nan,'-','Color',t.cyan,'LineWidth',.8);
    p.rejected=plot3(ax,nan,nan,nan,'--','Color',t.red,'LineWidth',.8);
    p.prediction=plot3(ax,nan,nan,nan,'--o','Color',t.amber,'MarkerSize',4,'LineWidth',1.2);
    p.bubbles=gobjects(0); p.uncertainty=gobjects(0); p.labels=gobjects(0); return;
end
p=varargin{1}; d=varargin{2}; out=varargin{3}; ax=p.axes; t=p.theme;
for safe=[true false]
    x=[]; y=[];
    if d.available
        for k=find(d.safe==safe)
            tr=out.candidates(k).trajectory; x=[x tr.x NaN]; y=[y tr.y NaN]; %#ok<AGROW>
        end
    end
    h=p.rejected; if safe, h=p.safe; end
    set(h,'XData',x,'YData',y,'ZData',.07*ones(size(x)));
end
x=[]; y=[];
for k=1:numel(d.predictions)
    a=d.predictions(k);
    if k>numel(p.bubbles)
        p.bubbles(k)=patch(ax,nan,nan,nan,t.cyan,'FaceAlpha',.08,'EdgeColor',t.cyan);
        p.uncertainty(k)=patch(ax,'Faces',nan,'Vertices',nan(1,3),'FaceColor',t.amber,'FaceAlpha',.18,'EdgeColor','none');
        p.labels(k)=text(ax,0,0,0,'','Color',t.amber,'FontSize',9,'Clipping','on','Interpreter','none');
    end
    x=[x a.xy(1,:) NaN]; y=[y a.xy(2,:) NaN]; %#ok<AGROW>
    color=t.cyan; if a.conflict, color=t.red; end
    set(p.bubbles(k),'XData',a.bubble(1,:),'YData',a.bubble(2,:),'ZData',.03*ones(1,size(a.bubble,2)), ...
        'FaceColor',color,'EdgeColor',color,'Visible','on');
    vertices=[]; faces=[];
    for j=1:numel(a.ellipses)
        v=a.ellipses{j}'; faces(end+1,:)=size(vertices,1)+(1:size(v,1)); %#ok<AGROW>
        vertices=[vertices;v .04*ones(size(v,1),1)]; %#ok<AGROW>
    end
    set(p.uncertainty(k),'Vertices',vertices,'Faces',faces,'Visible','on');
    set(p.labels(k),'Position',[a.xy(1,1) a.xy(2,1)+a.envelope.lateral+.4 1.8], ...
        'String',sprintf('T%d / envelope %.1f x %.1f m',a.id,a.envelope.longitudinal,a.envelope.lateral),'Visible','on');
end
for k=numel(d.predictions)+1:numel(p.bubbles)
    p.bubbles(k).Visible='off'; p.uncertainty(k).Visible='off'; p.labels(k).Visible='off';
end
set(p.prediction,'XData',x,'YData',y,'ZData',.2*ones(size(x)));
view(ax,2); axis(ax,'normal'); ax.CameraViewAngleMode='auto';
end
