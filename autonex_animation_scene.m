function scene = autonex_animation_scene(action, varargin)
% Graphics only: all motion and decisions come from the original planner.
switch action
    case 'create'
        actors = varargin{1}; low = varargin{2}; high = varargin{3};
        scene.figure = figure('Name','AutoNex | Live driving simulation', ...
            'Color',[0.055 0.075 0.11], 'Position',[80 80 1280 720], 'Resize','off');
        scene.axes = axes('Parent',scene.figure,'Position',[0.04 0.10 0.92 0.73]);
        ax = scene.axes;
        hold(ax,'on'); axis(ax,'equal');
        set(ax,'Color',[0.55 0.72 0.83],'XColor',[0.8 0.85 0.9], ...
            'YColor',[0.8 0.85 0.9],'ZColor',[0.8 0.85 0.9]);
        patch(ax,[-150 500 500 -150],[-80 -80 80 80],[-.08 -.08 -.08 -.08], ...
            [0.24 0.39 0.26],'EdgeColor','none');
        patch(ax,[-150 500 500 -150],[low low high high],[0 0 0 0], ...
            [0.19 0.21 0.24],'EdgeColor','none');
        for y = [low high]
            plot3(ax,[-150 500],[y y],[.02 .02],'Color',[1 .85 .3],'LineWidth',2);
        end
        for y = [1.75 5.25]
            for x = -150:9:500
                plot3(ax,[x x+4],[y y],[.025 .025],'w','LineWidth',2);
            end
        end
        scene.candidates = gobjects(0);
        scene.selected = plot3(ax,nan,nan,nan,'Color',[0.1 1 0.72],'LineWidth',3);
        scene.tracks = plot3(ax,nan,nan,nan,'x','Color',[1 .85 .2],'MarkerSize',9,'LineWidth',1.5);
        colors = [0.08 .70 .95; .6 .65 .75; 1 .32 .13; .72 .42 .9; .95 .72 .16];
        scene.cars = gobjects(1,numel(actors));
        scene.labels = gobjects(1,numel(actors));
        for i = 1:numel(actors)
            group = hgtransform('Parent',ax);
            color = colors(mod(i-1,size(colors,1))+1,:);
            boxPart(group,[-2.25 2.25],[-.95 .95],[.25 .95],color);
            boxPart(group,[-1.15 1.15],[-.78 .78],[.95 1.48],color*.85);
            boxPart(group,[.65 1.12],[-.79 .79],[1.15 1.49],[.15 .27 .34]);
            for x = [-1.45 1.45]
                for y = [-.98 .98]
                    boxPart(group,[x-.38 x+.38],[y-.12 y+.12],[.08 .58],[.045 .05 .06]);
                end
            end
            for y = [-.68 .68]
                boxPart(group,[2.25 2.28],[y-.17 y+.17],[.55 .75],[1 1 .78]);
                boxPart(group,[-2.28 -2.25],[y-.17 y+.17],[.55 .75],[1 .1 .08]);
            end
            scene.cars(i) = group;
            scene.labels(i) = text(ax,0,0,2,actors(i).name,'Color','w', ...
                'FontWeight','bold','FontSize',9,'HorizontalAlignment','center','Interpreter','none');
        end
        scene.hud = annotation(scene.figure,'textbox',[.04 .85 .92 .13], ...
            'Color','w','EdgeColor','none','FontName','Consolas','FontSize',12,'Interpreter','none');
        annotation(scene.figure,'textbox',[.04 .01 .92 .05], ...
            'String','Cyan: ego | Orange: cut-in vehicle | Green: selected trajectory | Yellow crosses: confirmed tracks', ...
            'Color',[.8 .85 .9],'EdgeColor','none','FontSize',10);
        xlabel(ax,'Forward position (m)'); ylabel(ax,'Lateral position (m)');
        zlim(ax,[0 7]); view(ax,[-65 35]);
        camproj(ax,'perspective');
    case 'update'
        scene = varargin{1}; actors = varargin{2}; candidates = varargin{3};
        selected = varargin{4}; tracks = varargin{5}; d = varargin{6};
        ax = scene.axes;
        for i = 1:numel(actors)
            heading = atan2(actors(i).vy,max(actors(i).vx,0.01));
            scene.cars(i).Matrix = makehgtform('translate',[actors(i).x actors(i).y 0], ...
                'zrotate',heading);
            scene.labels(i).Position = [actors(i).x actors(i).y 2.1];
        end
        if numel(scene.candidates) ~= numel(candidates)
            delete(scene.candidates);
            scene.candidates = gobjects(1,numel(candidates));
            for i = 1:numel(candidates)
                scene.candidates(i) = plot3(ax,nan,nan,nan,'Color',[.38 .52 .6],'LineWidth',.5);
            end
        end
        for i = 1:numel(candidates)
            tr = candidates(i).trajectory;
            set(scene.candidates(i),'XData',tr.x,'YData',tr.y,'ZData',.04*ones(size(tr.x)));
        end
        if isempty(selected)
            set(scene.selected,'XData',nan,'YData',nan,'ZData',nan);
        else
            tr = selected.trajectory;
            set(scene.selected,'XData',tr.x,'YData',tr.y,'ZData',.07*ones(size(tr.x)));
        end
        xy = zeros(numel(tracks),2);
        for k = 1:numel(tracks)
            state = tracks(k).State; xy(k,:) = [state(1) state(3)];
        end
        set(scene.tracks,'XData',xy(:,1),'YData',xy(:,2),'ZData',1.9*ones(size(xy,1),1));
        x = actors(1).x;
        xlim(ax,[x-25 x+65]); ylim(ax,[-5 12]);
        campos(ax,[x-28 -24 24]); camtarget(ax,[x+16 3.5 0]); camva(ax,48);
        scene.hud.String = sprintf(['AUTONEX  |  LIVE CLOSED-LOOP SIMULATION  |  %.2f s\n' ...
            'Speed %.1f km/h   Target %.1f km/h   Target Y %.2f m   Cut-in %.1f%%\n' ...
            'Planner: %s   Maneuver: %s   Stability: %s   Guardian: %s'], ...
            d.time,d.speed,d.targetSpeed,d.targetY,d.pCut,d.selectionMode, ...
            d.selectedName,d.stabilityMode,d.guardianMode);
    otherwise
        error('AutoNex:AnimationAction','Unknown animation action.');
end
end

function boxPart(parent,x,y,z,color)
vertices = [x(1) y(1) z(1); x(2) y(1) z(1); x(2) y(2) z(1); x(1) y(2) z(1); ...
            x(1) y(1) z(2); x(2) y(1) z(2); x(2) y(2) z(2); x(1) y(2) z(2)];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Parent',parent,'Vertices',vertices,'Faces',faces,'FaceColor',color, ...
    'EdgeColor',color*.65,'LineWidth',.5);
end
