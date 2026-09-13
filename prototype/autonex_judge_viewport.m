function autonex_judge_viewport(ax)
% Display-only framing: equal metres, ego following, full-width plot box.
if ~isstruct(ax.UserData) || ~isfield(ax.UserData,'egoX'), return; end
u=ax.UserData;
p=getpixelposition(ax.Parent); w=max(100,p(3)); h=max(80,p(4)-24);
pixels=[.05*w .12*h .94*w .85*h]; span=70;
height=span*pixels(4)/max(1,pixels(3));
view(ax,2); axis(ax,'equal'); ax.PlotBoxAspectRatioMode='auto';
ax.CameraPositionMode='auto'; ax.CameraTargetMode='auto';
ax.CameraUpVectorMode='auto'; ax.CameraViewAngleMode='auto';
xlim(ax,[u.egoX-15 u.egoX+55]);
ylim(ax,u.centerY+[-1 1]*height/2); zlim(ax,[-.1 4]);
% Set the orthographic camera's vertical field explicitly. MATLAB's automatic
% 3-D camera fit otherwise reserves space around the shallow world box.
centerX=u.egoX+20;
ax.CameraPosition=[centerX u.centerY 100];
ax.CameraTarget=[centerX u.centerY 0]; ax.CameraUpVector=[0 1 0];
ax.Projection='orthographic'; ax.CameraViewAngle=2*atand(height/200);
ax.Position=pixels;
end
