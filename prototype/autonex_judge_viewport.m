function autonex_judge_viewport(ax)
% Display-only framing: equal metres, ego following, full-width plot box.
if ~isstruct(ax.UserData) || ~isfield(ax.UserData,'egoX'), return; end
u=ax.UserData; pixels=getpixelposition(ax); span=70;
height=span*pixels(4)/max(1,pixels(3));
view(ax,2); axis(ax,'equal'); ax.PlotBoxAspectRatioMode='auto';
ax.CameraPositionMode='auto'; ax.CameraTargetMode='auto';
ax.CameraUpVectorMode='auto'; ax.CameraViewAngleMode='auto';
xlim(ax,[u.egoX-15 u.egoX+55]);
ylim(ax,u.centerY+[-1 1]*height/2); zlim(ax,[-.1 4]);
end
