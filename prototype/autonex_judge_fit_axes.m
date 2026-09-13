function autonex_judge_fit_axes(ax)
% Explicit pixel layout avoids normalized-axis collapse during scaled UI setup.
p=getpixelposition(ax.Parent); w=max(100,p(3)); h=max(80,p(4)-24);
ax.Units='pixels'; ax.PositionConstraint='innerposition';
ax.Position=[.05*w .12*h .94*w .85*h];
if strcmp(ax.Tag,'WorldModel'), autonex_judge_viewport(ax); end
end
