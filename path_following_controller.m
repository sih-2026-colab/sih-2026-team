function control=path_following_controller(ego,yaw,previousSteering,path,opts,previousPath,safetyOverride)
% Pure Pursuit on the planner-approved sampled path. Empty means no tracking.
control=struct('steeringAngle',0,'targetHeading',yaw,'lateralError',0,'selectedPathY',ego.y);
desired=0;
if nargin<6, previousPath=[]; end
if nargin<7, safetyOverride=false; end
if ~isempty(path)
    x=path.x(:); y=path.y(:);
    assert(all(isfinite([x;y])),'Nonfinite selected path');
    [~,near]=min(hypot(x-ego.x,y-ego.y));
    arc=[0;cumsum(hypot(diff(x),diff(y)))];
    [arc,uniqueIndex]=unique(arc,'stable'); x=x(uniqueIndex); y=y(uniqueIndex);
    if numel(arc)>1
        near=min(near,numel(arc));
        distance=max(opts.pathLookaheadMin,opts.pathLookaheadTime*hypot(ego.vx,ego.vy));
        target=min(arc(end),arc(near)+distance);
        tx=interp1(arc,x,target); ty=interp1(arc,y,target);
        heading=atan2(ty-ego.y,tx-ego.x);
        alpha=atan2(sin(heading-yaw),cos(heading-yaw));
        desired=atan2(2*opts.wheelbase*sin(alpha),max(hypot(tx-ego.x,ty-ego.y),.1));
        control.targetHeading=heading; control.selectedPathY=ty;
        segment=min(near,numel(x)-1);
        tangent=atan2(y(segment+1)-y(segment),x(segment+1)-x(segment));
        control.lateralError=-(ego.x-x(segment))*sin(tangent)+(ego.y-y(segment))*cos(tangent);
    end
end
% Measure execution error against the last commanded path, before replanning
% anchors a new trajectory at ego and would artificially reset error to zero.
if ~isempty(path) && ~isempty(previousPath)
    px=previousPath.x(:); py=previousPath.y(:);
    dx=diff(px); dy=diff(py); lengthSquared=dx.^2+dy.^2;
    fraction=max(0,min(1,((ego.x-px(1:end-1)).*dx+(ego.y-py(1:end-1)).*dy)./max(lengthSquared,eps)));
    ex=ego.x-(px(1:end-1)+fraction.*dx); ey=ego.y-(py(1:end-1)+fraction.*dy);
    [~,segment]=min(hypot(ex,ey));
    control.lateralError=(-ex(segment)*dy(segment)+ey(segment)*dx(segment))/sqrt(max(lengthSquared(segment),eps));
end
control.rawSteeringAngle=desired;
limit=min(opts.maxSteeringAngle,atan(opts.maxLateralAcceleration*opts.wheelbase/max(ego.vx^2+ego.vy^2,.1)));
if safetyOverride, limit=opts.maxSteeringAngle; end
desired=max(-limit,min(limit,desired));
control.requestedSteeringAngle=desired;
control.requestedSteeringRate=(desired-previousSteering)/opts.dt;
control.guardianOverrideActive=safetyOverride;
step=opts.maxSteeringRate*opts.dt;
if opts.enforcePhysicalSteeringRate
    % Magnitude/comfort shaping sets the request. Physical slew is last so
    % a changing comfort envelope cannot instantaneously reset the actuator.
    control.steeringAngle=previousSteering+max(-step,min(step,desired-previousSteering));
    assert(abs(control.steeringAngle)<=opts.maxSteeringAngle+1e-9,'Invalid steering actuator state');
else
    if safetyOverride, step=inf; end
    control.steeringAngle=max(-limit,min(limit,previousSteering+max(-step,min(step,desired-previousSteering))));
end
control.steeringRate=(control.steeringAngle-previousSteering)/opts.dt;
control.appliedSteeringAngle=control.steeringAngle;
control.appliedSteeringRate=control.steeringRate;
control.steeringSaturation=abs(control.rawSteeringAngle)>opts.maxSteeringAngle+1e-9;
control.steeringRateSaturation=abs(control.requestedSteeringRate)>opts.maxSteeringRate+1e-9;
end
