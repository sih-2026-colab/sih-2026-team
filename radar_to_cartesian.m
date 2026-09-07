function [globalX, globalY, relativeX, relativeY] = ...
    radar_to_cartesian(detection, ego)

    %% =====================================================
    % Convert radar polar measurement:
    %
    % range + radar-relative angle
    %
    % into ego-relative and global Cartesian coordinates.
    %% =====================================================

    range = detection.range;

    % Convert radar-relative angle into vehicle/global angle
    globalAngleDeg = ...
        detection.angle + detection.mountAngle;


    %% Ego-relative position

    relativeX = ...
        range * cosd(globalAngleDeg);

    relativeY = ...
        range * sind(globalAngleDeg);


    %% Global position
    %
    % Our current highway model assumes ego heading = 0 deg.

    globalX = ...
        ego.x + relativeX;

    globalY = ...
        ego.y + relativeY;

end