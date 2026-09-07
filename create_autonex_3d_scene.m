function [worldPoints, sensorPoints, sensorPose] = ...
    create_autonex_3d_scene(actors)

    %% =====================================================
    % AUTONEX SYNTHETIC 3-D SENSOR SCENE
    %
    % Coordinate system:
    % X = forward
    % Y = lateral
    % Z = height
    %
    % This is currently a synthetic LiDAR/depth abstraction.
    %% =====================================================

    ego = actors(1);

    %% Sensor mounted above ego vehicle

    sensorHeight = 1.20;

    sensorPose = [
        ego.x ...
        ego.y ...
        sensorHeight ...
        1 0 0 0
    ];


    worldPoints = [];


    %% =====================================================
    % 1. ROAD SURFACE
    %% =====================================================

    xRoad = ego.x:1.0:(ego.x + 70);

    yRoad = -1.75:0.75:8.75;

    [XR,YR] = meshgrid(xRoad,yRoad);

    ZR = zeros(size(XR));

    roadPoints = [
        XR(:) ...
        YR(:) ...
        ZR(:)
    ];

    worldPoints = [
        worldPoints
        roadPoints
    ];


    %% =====================================================
    % 2. ROAD EDGES
    %% =====================================================

    edgeX = ego.x:0.50:(ego.x + 70);

    edgeZ = 0:0.25:0.75;

    [EX,EZ] = meshgrid(edgeX,edgeZ);


    %% Left / lower road boundary

    lowerEdge = [
        EX(:) ...
        -1.75 * ones(numel(EX),1) ...
        EZ(:)
    ];


    %% Right / upper road boundary

    upperEdge = [
        EX(:) ...
        8.75 * ones(numel(EX),1) ...
        EZ(:)
    ];


    worldPoints = [
        worldPoints
        lowerEdge
        upperEdge
    ];


    %% =====================================================
    % 3. VEHICLES / ROAD USERS
    %% =====================================================

    vehicleLength = 4.5;
    vehicleWidth  = 1.9;
    vehicleHeight = 1.5;

    pointSpacing = 0.30;


    for i = 2:length(actors)

        vehiclePoints = ...
            create_box_surface_points( ...
                actors(i).x, ...
                actors(i).y, ...
                vehicleHeight/2, ...
                vehicleLength, ...
                vehicleWidth, ...
                vehicleHeight, ...
                pointSpacing);


        worldPoints = [
            worldPoints
            vehiclePoints
        ];

    end


    %% =====================================================
    % 4. CONVERT WORLD POINTS TO SENSOR COORDINATES
    %
    % Current prototype assumes zero ego heading.
    %% =====================================================

    sensorPosition = ...
        sensorPose(1:3);


    sensorPoints = ...
        worldPoints - ...
        sensorPosition;


end



%% =========================================================
% LOCAL HELPER — BOX SURFACE POINTS
%% =========================================================

function points = ...
    create_box_surface_points( ...
        cx,cy,cz,L,W,H,spacing)

    xValues = ...
        (cx-L/2):spacing:(cx+L/2);

    yValues = ...
        (cy-W/2):spacing:(cy+W/2);

    zValues = ...
        0:spacing:H;


    points = [];


    %% Front and rear surfaces

    [Y,Z] = ...
        meshgrid(yValues,zValues);


    front = [
        (cx+L/2)*ones(numel(Y),1) ...
        Y(:) ...
        Z(:)
    ];


    rear = [
        (cx-L/2)*ones(numel(Y),1) ...
        Y(:) ...
        Z(:)
    ];


    %% Left and right surfaces

    [X,Z] = ...
        meshgrid(xValues,zValues);


    leftSide = [
        X(:) ...
        (cy-W/2)*ones(numel(X),1) ...
        Z(:)
    ];


    rightSide = [
        X(:) ...
        (cy+W/2)*ones(numel(X),1) ...
        Z(:)
    ];


    %% Roof

    [X,Y] = ...
        meshgrid(xValues,yValues);


    roof = [
        X(:) ...
        Y(:) ...
        H*ones(numel(X),1)
    ];


    points = [
        front
        rear
        leftSide
        rightSide
        roof
    ];

end