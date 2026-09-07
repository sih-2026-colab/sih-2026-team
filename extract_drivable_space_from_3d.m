function drivable = ...
    extract_drivable_space_from_3d( ...
        map3D, ...
        ego, ...
        forwardDistance, ...
        lateralMin, ...
        lateralMax)

    %% =====================================================
    % AUTONEX LANE-INDEPENDENT DRIVABLE-SPACE EXTRACTION
    %
    % Classification:
    %  -1 = UNKNOWN
    %   0 = FREE
    %   1 = OCCUPIED
    %% =====================================================


    %% =====================================================
    % GRID
    %% =====================================================

    xResolution = 0.50;
    yResolution = 0.25;


    xValues = ...
        ego.x:xResolution:(ego.x + forwardDistance);


    yValues = ...
        lateralMin:yResolution:lateralMax;


    [X,Y] = ...
        meshgrid( ...
            xValues, ...
            yValues);


    %% =====================================================
    % VEHICLE-RELEVANT HEIGHTS
    %% =====================================================

    zLevels = [
        0.30
        0.60
        0.90
        1.20
        1.50
    ];


    occupancyStack = ...
        zeros( ...
            size(X,1), ...
            size(X,2), ...
            length(zLevels));


    %% =====================================================
    % QUERY OCCUPANCY
    %% =====================================================

    for zIndex = 1:length(zLevels)

        queryPoints = [
            X(:) ...
            Y(:) ...
            zLevels(zIndex) * ...
            ones(numel(X),1)
        ];


        occupancyValues = ...
            getOccupancy( ...
                map3D, ...
                queryPoints);


        occupancyStack(:,:,zIndex) = ...
            reshape( ...
                occupancyValues, ...
                size(X));

    end


    %% =====================================================
    % OCCUPANCY STATISTICS PER X-Y COLUMN
    %% =====================================================

    maximumOccupancy = ...
        max( ...
            occupancyStack, ...
            [], ...
            3);


    minimumOccupancy = ...
        min( ...
            occupancyStack, ...
            [], ...
            3);


    %% =====================================================
    % IMPORTANT THRESHOLDS
    %
    % insertPointCloud typically contributes approximately:
    %
    % free observation     -> 0.4
    % unknown              -> 0.5
    % occupied observation -> 0.7
    %% =====================================================

    freeThreshold = 0.45;

    occupiedThreshold = 0.65;


    %% =====================================================
    % CLASSIFICATION
    %% =====================================================

    classification = ...
        -1 * ones(size(X));


    %% -----------------------------------------------------
    % OCCUPIED:
    % obstacle seen at any vehicle-relevant height
    %% -----------------------------------------------------

    occupiedMask = ...
        maximumOccupancy >= ...
        occupiedThreshold;


    %% -----------------------------------------------------
    % FREE:
    %
    % At least one height was positively observed free,
    % and NO height contains a confirmed obstacle.
    %% -----------------------------------------------------

    observedFreeMask = ...
        minimumOccupancy <= ...
        freeThreshold;


    freeMask = ...
        observedFreeMask & ...
        ~occupiedMask;


    %% -----------------------------------------------------
    % SAVE CLASSIFICATION
    %% -----------------------------------------------------

    classification(freeMask) = 0;

    classification(occupiedMask) = 1;


    unknownMask = ...
        classification == -1;


    %% =====================================================
    % OUTPUT
    %% =====================================================

    drivable.xValues = ...
        xValues;

    drivable.yValues = ...
        yValues;

    drivable.X = ...
        X;

    drivable.Y = ...
        Y;

    drivable.classification = ...
        classification;

    drivable.maximumOccupancy = ...
        maximumOccupancy;

    drivable.minimumOccupancy = ...
        minimumOccupancy;

    drivable.freeMask = ...
        freeMask;

    drivable.occupiedMask = ...
        occupiedMask;

    drivable.unknownMask = ...
        unknownMask;

    drivable.freeThreshold = ...
        freeThreshold;

    drivable.occupiedThreshold = ...
        occupiedThreshold;

end