function safeSpace = ...
    inflate_drivable_space_for_vehicle( ...
        drivable, ...
        vehicleLength, ...
        vehicleWidth)

    %% =====================================================
    % AUTONEX VEHICLE-FOOTPRINT SAFETY INFLATION
    %
    % Expands occupied / unknown regions so planning can
    % operate using the EGO VEHICLE CENTER.
    %
    % No Image Processing Toolbox required.
    %% =====================================================


    %% =====================================================
    % SAFETY MARGINS
    %% =====================================================

    longitudinalMargin = 0.50;   % metres
    lateralMargin = 0.35;        % metres


    %% =====================================================
    % GRID RESOLUTION
    %% =====================================================

    xResolution = ...
        mean(diff(drivable.xValues));

    yResolution = ...
        mean(diff(drivable.yValues));


    %% =====================================================
    % REQUIRED INFLATION DISTANCE
    %% =====================================================

    halfLength = ...
        vehicleLength / 2 + ...
        longitudinalMargin;


    halfWidth = ...
        vehicleWidth / 2 + ...
        lateralMargin;


    inflateXCells = ...
        ceil( ...
            halfLength / ...
            xResolution);


    inflateYCells = ...
        ceil( ...
            halfWidth / ...
            yResolution);


    %% =====================================================
    % CONSERVATIVE BLOCKED SPACE
    %
    % UNKNOWN is not treated as safe driving space.
    %% =====================================================

    blockedMask = ...
        drivable.occupiedMask | ...
        drivable.unknownMask;


    %% =====================================================
    % RECTANGULAR VEHICLE-FOOTPRINT KERNEL
    %% =====================================================

    kernelHeight = ...
        2 * inflateYCells + 1;


    kernelWidth = ...
        2 * inflateXCells + 1;


    inflationKernel = ...
        ones( ...
            kernelHeight, ...
            kernelWidth);


    %% =====================================================
    % INFLATE BLOCKED CELLS
    %% =====================================================

    inflatedBlocked = ...
        conv2( ...
            double(blockedMask), ...
            inflationKernel, ...
            'same') > 0;


    %% =====================================================
    % SAFE CENTER LOCATIONS
    %
    % Ego center may occupy a cell only when:
    %
    % 1. that cell was positively observed free
    % 2. vehicle footprint does not overlap blocked space
    %% =====================================================

    safeFreeMask = ...
        drivable.freeMask & ...
        ~inflatedBlocked;

    % Outside the observed grid is unknown, not zero-padded free space.
    coverage = conv2(ones(size(blockedMask)), inflationKernel, 'same');
    safeFreeMask = safeFreeMask & coverage == numel(inflationKernel);


    %% =====================================================
    % OUTPUT
    %% =====================================================

    safeSpace.safeFreeMask = ...
        safeFreeMask;


    safeSpace.blockedMask = ...
        blockedMask;


    safeSpace.inflatedBlockedMask = ...
        inflatedBlocked;


    safeSpace.inflateXCells = ...
        inflateXCells;


    safeSpace.inflateYCells = ...
        inflateYCells;


    safeSpace.xResolution = ...
        xResolution;


    safeSpace.yResolution = ...
        yResolution;


    safeSpace.vehicleLength = ...
        vehicleLength;


    safeSpace.vehicleWidth = ...
        vehicleWidth;


    safeSpace.longitudinalMargin = ...
        longitudinalMargin;


    safeSpace.lateralMargin = ...
        lateralMargin;

end
