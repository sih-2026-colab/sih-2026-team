function actors = configure_autonex_scenario(actors,name)

% Dedicated SIH cases share sensing, fusion, tracking and control,
% not canned planner outputs.

% New coverage is isolated so existing scenario geometry stays unchanged.
if ismember(name,{'animal_crossing','occluded_pedestrian', ...
        'unsignalized_intersection','dense_market'})
    actors = configure_new_sih_scenario(actors,name);
    return;
end
if ~ismember(name,{ ...
        'night_pedestrian', ...
        'animal', ...
        'pothole', ...
        'missing_lane'})

    return;

end


%% =========================================================
% MOVE NORMAL HIGHWAY TRAFFIC OUT OF TEST AREA
%% =========================================================

for k = 2:numel(actors)

    actors(k).x = ...
        250 + k*30;

    actors(k).vy = 0;

end


%% Moderate speed for special SIH scenarios

actors(1).vx = 10;


%% =========================================================
% SCENARIO CONFIGURATION
%% =========================================================

switch name

    %% -----------------------------------------------------
    % NIGHT PEDESTRIAN
    %% -----------------------------------------------------

    case 'night_pedestrian'

        actors(3).type = ...
            'pedestrian';

        actors(3).name = ...
            'PEDESTRIAN';

        actors(3).x = 24;

        actors(3).y = 3.5;

        actors(3).vx = 0;

        actors(3).vy = 1.2;

        actors(3).ax = 0;

        actors(3).ay = 0;


    %% -----------------------------------------------------
    % ANIMAL
    %% -----------------------------------------------------

    case 'animal'

        actors(3).type = ...
            'animal';

        actors(3).name = ...
            'ANIMAL';

        actors(3).x = 28;

        actors(3).y = 7;

        actors(3).vx = 0.3;

        actors(3).vy = 0;

        actors(3).ax = 0;

        actors(3).ay = 0;


    %% -----------------------------------------------------
    % POTHOLE
    %% -----------------------------------------------------

    case 'pothole'

        actors(3).type = ...
            'pothole';

        actors(3).name = ...
            'POTHOLE';

        actors(3).x = 28;

        actors(3).y = 7;

        actors(3).vx = 0;

        actors(3).vy = 0;

        actors(3).ax = 0;

        actors(3).ay = 0;


    %% -----------------------------------------------------
    % MISSING-LANE / UNSTRUCTURED-ROAD TEST
    %
    % Two staggered stationary obstacles force the
    % occupancy-derived corridor to move laterally.
    %
    % The planner must NOT simply follow Y = 7.
    %% -----------------------------------------------------

    case 'missing_lane'

        %% Upper-side blocker directly on old Y=7 path

        actors(3).type = ...
            'car';

        actors(3).name = ...
            'UPPER-BLOCKER';

        actors(3).x = 28;

        actors(3).y = 7.0;

        actors(3).vx = 0;

        actors(3).vy = 0;

        actors(3).ax = 0;

        actors(3).ay = 0;


        %% Lower-side blocker farther ahead
        %
        % This creates a non-trivial free-space corridor
        % rather than one straight fixed lane.

        actors(4).type = ...
            'car';

        actors(4).name = ...
            'LOWER-BLOCKER';

        actors(4).x = 48;

        actors(4).y = 0.0;

        actors(4).vx = 0;

        actors(4).vy = 0;

        actors(4).ax = 0;

        actors(4).ay = 0;

end

end