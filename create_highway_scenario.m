function actors = create_highway_scenario()

    % Lane centres in metres
    lane1 = 0.0;
    lane2 = 3.5;
    lane3 = 7.0;

    % =========================================================
    % ACTOR 1 — AUTONEX EGO VEHICLE
    % Lane 3, 75 km/h
    % =========================================================
    actors(1).id = 1;
    actors(1).name = 'EGO';
    actors(1).type = 'car';

    actors(1).x = 0;
    actors(1).y = lane3;

    actors(1).vx = 75 / 3.6;
    actors(1).vy = 0;

    actors(1).ax = 0;
    actors(1).ay = 0;

    actors(1).heading = 0;
    actors(1).uncertainty = 0.2;


    % =========================================================
    % ACTOR 2 — REAR VEHICLE
    % Behind ego in lane 3, 70 km/h
    % =========================================================
    actors(2).id = 2;
    actors(2).name = 'REAR';
    actors(2).type = 'car';

    actors(2).x = -60;
    actors(2).y = lane3;

    actors(2).vx = 70 / 3.6;
    actors(2).vy = 0;

    actors(2).ax = 0;
    actors(2).ay = 0;

    actors(2).heading = 0;
    actors(2).uncertainty = 0.3;


    % =========================================================
    % ACTOR 3 — CUT-IN VEHICLE
    % Lane 2 → Lane 3, 80 km/h
    % =========================================================
    actors(3).id = 3;
    actors(3).name = 'CUT-IN';
    actors(3).type = 'car';

    actors(3).x = 35;
    actors(3).y = lane2;

    actors(3).vx = 80 / 3.6;

    % Positive lateral speed means moving toward lane 3
    actors(3).vy = 1.8;

    actors(3).ax = 0;
    actors(3).ay = 0;

    actors(3).heading = 0;
    actors(3).uncertainty = 0.5;


    % =========================================================
    % ACTOR 4 — LANE 1 TRAFFIC
    % 60 km/h
    % =========================================================
    actors(4).id = 4;
    actors(4).name = 'LANE1-CAR';
    actors(4).type = 'car';

    actors(4).x = 10;
    actors(4).y = lane1;

    actors(4).vx = 60 / 3.6;
    actors(4).vy = 0;

    actors(4).ax = 0;
    actors(4).ay = 0;

    actors(4).heading = 0;
    actors(4).uncertainty = 0.3;


    % =========================================================
    % ACTOR 5 — LANE 2 TRAFFIC
    % 60 km/h
    % =========================================================
    actors(5).id = 5;
    actors(5).name = 'LANE2-CAR';
    actors(5).type = 'car';

    actors(5).x = 38;
    actors(5).y = lane2;

    actors(5).vx = 60 / 3.6;
    actors(5).vy = 0;

    actors(5).ax = 0;
    actors(5).ay = 0;

    actors(5).heading = 0;
    actors(5).uncertainty = 0.3;

end