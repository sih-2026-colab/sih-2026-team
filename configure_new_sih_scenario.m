function actors = configure_new_sih_scenario(actors,name)
% Scenario data only: metres, seconds, m/s; world +X is ego forward.
% Reuse the existing actor schema. No hidden/reveal flags or planner hooks.
% At least three actors are retained for update_highway_actors compatibility.
% Crossing actors continue under the existing constant-velocity integration.
base=actors(1);
actors=base;
actors(1).vx=8; actors(1).vy=0;
actors(1).ax=0; actors(1).ay=0; actors(1).heading=0;
switch name
    case 'animal_crossing'
        actors(1).vx=10;
        % Animal reaches Y=7 at 3.33 s; unbraked ego reaches X=32 at 3.2 s.
        % 32 m exceeds the initial 10^2/(2*6)=8.33 m braking distance.
        specs={ ...
            'CROSSING-ANIMAL','animal',32,12,0,-1.5,.6; ...
            'ROADSIDE-PARKED','car',55,0,0,0,.2};
    case 'occluded_pedestrian'
        % At t=0, ego-to-pedestrian ray passes through the parked car.
        % Pedestrian starts beyond its front bumper and walks toward Y=7.
        % Depth scan respects this occlusion; radar/RGB/thermal currently do
        % NOT reject actors hidden by other actors. See coverage notes.
        specs={ ...
            'PARKED-OCCLUDER','car',24,4.5,0,0,.2; ...
            'EMERGING-PEDESTRIAN','pedestrian',28,4,0,1,.7};
    case 'unsignalized_intersection'
        % Lightweight crossing conflict at X=32, not a junction road mesh.
        % Crossing car and unbraked ego reach (32,7) at 4 s.
        % Heading records motion direction; existing non-ego boxes remain
        % axis-aligned and road boundaries remain the straight-road strip.
        specs={ ...
            'CROSSING-CAR','car',32,-5,0,3,.5; ...
            'JUNCTION-PEDESTRIAN','pedestrian',38,12,0,-1,.6};
    case 'dense_market'
        actors(1).vx=6;
        % Supported types only. Bike inherits the existing car-sized box.
        % Irregular positions, oncoming flow and lateral walkers; no lanes.
        specs={ ...
            'SLOW-MARKET-CAR','car',24,5.1,1.5,0,.3; ...
            'ONCOMING-BIKE','bike',42,1.1,-2,.1,.5; ...
            'MARKET-WALKER','pedestrian',20,2.4,.3,.7,.7; ...
            'MARKET-ANIMAL','animal',36,8.2,.2,-.45,.7; ...
            'RETURNING-WALKER','pedestrian',48,6.2,-.5,-.35,.6; ...
            'PARKED-STALL-CAR','car',58,-.4,0,0,.2};
    otherwise
        error('AutoNex:UnknownNewScenario','Unknown new SIH scenario: %s',name);
end
for k=1:size(specs,1)
    actor=base;
    actor.id=k+1; actor.name=specs{k,1}; actor.type=specs{k,2};
    actor.x=specs{k,3}; actor.y=specs{k,4};
    actor.vx=specs{k,5}; actor.vy=specs{k,6};
    actor.ax=0; actor.ay=0;
    actor.heading=atan2(actor.vy,actor.vx);
    actor.uncertainty=specs{k,7};
    actors(k+1)=actor; %#ok<AGROW>
end
end
