function result = simulate_aeb(aeb_enabled, scene)
%SIMULATE_AEB Run the urban-intersection prototype without RoadRunner.
    if nargin < 1, aeb_enabled = true; end
    if nargin < 2, scene = urban_intersection(); end

    thisDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(thisDir, '..', 'control'));
    addpath(fullfile(thisDir, '..', 'scenarios'));

    ego = scene.ego;
    objects = scene.objects;
    mode = 'CRUISE';
    collision_with = '';
    t = 0;
    n = 0;

    while t <= scene.t_end
        n = n + 1;
        [ttc, threat] = aeb_logic(ego, objects, scene.hit_dist_m, 3.0, scene.dt);
        if aeb_enabled
            [ax, mode] = aeb_step(ego, objects, mode, scene.ttc_brake_s, scene.a_brake, scene.hit_dist_m);
        else
            ax = 0;
        end
        for i = 1:numel(objects)
            if hypot(ego.x - objects(i).x, ego.y - objects(i).y) <= scene.hit_dist_m
                collision_with = objects(i).label;
                break
            end
        end
        log(n).t = t; %#ok<AGROW>
        log(n).ego_x = ego.x;
        log(n).ego_vx = ego.vx;
        log(n).ttc = ttc;
        log(n).threat = threat;
        log(n).mode = mode;
        log(n).collision = collision_with;
        if ~isempty(collision_with)
            break
        end
        ego.vx = max(0, ego.vx + ax * scene.dt);
        ego.x = ego.x + ego.vx * scene.dt;
        ego.y = ego.y + ego.vy * scene.dt;
        for i = 1:numel(objects)
            objects(i).x = objects(i).x + objects(i).vx * scene.dt;
            objects(i).y = objects(i).y + objects(i).vy * scene.dt;
        end
        t = t + scene.dt;
    end

    result.aeb_enabled = aeb_enabled;
    result.collision = ~isempty(collision_with);
    result.collision_with = collision_with;
    result.final_mode = mode;
    result.final_speed = ego.vx;
    result.final_x = ego.x;
    result.log = log;
end
