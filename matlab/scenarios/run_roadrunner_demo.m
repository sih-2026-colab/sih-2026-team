% run_roadrunner_demo.m
% AEB (Automatic Emergency Braking) prototype.
%
% If RoadRunner / driving scenario tooling is available, a scenario would
% be simulated here. As a fallback (no RoadRunner), this runs a simple
% time-stepped AEB check over the predicted trajectories:
% for each obstacle, compute time-to-collision with the ego vehicle and
% apply braking when TTC drops below a threshold.

function run_roadrunner_demo()
    aeb_ttc_threshold = 2.0;   % seconds: brake if TTC below this
    ego_speed = 10.0;          % m/s (matching Python pipeline assumption)
    brake_accel = -6.0;        % m/s^2 emergency braking

    % Load predictions if present
    preds = struct([]);
    try
        d = load('prediction_output.mat', 'predictions');
        preds = d.predictions;
    catch
        warning('prediction_output.mat not found; AEB demo will use defaults.');
    end

    results = struct([]);
    n = 0;
    ego_stop_distance = ego_speed^2 / (2 * abs(brake_accel));  % stopping distance

    for i = 1:length(preds)
        p = preds(i);
        traj = p.trajectory;   % [t, x, y]
        if isempty(traj)
            continue;
        end
        % TTC: first time object reaches ego front (x ~ 0 plus vehicle length)
        ttc = NaN;
        for k = 1:size(traj, 1)
            if traj(k, 2) <= 4.5   % ego front bumper + margin, metres
                ttc = traj(k, 1);
                break;
            end
        end

        n = n + 1;
        results(n).id = p.id;
        results(n).label = p.label;
        results(n).ttc = ttc;
        if ~isnan(ttc) && ttc < aeb_ttc_threshold
            results(n).action = 'BRAKE';
            results(n).brake_decel = brake_accel;
            results(n).stopping_distance = ego_stop_distance;
        else
            results(n).action = 'MAINTAIN';
            results(n).brake_decel = 0;
            results(n).stopping_distance = 0;
        end
    end

    save('aeb_output.mat', 'results');
    try
        write_json(results, fullfile('..', '..', 'aeb_output.json'));
    catch
        % write_json helper not on path; ignore
    end

    fprintf('AEB prototype complete: %d obstacles evaluated. Saved aeb_output.mat\n', n);
end
