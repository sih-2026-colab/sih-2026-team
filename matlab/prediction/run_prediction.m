% run_prediction.m
% Starter script: load perception detections from a MAT-file, predict
% future trajectories (constant-velocity baseline), and save predictions.
%
% Input:  'perception_output.mat' containing variable 'detections'
% Output: 'prediction_output.mat'  containing variable 'predictions'

function run_prediction()
    try
        data = load('perception_output.mat', 'detections');
        detections = data.detections;
    catch
        warning('perception_output.mat not found or missing ''detections''. Using empty array.');
        detections = struct([]);
    end

    horizon_s = 3.0;   % prediction horizon (seconds)
    dt = 0.5;          % timestep (seconds)
    steps = round(horizon_s / dt);

    predictions = struct([]);
    for i = 1:length(detections)
        d = detections(i);
        traj = zeros(steps, 3);  % [t, x, y]
        for k = 1:steps
            t = k * dt;
            traj(k, :) = [t, d.x + d.speed * t, d.y];
        end
        predictions(i).id = d.id;
        predictions(i).label = d.label;
        predictions(i).trajectory = traj;
        predictions(i).model = 'constant_velocity';
    end

    save('prediction_output.mat', 'predictions');
    try
        write_json(predictions, fullfile('..','..','prediction_output.json'));
    catch
        % write_json helper not on path; ignore
    end
    disp('Prediction run complete. Saved prediction_output.mat');
end
