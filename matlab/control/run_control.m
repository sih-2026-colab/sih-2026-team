% run_control.m
% Control-layer prototype: converts AEB decisions + planned waypoints into
% actuator commands using a simple PID longitudinal controller.
%
% Inputs:  'aeb_output.mat'  (results from run_roadrunner_demo.m)
%          'planning_output.mat' (plans from run_planner.m)
% Output:  'control_output.mat' containing 'commands' (struct array)

function run_control()
    kp = 1.0; ki = 0.05; kd = 0.1;   % PID gains (prototype tuning)
    dt = 0.1;                        % control period, s
    target_speed = 10.0;             % m/s cruise target
    ego_speed = 10.0;                % current ego speed

    % Load AEB decisions
    aeb_action = 'MAINTAIN';
    try
        d = load('aeb_output.mat', 'results');
        % Brake if ANY obstacle demanded braking
        for i = 1:length(d.results)
            if strcmp(d.results(i).action, 'BRAKE')
                aeb_action = 'BRAKE';
                break;
            end
        end
    catch
        warning('aeb_output.mat not found; controller assumes MAINTAIN.');
    end

    % Load planner target speed if available
    try
        d2 = load('planning_output.mat', 'plans');
        if ~isempty(d2.plans) && isfield(d2.plans(1), 'waypoints')
            wp = d2.plans(1).waypoints;
            if numel(wp) >= 3 && wp(3) > 0
                target_speed = wp(3);
            end
        end
    catch
        % keep default target
    end

    % Longitudinal PID: brake overrides cruise target
    if strcmp(aeb_action, 'BRAKE')
        target_speed = 0.0;
    end

    integral = 0; previous_error = 0;
    for step = 1:10   % short horizon prototype
        error = target_speed - ego_speed;
        integral = integral + error * dt;
        derivative = (error - previous_error) / dt;
        accel = kp * error + ki * integral + kd * derivative;
        accel = max(-6, min(2, accel));   % actuator saturation
        ego_speed = ego_speed + accel * dt;
        previous_error = error;
    end

    commands = struct();
    commands.aeb_action = aeb_action;
    commands.target_speed = target_speed;
    commands.estimated_speed = ego_speed;
    commands.throttle_brake_accel = previous_error;   % last computed accel proxy
    commands.controller = 'PID';

    save('control_output.mat', 'commands');
    try
        write_json(commands, fullfile('..', '..', 'control_output.json'));
    catch
        % write_json helper not on path; ignore
    end
    fprintf('Control run complete (action=%s, target=%.1f m/s). Saved control_output.mat\n', ...
        aeb_action, target_speed);
end
