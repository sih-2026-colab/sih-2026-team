function preview_aeb()
%PREVIEW_AEB Side-by-side MATLAB figure of AEB off vs on (same as HTML preview).
    thisDir = fileparts(mfilename('fullpath'));
    addpath(thisDir);
    addpath(fullfile(thisDir, '..', 'control'));
    addpath(fullfile(thisDir, '..', 'scenarios'));
    addpath(fullfile(thisDir, '..', 'utils'));

    off = simulate_aeb(false);
    on = simulate_aeb(true);

    fig = figure('Name', 'SIH 2026 Urban Intersection AEB', 'Color', [0.06 0.07 0.09]);
    subplot(1,2,1); hold on; title('AEB OFF', 'Color', [0.9 0.9 0.9]);
    plot([off.log.t], [off.log.ego_x], 'Color', [0.83 0.40 0.36], 'LineWidth', 1.5);
    xlabel('t (s)'); ylabel('ego x (m)');
    grid on;
    subplot(1,2,2); hold on; title('AEB ON', 'Color', [0.9 0.9 0.9]);
    plot([on.log.t], [on.log.ego_x], 'Color', [0.37 0.70 0.48], 'LineWidth', 1.5);
    xlabel('t (s)'); ylabel('ego x (m)');
    grid on;

    report = struct('aeb_off', rmfield(off, 'log'), 'aeb_on', rmfield(on, 'log'));
    outDir = fullfile(thisDir, '..', '..', 'results');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    write_json(report, fullfile(outDir, 'aeb_preview_report.json'));
    fprintf('AEB OFF collision=%d (%s)\n', report.aeb_off.collision, report.aeb_off.collision_with);
    fprintf('AEB ON  collision=%d  final_x=%.2f m  speed=%.2f m/s\n', ...
        report.aeb_on.collision, report.aeb_on.final_x, report.aeb_on.final_speed);

    if nargout == 0 && ~isempty(fig)
        % keep figure open
    end
end
