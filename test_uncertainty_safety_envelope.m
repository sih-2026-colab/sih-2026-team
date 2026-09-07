clear;
clc;


fprintf('\n');
fprintf('====================================================\n');
fprintf(' AutoNex Uncertainty-Aware Safety Bubble\n');
fprintf('====================================================\n\n');


%% =========================================================
% DAY EXAMPLE
%% =========================================================

dayConfidence = 0.86;

dayUncertainty = 0.50;

relativeSpeed = 3.0;


dayEnvelope = ...
    calculate_uncertainty_safety_envelope( ...
        dayUncertainty, ...
        dayConfidence, ...
        relativeSpeed);


%% =========================================================
% NIGHT EXAMPLE
%% =========================================================

nightConfidence = 0.73;

nightUncertainty = 0.95;


nightEnvelope = ...
    calculate_uncertainty_safety_envelope( ...
        nightUncertainty, ...
        nightConfidence, ...
        relativeSpeed);


%% =========================================================
% PRINT
%% =========================================================

fprintf('DAY:\n');

fprintf( ...
    'Confidence      : %.1f%%\n', ...
    dayConfidence * 100);

fprintf( ...
    'Uncertainty     : %.2f\n', ...
    dayUncertainty);

fprintf( ...
    'Longitudinal    : %.2f m\n', ...
    dayEnvelope.longitudinal);

fprintf( ...
    'Lateral         : %.2f m\n\n', ...
    dayEnvelope.lateral);


fprintf('NIGHT:\n');

fprintf( ...
    'Confidence      : %.1f%%\n', ...
    nightConfidence * 100);

fprintf( ...
    'Uncertainty     : %.2f\n', ...
    nightUncertainty);

fprintf( ...
    'Longitudinal    : %.2f m\n', ...
    nightEnvelope.longitudinal);

fprintf( ...
    'Lateral         : %.2f m\n', ...
    nightEnvelope.lateral);