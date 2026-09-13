%% Startup script for SIH Autonomous Driving project

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(genpath(fullfile(projectRoot, 'matlab')));
addpath(fullfile(projectRoot, 'tests'));

projectConfig();
clear projectRoot;
disp('Startup complete.');
