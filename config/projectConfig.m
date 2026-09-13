function projectConfig()
%PROJECTCONFIG Validate and expose the SIH project directory layout.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
requiredDirectories = {
    fullfile(projectRoot, 'matlab'), ...
    fullfile(projectRoot, 'simulink'), ...
    fullfile(projectRoot, 'roadrunner'), ...
    fullfile(projectRoot, 'tests')};

assert(all(cellfun(@isfolder, requiredDirectories)), ...
    'SIH:InvalidProjectLayout', 'One or more required project folders are missing.');
assignin('base', 'SIH_PROJECT_ROOT', projectRoot);
disp('Project configuration loaded.');
end
