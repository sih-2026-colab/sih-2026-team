function simulation=run_autonex_roadrunner(projectFolder,scenarioFile)
% Launch a real, prepared RoadRunner scenario. No silent MATLAB fallback.
% This launcher does not install software or create the required actor behavior.
assert(nargin==2,'Supply a RoadRunner project folder and native .rrscenario file.');
assert(exist('roadrunner','file')==2 && ~isempty(ver('driving')), ...
    'AutoNex:RoadRunnerUnavailable', ...
    'RoadRunner API and Automated Driving Toolbox must be installed and licensed.');
assert(isfolder(projectFolder),'RoadRunner project folder does not exist.');
assert(isfile(scenarioFile),'Native RoadRunner scenario file does not exist.');
app=roadrunner(projectFolder);
openScenario(app,scenarioFile);
simulation=createSimulation(app);
set(simulation,'PacingRate',1);
set(simulation,'SimulationCommand','Start');
end
