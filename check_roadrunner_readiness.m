function status=check_roadrunner_readiness(projectFolder)
% Report missing prerequisites explicitly; a MATLAB fallback is not RoadRunner integration.
if nargin==0, projectFolder=getenv('ROADRUNNER_PROJECT'); end
status=struct('matlabAPI',exist('roadrunner','file')==2, ...
    'automatedDrivingToolbox',~isempty(ver('driving')), ...
    'projectFolder',projectFolder,'projectExists',false,'scenarioExists',false,'ready',false);
if ~isempty(projectFolder) && isfolder(projectFolder)
    status.projectExists=~isempty(dir(fullfile(projectFolder,'**','*.rrproj')));
    status.scenarioExists=~isempty(dir(fullfile(projectFolder,'**','*.rrscenario')));
end
status.ready=status.matlabAPI && status.automatedDrivingToolbox && status.projectExists && status.scenarioExists;
fid=fopen('results/roadrunner_readiness.json','w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(status,PrettyPrint=true)); disp(status);
end
