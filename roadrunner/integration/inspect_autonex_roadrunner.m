function report=inspect_autonex_roadrunner()
% Non-launching, non-mutating availability and asset inspection.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
products=ver;
report=struct('matlabVersion',version,'entryPoint',which('roadrunner'), ...
    'setupEntryPoint',which('roadrunnerSetup'),'products',{{products.Name}}, ...
    'nativeSceneVerified',false,'sceneLaunch','NOT ATTEMPTED: native assets not verified', ...
    'assets',struct([]));
for name={'Scenes/VillageRoad.rrscene','Scenarios/VillageBasic.rrscenario'}
    path=fullfile(root,'roadrunner',name{1}); info=dir(path);
    a=struct('path',name{1},'exists',isfile(path),'bytes',0,'format','missing','description',struct);
    if a.exists
        a.bytes=info.bytes;
        try
            a.description=jsondecode(fileread(path)); a.format='JSON description, not a verified native RoadRunner asset';
        catch
            a.format='Native/binary candidate; requires RoadRunner verification';
        end
    end
    if isempty(report.assets), report.assets=a; else, report.assets(end+1)=a; end %#ok<AGROW>
end
report.matlabConnectionAvailable=~isempty(report.entryPoint);
if ~report.matlabConnectionAvailable, report.sceneLaunch='UNAVAILABLE: MATLAB roadrunner entry point absent'; end
fid=fopen(fullfile(root,'results','judge_p3_roadrunner.json'),'w');
assert(fid>=0); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));
disp(report.sceneLaunch);
end
