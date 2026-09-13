function results=run_motion_regressions()
% Execute every existing test_*.m entry point, including legacy test scripts.
root=fileparts(fileparts(mfilename('fullpath')));
files=[dir(fullfile(root,'test_*.m')); dir(fullfile(root,'tests','test*.m'))];
results=struct('name',{},'passed',{},'message',{});
set(groot,'defaultFigureVisible','off');
for k=1:numel(files)
    [~,name]=fileparts(files(k).name);
    try
        evalin('base',[name ';']);
        row=struct('name',name,'passed',true,'message','');
        fprintf('REGRESSION_PASS %s\n',name);
    catch err
        row=struct('name',name,'passed',false,'message',getReport(err,'extended','hyperlinks','off'));
        fprintf('REGRESSION_FAIL %s: %s\n',name,err.message);
    end
    close all force;
    results(end+1)=row; %#ok<AGROW>
end
fprintf('REGRESSION_TOTAL passed=%d failed=%d total=%d\n',sum([results.passed]),sum(~[results.passed]),numel(results));
disp(struct2table(results));
end
