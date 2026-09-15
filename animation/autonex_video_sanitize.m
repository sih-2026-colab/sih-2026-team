function [value,issues]=autonex_video_sanitize(value,path)
% Preserve finite values; explicitly record every NaN/+Inf/-Inf location.
% Nonfinite scalar -> []; mixed arrays -> same-sized cell array with [] holes.
if nargin<2, path='$'; end
issues=struct('path',{},'kind',{});
if isnumeric(value)
    bad=find(~isfinite(value));
    if isempty(bad), return; end
    for k=1:numel(bad)
        n=value(bad(k)); kind='NaN';
        if isinf(n) && n>0, kind='+Inf'; elseif isinf(n), kind='-Inf'; end
        issues(end+1)=struct('path',sprintf('%s(%d)',path,bad(k)),'kind',kind); %#ok<AGROW>
    end
    if isscalar(value), value=[];
    else
        value=num2cell(value); value(bad)={[]};
    end
elseif isstruct(value)
    names=fieldnames(value);
    for k=1:numel(value)
        for j=1:numel(names)
            [value(k).(names{j}),more]=autonex_video_sanitize(value(k).(names{j}), ...
                sprintf('%s(%d).%s',path,k,names{j}));
            issues=[issues more]; %#ok<AGROW>
        end
    end
elseif iscell(value)
    for k=1:numel(value)
        [value{k},more]=autonex_video_sanitize(value{k},sprintf('%s{%d}',path,k));
        issues=[issues more]; %#ok<AGROW>
    end
elseif ~(ischar(value) || isstring(value) || islogical(value))
    error('AutoNex:ExportType','Unsupported telemetry type %s at %s',class(value),path);
end
end
