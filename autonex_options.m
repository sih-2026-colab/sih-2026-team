function opts=autonex_options(opts)
if nargin==0, opts=struct; end
defaults=struct('seed',60,'dt',.05,'duration',6,'scenario','cut_in', ...
    'laneIndependent',true,'visualize',false,'exportVideo',false, ...
    'videoFile','autonex_safe_space_animation.mp4','reportFile','');
names=fieldnames(defaults);
for k=1:numel(names)
    if ~isfield(opts,names{k}), opts.(names{k})=defaults.(names{k}); end
end
end
