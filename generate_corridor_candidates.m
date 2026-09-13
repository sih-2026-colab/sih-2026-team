function candidates = generate_corridor_candidates(ego,horizons,corridor,drivable,safeSpace,expandedSearch)
% Generate targets from connected free-space geometry, not fixed lane positions.
candidates=struct([]);
if nargin<6, expandedSearch=false; end
if ~corridor.valid, return; end
lookX=min(corridor.x(end),ego.x+max(8,ego.vx*1.2));
center=interp1(corridor.x,corridor.referenceY,lookX);
low=interp1(corridor.x,corridor.lowerY,lookX);
high=interp1(corridor.x,corridor.upperY,lookX);
targets=unique([center max(low,center-.45) min(high,center+.45)]);
if expandedSearch, targets=unique([targets linspace(low,high,5)]); end
speeds=unique([75 73 70 65 45 20 0 max(0,ego.vx*3.6)]);
for target=targets
    for speed=speeds
        maneuverTime=max(2.5,sqrt(6*abs(target-ego.y)/2.5));
        tr=generate_2d_trajectory(ego,speed,target,horizons,maneuverTime);
        if ~trajectory_in_safe_space(tr,drivable,safeSpace,ego), continue; end
        k=numel(candidates)+1;
        candidate=struct('id',k,'name',sprintf('CORRIDOR_%.2f',target), ...
            'targetSpeedKmh',speed,'targetY',target,'maneuverTime',maneuverTime,'trajectory',tr);
        if isempty(candidates), candidates=candidate; else, candidates(k)=candidate; end
    end
end
end
