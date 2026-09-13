function rows=audit_rear_candidate_gates(ego,tracks,fused,corridor,drivable,safeSpace,opts)
rows=struct([]); if ~corridor.valid, return; end
x=min(corridor.x(end),ego.x+max(8,ego.vx*1.2));
center=interp1(corridor.x,corridor.referenceY,x);
lo=interp1(corridor.x,corridor.lowerY,x); hi=interp1(corridor.x,corridor.upperY,x);
targets=unique([center max(lo,center-.45) min(hi,center+.45) linspace(lo,hi,5)]);
speeds=unique([75 73 70 65 45 20 5 0 max(0,ego.vx*3.6)]);
frontMask=arrayfun(@(q)q.State(1)>=ego.x-2.25,tracks);
for y=targets
    for speed=speeds
        T=max(2.5,sqrt(6*abs(y-ego.y)/2.5));
        tr=generate_2d_trajectory(ego,speed,y,0:.1:opts.rearActionHorizon,T);
        c=struct('id',1,'name','AUDIT','targetSpeedKmh',speed,'targetY',y,'maneuverTime',T,'trajectory',tr);
        r=evaluate_2d_trajectories(c,tracks(frontMask),fused(frontMask),ego,[],-1.75,8.75);
        [~,~,~,g]=cat_reflex_2d_guardian(c,r,[],NaN,tracks(frontMask),-1.75,8.75);
        row=struct('targetY',y,'speed',speed, ...
            'occupancySafe',trajectory_in_safe_space(tr,drivable,safeSpace,ego), ...
            'frontSafe',r.safe,'guardianSafe',g.safe,'frontDetails',r,'guardianDetails',g);
        if isempty(rows), rows=row; else, rows(end+1)=row; end %#ok<AGROW>
    end
end
end
