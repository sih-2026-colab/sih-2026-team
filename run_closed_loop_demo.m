function evidence=run_closed_loop_demo()
% Real AutoNex planner/controller/plant, no demonstration planner.
evidence=struct([]);
for scenario={'clear_road','missing_lane','unavailable_corridor'}
    label=scenario{1}; base=label;
    if strcmp(label,'unavailable_corridor'), base='clear_road'; end
    opts=autonex_options(struct('scenario',base,'useLearnedIntent',false));
    state=[]; previous=[]; modes={}; switches=0; noPath=0; collisions=0; departures=0;
    for k=0:160
        t=k*opts.dt;
        if strcmp(label,'unavailable_corridor') && k>=10
            % Inject unavailable observed space, not a fabricated trajectory.
            % Freeze this fixture's invalid corridor until the ego stops.
            state.corridor.valid=false; state.mapTime=t;
        end
        [state,out]=autonex_step(state,t,opts);
        collisions=collisions+out.collision; departures=departures+out.boundaryViolation;
        assert(all(isfinite(struct2array(state.ego))));
        assert(state.ego.x==state.actors(1).x && state.ego.y==state.actors(1).y);
        assert(state.ego.yaw==state.egoYaw && state.ego.yaw==state.actors(1).heading);
        assert(abs(state.ego.speed-hypot(state.actors(1).vx,state.actors(1).vy))<1e-10);
        assert(state.ego.acceleration==state.actors(1).ax && state.ego.steering==state.steeringAngle);
        assert(abs(state.ego.steering)<=opts.maxSteeringAngle+1e-10);
        assert(state.ego.acceleration>=-opts.maxDeceleration && state.ego.acceleration<=opts.maxAcceleration);
        if ~isempty(previous)
            % Exact next-cycle feedback, not merely a fresh initial scenario.
            assert(out.x==previous.nextEgo.x && out.y==previous.nextEgo.y);
            assert(out.egoYaw==previous.nextEgo.yaw);
            assert(abs(out.speed/3.6-previous.nextEgo.speed)<1e-10);
            assert(abs(state.ego.steering-previous.nextEgo.steering)<=opts.maxSteeringRate*opts.dt+1e-10);
            assert(out.actors(2).x~=previous.actors(2).x);
            if ~strcmp(out.selectedName,previous.selectedName) && out.speed>1 && ~isempty(out.selected)
                switches=switches+1;
            end
        end
        if ~isempty(out.selected)
            p=out.selected.trajectory;
            assert(abs(p.x(1)-out.x)<1e-9 && abs(p.y(1)-out.y)<1e-9);
        end
        if strcmp(label,'unavailable_corridor') && k>=10
            assert(isempty(out.selected) && out.candidateCount==0 && out.emergency);
            assert(state.ego.speed<=out.speed/3.6+1e-10 && out.ax<=0);
            noPath=noPath+1;
        end
        if ~ismember(out.motionMode,modes)
            modes{end+1}=out.motionMode; %#ok<AGROW>
            fprintf('%s t=%.2f mode=%s x=%.3f y=%.3f speed=%.3f m/s ax=%.3f\n', ...
                label,t,out.motionMode,out.x,out.y,out.speed/3.6,out.ax);
        end
        previous=out;
    end
    if strcmp(label,'clear_road'), assert(state.ego.x>100 && ismember('CRUISE',modes)); end
    if strcmp(label,'missing_lane'), assert(switches>0 && abs(state.ego.y-7)>.5); end
    if strcmp(label,'unavailable_corridor')
        assert(state.ego.speed==0 && noPath==151 && ismember('BRAKE',modes) && ismember('STOP',modes));
    end
    row=struct('scenario',label,'steps',161,'x',state.ego.x,'y',state.ego.y, ...
        'speed',state.ego.speed,'movingSwitches',switches,'noPathSteps',noPath, ...
        'collisionSteps',collisions,'boundarySteps',departures,'modes',{modes});
    if isempty(evidence), evidence=row; else, evidence(end+1)=row; end %#ok<AGROW>
    fprintf('%s MULTISTEP_PASS steps=161 x=%.3f y=%.3f speed=%.3f moving_switches=%d feedback=PASS\n', ...
        label,row.x,row.y,row.speed,switches);
    fprintf('  safety_observations collision_steps=%d boundary_steps=%d\n',collisions,departures);
end
fprintf('F G H REPLAN_NO_SAFE_PATH_STATE_FEEDBACK_PASS\n');
end
