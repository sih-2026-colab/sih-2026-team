function results=validate_multisensor_acceptance()
% Individually counted assertions; no safety claims from an exception-free run.
results=struct('name',{},'passed',{},'exception',{});
masks=[1 0 0;0 1 0;0 0 1;1 1 0;1 0 1;0 1 1;1 1 1];
names={'camera','radar','lidar','camera_radar','camera_lidar','radar_lidar','all_three'};
for k=1:7, check(['sources_' names{k}],@()combination(masks(k,:))); end
classes={'car','bus','bike','auto-rickshaw','pedestrian','animal','pushcart','obstacle','unknown'};
for k=1:numel(classes), check(['class_' classes{k}],@()class_case(classes{k})); end
check('geometry_yaw',@()geometry(false,false));
check('geometry_offsets',@()geometry(true,false));
check('geometry_turning_offsets',@()geometry(true,true));
check('doppler_observability',@doppler);
check('nearby_noisy_lidar',@nearby);
for k=1:4, check(sprintf('dropout_recovery_%d',k),@()dropout(k)); end
check('quality_uncertainty',@quality);
for scenario={'cut_in','missing_lane','tracking_loss'}
    check(['closed_loop_' scenario{1}],@()closed_loop(scenario{1}));
end
fprintf('MULTISENSOR_ACCEPTANCE_TOTAL passed=%d failed=%d exceptions=%d total=%d\n', ...
    sum([results.passed]),sum(~[results.passed]),sum(~cellfun(@isempty,{results.exception})),numel(results));
    function check(name,job)
        try
            rng(61); job(); row=struct('name',name,'passed',true,'exception','');
            fprintf('ACCEPTANCE_PASS %s\n',name);
        catch err
            row=struct('name',name,'passed',false,'exception',getReport(err,'extended','hyperlinks','off'));
            fprintf('ACCEPTANCE_FAIL %s: %s\n',name,row.exception);
        end
        results(end+1)=row;
    end
end
function [a,e]=fixture()
a=create_highway_scenario(); a=a(1:4); a(1).x=0; a(1).y=3.5; a(1).vx=5; a(1).vy=0;
for k=2:4, a(k).x=20*(k-1); a(k).y=1+2*(k-2); a(k).vx=8; a(k).vy=0; a(k).type='car'; end
e=struct('lightLevel',1,'visibility',1,'radarQuality',.95,'thermalQuality',.85);
end
function a=advance(a,dt)
for k=1:numel(a), a(k).x=a(k).x+a(k).vx*dt; a(k).y=a(k).y+a(k).vy*dt; end
end
function combination(mask)
[a,e]=fixture(); memory=[];
for step=0:7
    frame=autonex_sensor_frame(a,e,step*.05,mask);
    [reports,groups]=fuse_autonex_detections(frame);
    assert(numel(reports)==3 && numel(groups)==3,'Duplicate or missing per-object report');
    [memory,tr,f,w]=update_autonex_world_model(memory,frame,'IMM'); a=advance(a,.05);
end
assert(numel(w)==3 && numel(tr)==3 && numel(f)==3);
expected={'camera','radar','lidar'}; expected=expected(logical(mask));
for k=1:3
    assert(isequal(w(k).sensorSources,expected),'Incorrect current sensor provenance');
    assert(all(isfinite([w(k).x w(k).y w(k).vx w(k).vy w(k).heading w(k).confidence w(k).uncertainty])));
end
assert(numel(unique([w.id]))==3);
end
function class_case(label)
[a,e]=fixture(); a=a(1:2); a(2).type=label; memory=[];
for t=[0 .05 .1]
    [memory,~,~,w]=update_autonex_world_model(memory,autonex_sensor_frame(a,e,t,[1 1 1]),'CV');
end
assert(numel(w)==1 && strcmp(w.class,autonex_object_class(label)));
priors=struct('car',[4.5 1.9],'bus',[10 2.5],'bike',[2 .8], ...
    'auto_rickshaw',[2.6 1.4],'pedestrian',[.6 .6],'animal',[2 .8], ...
    'pushcart',[1.8 1],'obstacle',[1 1],'unknown',[1 1]);
[L,W,source]=autonex_perception_dimensions(a(2));
assert(isequal([L W],priors.(strrep(label,'-','_'))) && ~isempty(source));
b=a(2); b.length=3.2; b.width=1.3;
[L,W,source]=autonex_perception_dimensions(b);
assert(isequal([L W],[3.2 1.3]) && strcmp(source,'explicit'));
end
function geometry(offsets,turning)
[a,e]=fixture(); a=a(1:2); a(2).x=15; a(2).y=5; a(2).vx=0; a(1).vx=3;
c=autonex_sensor_config();
if offsets
    c.cameraPose=[1 .2 .1]; c.radarPoses(:,1:2)=repmat([1.3 .4],4,1);
    c.radarPoses(:,3)=c.radarPoses(:,3)+.1; c.lidarPose=[.8 -.2 .3];
end
opts=autonex_options(); yaw=.3;
for step=1:10
    a(1).heading=yaw; a(1).vx=3*cos(yaw); a(1).vy=3*sin(yaw);
    c.yawRate=0; if turning, c.yawRate=3/opts.wheelbase*tan(.12); end
    frame=autonex_sensor_frame(a,e,step*.05,[1 1 1],c);
    assert(numel(frame.camera)==1 && hypot(frame.camera.x-a(2).x,frame.camera.y-a(2).y)<1.5);
    assert(~isempty(frame.lidar));
    assert(min(hypot([frame.lidar.x]-a(2).x,[frame.lidar.y]-a(2).y))<2.6);
    d=frame.radar(find(strcmp({frame.radar.sensor},'FRONT_RADAR'),1));
    E=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)]; offset=E*c.radarPoses(1,1:2)';
    assert(norm(d.sensorOrigin-([a(1).x;a(1).y]+offset))<1e-10);
    expectedV=[a(1).vx;a(1).vy]+c.yawRate*[-offset(2);offset(1)];
    assert(norm(d.sensorVelocity-expectedV)<1e-10);
    assert(hypot(d.x-a(2).x,d.y-a(2).y)<.8);
    if turning, [a(1),yaw]=update_ego_bicycle(a(1),yaw,0,.12,opts); else, yaw=yaw+.02; end
end
end
function doppler()
[a,e]=fixture(); a=a(1:2); a(2).y=a(1).y;
frame=autonex_sensor_frame(a,e,0,[0 1 0]);
d=frame.radar(1); u=d.lineOfSight; v=[-u(2);u(1)];
assert(abs(d.radialVelocity-3)<.6);
assert(abs(dot(d.velocity,u)-8)<.6);
assert(v'*d.velocityCovariance*v>1e3 && u'*d.velocityCovariance*u<.1);
[reports,~]=fuse_autonex_detections(frame);
assert(numel(reports{1}.Measurement)==6 && abs(reports{1}.Measurement(4)-8)<.6);
[~,tr,~,w]=update_autonex_world_model([],frame,'IMM');
% First frame may be tentative; establish track with the second frame.
memory=[];
for t=[0 .05 .1]
    [memory,tr,~,w]=update_autonex_world_model(memory,autonex_sensor_frame(a,e,t,[0 1 0]),'IMM');
    a=advance(a,.05);
end
assert(numel(w)==1 && abs(w.vx-8)<1);
assert(tr.StateCovariance(4,4)>tr.StateCovariance(2,2),'Unobserved lateral velocity overconfident');
end
function nearby()
[a,e]=fixture(); a=a(1:3); a(2).x=18; a(3).x=18; a(2).y=1; a(3).y=4.5;
c=autonex_sensor_config(struct('lidarNoise',.03)); memory=[];
for step=0:7
    frame=autonex_sensor_frame(a,e,step*.05,[0 0 1],c);
    assert(numel(frame.lidar)==2,'Separated nearby boxes merged or noise split an object');
    [memory,~,~,w]=update_autonex_world_model(memory,frame,'IMM');
end
assert(numel(w)==2 && numel(unique([w.id]))==2);
end
function dropout(which)
[a,e]=fixture(); memory=[];
for step=0:9
    [memory,~,~,w]=update_autonex_world_model(memory,autonex_sensor_frame(a,e,step*.05,[1 1 1]),'IMM');
    a=advance(a,.05);
end
ids=[w.id]; previous=[w.uncertainty]; mask=[1 1 1];
if which==4, mask(:)=0; else, mask(which)=0; end
for step=10:12
    [memory,tr,f,w]=update_autonex_world_model(memory,autonex_sensor_frame(a,e,step*.05,mask),'IMM');
    assert(isequal([w.id],ids) && numel(f)==numel(tr));
    assert(all([w.uncertainty]>previous),'Lost modality did not increase uncertainty');
    expected={'camera','radar','lidar'}; expected=expected(logical(mask));
    for j=1:numel(w)
        if isempty(expected), assert(isempty(w(j).sensorSources));
        else, assert(isequal(w(j).sensorSources,expected)); end
    end
    a=advance(a,.05);
end
for step=13:15
    [memory,~,~,w]=update_autonex_world_model(memory,autonex_sensor_frame(a,e,step*.05,[1 1 1]),'IMM');
    a=advance(a,.05);
end
assert(isequal([w.id],ids) && ~any([w.coasted]));
end
function quality()
[a,e]=fixture(); normal=[]; degraded=[];
for step=0:7
    good=autonex_sensor_frame(a,e,step*.05,[1 1 1]); bad=good;
    for name={'camera','radar','lidar'}
        for k=1:numel(bad.(name{1}))
            bad.(name{1})(k).covariance=4*bad.(name{1})(k).covariance;
            bad.(name{1})(k).confidence=.5*bad.(name{1})(k).confidence;
        end
    end
    [normal,~,~,w]=update_autonex_world_model(normal,good,'IMM');
    [degraded,~,~,b]=update_autonex_world_model(degraded,bad,'IMM');
    a=advance(a,.05);
end
assert(numel(w)==3 && numel(b)==3);
assert(all([b.uncertainty]>[w.uncertainty]) && all([b.confidence]<=[w.confidence]));
end
function closed_loop(name)
opts=autonex_options(struct('scenario',name,'perceptionMode','camera_radar_lidar','useLearnedIntent',false));
state=[]; previous=[]; observed=0; candidateSteps=0; brake=0; collisions=0; boundary=0;
for step=0:80
    [state,out]=autonex_step(state,step*opts.dt,opts);
    observed=observed+numel(out.worldModel); candidateSteps=candidateSteps+(out.candidateCount>0);
    brake=brake+(out.ax<0); collisions=collisions+out.collision; boundary=boundary+out.boundaryViolation;
    assert(numel(out.worldModel)==numel(out.tracks));
    if ~isempty(previous), assert(out.x==previous.x && out.y==previous.y); end
    previous=out.nextEgo;
    assert(abs(out.steeringRate)<=opts.maxSteeringRate+1e-9 && out.ax>=-opts.maxDeceleration);
end
assert(observed>0 && candidateSteps>0 && state.ego.x>5 && collisions==0 && boundary==0);
if strcmp(name,'tracking_loss'), assert(brake>0); end
fprintf('FUSION_LOOP %s steps=81 x=%.3f candidateSteps=%d brakeSteps=%d collision=%d boundary=%d\n', ...
    name,state.ego.x,candidateSteps,brake,collisions,boundary);
end
