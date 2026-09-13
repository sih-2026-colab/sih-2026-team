function test_multisensor_world_model()
environment=struct('lightLevel',1,'visibility',1,'radarQuality',.95,'thermalQuality',.85);
actors=create_highway_scenario(); actors=actors(1:4);
actors(1).x=0; actors(1).y=3.5; actors(1).vx=5; actors(1).vy=0;
for k=2:4
    actors(k).x=10*k; actors(k).y=1+2*(k-2); actors(k).vx=8; actors(k).vy=0;
    actors(k).type='car';
end
combinations=[1 0 0;0 1 0;0 0 1;1 1 0;1 0 1;1 1 1];
names={'CAMERA_ONLY','RADAR_ONLY','LIDAR_ONLY','CAMERA_RADAR','CAMERA_LIDAR','ALL_THREE'};
for c=1:6
    memory=[]; rng(61); scene=actors;
    for step=0:7
        frame=autonex_sensor_frame(scene,environment,step*.05,combinations(c,:));
        [memory,tracks,fused,world]=update_autonex_world_model(memory,frame,'IMM');
        for k=1:numel(scene), scene(k).x=scene(k).x+scene(k).vx*.05; end
    end
    assert(~isempty(world),'No objects tracked');
    assert(numel(world)==numel(tracks) && numel(world)==numel(fused));
    assert(numel(world)<=numel(scene)-1,'Duplicate object tracks');
    assert(numel(unique([world.id]))==numel(world));
    assert(all(isfinite([world.x world.y world.vx world.vy world.confidence world.uncertainty])));
    if c~=3, assert(numel(world)==3,'Simultaneous actors missing'); end
    if combinations(c,1), assert(all(strcmp({world.class},'car'))); end
    if combinations(c,2), assert(max(abs([world.vx]-8))<2,'Radar Doppler velocity not used'); end
    fprintf('FUSION_%s_PASS tracks=%d\n',names{c},numel(world));
end
% Exact common observations prove one report per physical object, not sensor.
frame=autonex_sensor_frame(actors,environment,0,[1 1 1]);
[reports,groups]=fuse_autonex_detections(frame);
assert(numel(reports)==3 && numel(groups)==3,'Cross-sensor duplicate birth');
assert(all(arrayfun(@(g)numel(unique(g.sources))==numel(g.sources),groups)));
% Drop all sensors briefly: coast identity, clear active provenance, inflate uncertainty.
ids=[world.id]; priorUncertainty=[world.uncertainty];
empty=struct('time',.4,'camera',struct([]),'radar',struct([]),'lidar',struct([]));
for step=8:10
    empty.time=step*.05;
    [memory,~,~,world]=update_autonex_world_model(memory,empty,'IMM');
end
assert(isequal([world.id],ids) && all([world.coasted]));
assert(all(cellfun(@isempty,{world.sensorSources})));
assert(all([world.uncertainty]>priorUncertainty));
for k=1:numel(scene), scene(k).x=scene(k).x+scene(k).vx*.15; end
frame=autonex_sensor_frame(scene,environment,.55,[1 1 1]);
[memory,~,~,world]=update_autonex_world_model(memory,frame,'IMM'); %#ok<ASGLU>
assert(isequal([world.id],ids) && ~any([world.coasted]));
fprintf('DUPLICATE_PREVENTION_DROPOUT_RECOVERY_PASS\n');
% Classification vocabulary is separate from ground-truth tracking identity.
labels={'car','bus','bike','auto','pedestrian','animal','pushcart','obstacle','other'};
expected={'car','bus','two-wheeler','auto-rickshaw','pedestrian','animal','pushcart','static obstacle','unknown'};
assert(isequal(cellfun(@autonex_object_class,labels,'UniformOutput',false),expected));
for k=1:numel(labels)
    scene=actors(1:2); scene(2).type=labels{k}; memory=[];
    for t=[0 .05 .1]
        frame=autonex_sensor_frame(scene,environment,t,[1 0 0]);
        [memory,~,~,w]=update_autonex_world_model(memory,frame,'CV');
    end
    assert(numel(w)==1 && strcmp(w.class,expected{k}),'Camera class propagation');
end
% A 90-degree heading must rotate camera/radar field of view, not world truth.
turned=actors(1:2); turned(1).heading=pi/2; turned(2).x=0; turned(2).y=23.5;
frame=autonex_sensor_frame(turned,environment,0,[1 1 0]);
assert(numel(frame.camera)==1 && abs(frame.camera.x)<2);
assert(any(strcmp({frame.radar.sensor},'FRONT_RADAR')));
fprintf('CLASS_VOCABULARY_EGO_HEADING_PASS\n');
% Feed fused GNN tracks directly to the unchanged planner/controller loop.
opts=autonex_options(struct('perceptionMode','camera_radar_lidar','scenario','cut_in','useLearnedIntent',false));
state=[]; count=0; previous=[];
for step=0:40
    [state,out]=autonex_step(state,step*opts.dt,opts);
    count=count+numel(out.worldModel);
    assert(numel(out.worldModel)==numel(out.tracks));
    if ~isempty(previous), assert(out.x==previous.x && out.y==previous.y); end
    assert(abs(out.steeringRate)<=opts.maxSteeringRate+1e-9);
    previous=out.nextEgo;
end
assert(count>0 && state.ego.x>5);
fprintf('FUSED_TRACKS_CLOSED_LOOP_PASS steps=41 x=%.3f tracks_observed=%d\n',state.ego.x,count);
fprintf('MULTISENSOR_WORLD_MODEL_ALL_PASS\n');
end
