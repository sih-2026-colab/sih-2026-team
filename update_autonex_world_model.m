function [memory,tracks,fused,world]=update_autonex_world_model(memory,frame,model)
% GNN owns identity, filter state, covariance, confirmation and deletion.
if isempty(memory)
    if strcmpi(model,'CV'), model='CV_EKF'; end
    memory=struct('tracker',create_autonex_gnn_tracker(model),'initialized',false,'labels',struct([]));
end
[reports,~]=fuse_autonex_detections(frame); tracks=[]; fused=struct([]); world=struct([]);
if isempty(reports) && ~memory.initialized, return; end
[tracks,~,~]=memory.tracker(reports,frame.time); memory.initialized=true;
if isempty(tracks), memory.labels=struct([]); return; end
for k=1:numel(tracks)
    tr=tracks(k); g=tr.ObjectAttributes;
    if iscell(g), g=g{1}; end
    age=max(0,frame.time-g.time);
    label=g.class; known=[];
    if ~isempty(memory.labels), known=find([memory.labels.id]==tr.TrackID,1); end
    if ~strcmp(label,'unknown')
        entry=struct('id',tr.TrackID,'class',label);
        if isempty(known)
            if isempty(memory.labels), memory.labels=entry; else, memory.labels(end+1)=entry; end
        else, memory.labels(known)=entry; end
    elseif ~isempty(known), label=memory.labels(known).class; end
    sources=g.sources;
    if age>1e-8
        sources={}; g.cameraConfidence=NaN; g.radarConfidence=NaN; g.lidarConfidence=NaN;
    end
    s=tr.State; modalities=~isnan([g.cameraConfidence g.radarConfidence g.lidarConfidence]);
    confidence=g.confidence*exp(-age/.5);
    uncertainty=sqrt(max(0,tr.StateCovariance(1,1)+tr.StateCovariance(3,3)))+ ...
        .3*(3-sum(modalities))+age;
    devices=sources; sources={};
    if any(strcmp(devices,'CAMERA')), sources{end+1}='camera'; end
    if any(contains(devices,'RADAR')), sources{end+1}='radar'; end
    if any(strcmp(devices,'LIDAR')), sources{end+1}='lidar'; end
    w=struct('id',tr.TrackID,'class',label,'x',s(1),'y',s(3),'vx',s(2),'vy',s(4), ...
        'heading',atan2(s(4),s(2)),'confidence',confidence,'uncertainty',uncertainty, ...
        'sensorSources',{sources},'sensorDevices',{devices},'lastMeasurementTime',g.time,'coasted',age>1e-8);
    f=struct('trackID',tr.TrackID,'fusedConfidence',confidence,'fusedUncertainty',uncertainty, ...
        'radarConfidence',g.radarConfidence,'rgbConfidence',g.cameraConfidence, ...
        'thermalConfidence',NaN,'lidarConfidence',g.lidarConfidence,'numSensors',sum(modalities));
    if isempty(world), world=w; fused=f; else, world(end+1)=w; fused(end+1)=f; end %#ok<AGROW>
end
% Do not retain class history for tracks that have been deleted by GNN.
if ~isempty(memory.labels)
    memory.labels=memory.labels(ismember([memory.labels.id],[tracks.TrackID]));
end
end
