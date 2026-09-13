function [reports,groups]=fuse_autonex_detections(frame)
% One-to-one association within each sensor, before ONE shared GNN tracker.
% Conservative covariance intersection avoids counting overlapping radars as
% independent certainty. No ground-truth identity or class is used as a gate.
groups=struct([]); reports=cell(0,1);
radarNames={}; if ~isempty(frame.radar), radarNames=unique({frame.radar.sensor},'stable'); end
sources=[radarNames {'CAMERA','LIDAR'}];
for s=1:numel(sources)
    source=sources{s};
    if strcmp(source,'CAMERA'), detections=frame.camera;
    elseif strcmp(source,'LIDAR'), detections=frame.lidar;
    else, detections=frame.radar(strcmp({frame.radar.sensor},source)); end
    used=false(1,numel(groups));
    % Assign nearest globally first, so detection enumeration is not priority.
    pairs=zeros(0,3);
    for j=1:numel(detections)
        p=[detections(j).x;detections(j).y];
        for q=1:numel(groups)
            d=p-groups(q).position; S=detections(j).covariance+groups(q).covariance;
            cost=d'*(S\d);
            gate=3.5;
            % Depth observes an exposed surface, not necessarily the centre.
            % Use visual class size where known; unknown depth retains a
            % conservative bus-length allowance rather than duplicating it.
            if strcmp(source,'LIDAR')
                [L,~]=autonex_perception_dimensions(struct('type',groups(q).class));
                if strcmp(groups(q).class,'unknown'), L=10; end
                gate=max(gate,L/2+1);
            end
            if norm(d)<=gate && cost<=16, pairs(end+1,:)=[cost j q]; end %#ok<AGROW>
        end
    end
    pairs=sortrows(pairs,1); assignment=zeros(1,numel(detections));
    for row=1:size(pairs,1)
        j=pairs(row,2); q=pairs(row,3);
        if assignment(j)==0 && ~used(q), assignment(j)=q; used(q)=true; end
    end
    for j=1:numel(detections)
        d=detections(j); p=[d.x;d.y]; q=assignment(j);
        if q==0
            g=struct('position',p,'covariance',d.covariance,'velocity',[0;0], ...
                'velocityCovariance',1e4*eye(2),'sources',{{}},'class','unknown', ...
                'confidence',0,'cameraConfidence',NaN,'lidarConfidence',NaN,'radarConfidence',NaN,'time',frame.time);
            if isempty(groups), groups=g; else, groups(end+1)=g; end %#ok<AGROW>
            q=numel(groups);
        else
            A=groups(q).covariance; B=d.covariance;
            P=inv(.5*inv(A)+.5*inv(B));
            groups(q).position=P*(.5*(A\groups(q).position)+.5*(B\p));
            groups(q).covariance=P;
        end
        groups(q).sources{end+1}=source;
        groups(q).confidence=max(groups(q).confidence,d.confidence);
        if strcmp(source,'CAMERA')
            groups(q).class=autonex_object_class(d.type); groups(q).cameraConfidence=d.confidence;
        elseif strcmp(source,'LIDAR'), groups(q).lidarConfidence=d.confidence;
        else
            if isnan(groups(q).radarConfidence)
                groups(q).velocity=d.velocity; groups(q).velocityCovariance=d.velocityCovariance;
            else
                A=groups(q).velocityCovariance; B=d.velocityCovariance;
                V=inv(.5*inv(A)+.5*inv(B));
                groups(q).velocity=V*(.5*(A\groups(q).velocity)+.5*(B\d.velocity));
                groups(q).velocityCovariance=V;
            end
            groups(q).radarConfidence=d.confidence;
        end
    end
end
parameters=struct('Frame','Rectangular','HasVelocity',true);
for k=1:numel(groups)
    g=groups(k); P=blkdiag(g.covariance,.01,g.velocityCovariance,.01);
    reports{end+1,1}=objectDetection(frame.time,[g.position;0;g.velocity;0], ...
        'MeasurementNoise',P,'MeasurementParameters',parameters, ...
        'SensorIndex',1,'ObjectAttributes',{g});
end
end
