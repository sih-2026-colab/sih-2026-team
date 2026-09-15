function [events,memory]=autonex_video_events(frame,memory)
% Events are rising edges / identity changes, not scripted timestamps.
events=struct('time',{},'event',{},'trackId',{},'candidateId',{},'evidence',{});
keys={}; records=events; out=frame.source;
ped=~isempty(out.sensorFrame.camera) && any(strcmpi({out.sensorFrame.camera.type},'pedestrian'));
if ped, add('pedestrian','PEDESTRIAN_DETECTED',[],[],'sensorFrame.camera.type'); end
for k=1:numel(frame.tracks)
    id=frame.tracks(k).id;
    add(sprintf('track:%d',id),'TRACK_CONFIRMED',id,[],'out.tracks: confirmed GNN output');
end
for k=1:numel(frame.candidates)
    c=frame.candidates(k);
    % Candidate IDs are frame-local. Name + target identify event continuity.
    key=sprintf('%s:%.17g:%.17g',c.name,c.targetSpeedKmh,c.targetY);
    if ~c.safe
        add(['rejected:' key],'CANDIDATE_REJECTED',[],c.id,strjoin(c.rejectionReasons,';'));
    end
    if c.selected && c.safe
        add(['selected:' key],'SAFE_PATH_SELECTED',[],c.id,'exact selected candidate; planner.safe && guardian.safe');
    end
    % Existing Guardian gates: frontSafe >=2, rearSafe >=1.5 seconds.
    frontCritical = ~isempty(c.guardian.minFrontTTC) && ...
    any(c.guardian.minFrontTTC(:) < 2);

rearCritical = ~isempty(c.guardian.minRearTTC) && ...
    any(c.guardian.minRearTTC(:) < 1.5);

if frontCritical || rearCritical
        add(['ttc:' key],'TTC_CRITICAL',[],c.id,'candidate Guardian minFrontTTC<2 or minRearTTC<1.5; horizon minimum');
    end
end
if ~ismember(frame.guardian,{'APPROVED','NO_ACTION'})
    add(['guardian:' frame.guardian],'GUARDIAN_INTERVENTION',[],[],frame.guardian);
end
add(['decision:' frame.decision],frame.decision,[],[],'out.motionMode');
if isempty(memory), memory={}; end
for k=1:numel(keys)
    if ~ismember(keys{k},memory), events(end+1)=records(k); end %#ok<AGROW>
end
memory=keys;
    function add(key,event,track,candidate,evidence)
        keys{end+1}=key;
        records(end+1)=struct('time',frame.time,'event',event,'trackId',track, ...
            'candidateId',candidate,'evidence',evidence);
    end
end
