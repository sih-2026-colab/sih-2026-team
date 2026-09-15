function telemetry=export_autonex_video_telemetry(scenario,options,outputRoot)
% Run the existing simulator and export observations only. No graphics needed.
% export_autonex_video_telemetry('occluded_pedestrian') runs 10 seconds.
if nargin<1, scenario='occluded_pedestrian'; end
if nargin<2, options=struct; end
root=fileparts(fileparts(mfilename('fullpath')));
if nargin<3, outputRoot=fullfile(root,'results','animation'); end
allowed={'occluded_pedestrian','cut_in','animal_crossing','dense_market','missing_lane','intersection'};
scenario=char(scenario); assert(ismember(scenario,allowed),'Unsupported video scenario');
oldPath=path; oldRng=rng; cleanup=onCleanup(@()restore(oldPath,oldRng)); %#ok<NASGU>
addpath(root,'-begin');
assert(strcmp(which('autonex_step'),fullfile(root,'autonex_step.m')),'Wrong autonex_step on path');
options.scenario=scenario;
if strcmp(scenario,'intersection'), options.scenario='unsignalized_intersection'; end
if ~isfield(options,'duration'), options.duration=10; end
if ~isfield(options,'perceptionMode'), options.perceptionMode='camera_radar_lidar'; end
assert(strcmp(options.perceptionMode,'camera_radar_lidar'),'Export currently supports the validated P3 perception mode');
options.explainabilityTelemetry=true; options.sensorTelemetry=true;
opts=autonex_options(options);
validateattributes(opts.dt,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(opts.duration,{'numeric'},{'scalar','real','finite','nonnegative'});
telemetry=struct('schemaVersion','1.0','scenario',scenario,'coreScenario',opts.scenario, ...
    'options',opts,'coordinateFrame','AutoNex world XY; metres; heading/steering radians', ...
    'sampleConvention','out at t before integration; command applies [t,t+dt]; source.nextEgo is t+dt', ...
    'nonfinitePolicy','[] and nonfinite entries recording exact paths and original NaN/+Inf/-Inf', ...
    'frames',{{}},'events',struct([]));
state=[]; memory=[]; times=0:opts.dt:opts.duration;
telemetry.frames=cell(1,numel(times)); allEvents=struct('time',{},'event',{},'trackId',{},'candidateId',{},'evidence',{});
egoRows=cell(0,12); trackRows=cell(0,15); candidateRows=cell(0,14);
for k=1:numel(times)
    [state,out]=autonex_step(state,times(k),opts);
    assert(all(isfinite([out.time out.x out.y out.speed out.egoYaw out.ax out.ay out.steeringAngle])), ...
        'Nonfinite critical ego output at sample %d',k);
    before=rng; frame=autonex_telemetry_frame(out);
    [events,memory]=autonex_video_events(frame,memory); allEvents=[allEvents events]; %#ok<AGROW>
    [frame,issues]=autonex_video_sanitize(frame); frame.nonfinite=issues;
    assert(isequal(before,rng),'Export projection consumed RNG');
    telemetry.frames{k}=frame;
    egoRows(end+1,:)={out.time,out.x,out.y,out.egoYaw,out.speed/3.6,out.ax,out.ay, ...
        out.steeringAngle,out.motionMode,out.guardianMode,out.collision,out.boundaryViolation}; %#ok<AGROW>
    for j=1:numel(frame.tracks)
        tr=frame.tracks(j);
        trackRows(end+1,:)={out.time,tr.id,tr.class,tr.x,tr.y,tr.vx,tr.vy,tr.rangeM, ...
            tr.relativeVxMps,tr.relativeVyMps,tr.confidence,tr.uncertainty, ...
            strjoin(tr.sensorSources,';'),tr.coasted,tr.attributeAvailability}; %#ok<AGROW>
    end
    for j=1:numel(frame.candidates)
        c=frame.candidates(j);
        candidateRows(end+1,:)={out.time,c.id,c.name,c.targetSpeedKmh,c.targetY,c.planner.score, ...
            c.safe,c.selected,c.planner.minClearance,c.guardian.minFrontTTC, ...
            c.guardian.minRearTTC,c.guardian.risk,strjoin(c.rejectionReasons,';'), ...
            sprintf('frames{%d}.candidates(%d).trajectory',k,j)}; %#ok<AGROW>
    end
end
telemetry.events=allEvents;
[telemetry.options,telemetry.optionNonfinite]=autonex_video_sanitize(opts,'options');
folder=fullfile(outputRoot,scenario); if ~isfolder(folder), mkdir(folder); end
save(fullfile(folder,'telemetry.mat'),'telemetry','-v7');
writeJson(fullfile(folder,'telemetry.json'),telemetry);
writeJson(fullfile(folder,'events.json'),allEvents);
writeCsv(fullfile(folder,'ego.csv'),egoRows, ...
    {'time','x','y','heading_rad','speed_mps','ax','ay','steering_rad','decision','guardian','collision','boundary_violation'});
writeCsv(fullfile(folder,'tracks.csv'),trackRows, ...
    {'time','track_id','class','x','y','vx','vy','range_m','relative_vx_mps','relative_vy_mps','confidence','uncertainty','sensor_sources','coasted','attribute_availability'});
writeCsv(fullfile(folder,'candidates.csv'),candidateRows, ...
    {'time','candidate_id','name','target_speed_kmh','target_y','score','safe','selected','min_center_distance','min_front_ttc','min_rear_ttc','guardian_risk','rejection_reasons','trajectory_reference'});
fprintf('EXPORT_PASS %s samples=%d events=%d\n',scenario,numel(times),numel(allEvents));
end
function restore(p,r)
path(p); rng(r);
end
function writeJson(file,value)
f=fopen(file,'w','n','UTF-8'); assert(f>=0,'Cannot open %s',file);
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s',jsonencode(value,ConvertInfAndNaN=false));
end
function writeCsv(file,rows,names)
% Explicit textual blanks for unavailable values; never write NaN/Inf literals.
for k=1:numel(rows)
    v=rows{k};
    if isempty(v), rows{k}='';
    elseif isnumeric(v), assert(isscalar(v) && isfinite(v)); rows{k}=sprintf('%.17g',v);
    elseif islogical(v), rows{k}=sprintf('%d',v);
    end
end
writecell([names;rows],file);
end
