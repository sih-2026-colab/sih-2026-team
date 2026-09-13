function test_prototype_components()
test_safe_corridor_extraction;
command=cat_reflex_guardian(0,36);
acceleration=speed_tracking_controller(0,36,command);
assert(acceleration>0 && acceleration<=1.2,'Vehicle cannot recover speed after replanning');
command=cat_reflex_guardian(36,0);
acceleration=speed_tracking_controller(36,0,command);
assert(acceleration<0 && acceleration>=-3.5,'Controlled braking limit violated');
actors=create_highway_scenario();
radar=scan_autonex_radar_suite(actors);
reports=radar_to_object_detections(radar,actors(1),0);
assert(all(cellfun(@(d)numel(d.Measurement)==3,reports)));
assert(any(cellfun(@(d)d.SensorIndex>2,reports)),'Corner radar reports were discarded');
x=0:.5:20; y=-3:.25:3; [X,Y]=meshgrid(x,y);
flat=[X(:) Y(:) .01*sin(X(:))];
h=detect_road_surface_hazards(flat,x,y); assert(h.hazardCells==0,'Flat road false positive');
dip=abs(X-10)<1.5 & abs(Y)<1; flat(dip(:),3)=-.25;
h=detect_road_surface_hazards(flat,x,y); assert(h.hazardCells>10,'Depression was missed');
empty=flat; empty(:,3)=NaN;
h=detect_road_surface_hazards(empty,x,y); assert(h.hazardCells==0,'Unknown must not become a detected pothole');
detection=objectDetection(0,[10;3;0],'MeasurementNoise',.1*eye(3));
filter=initekfimm(detection);
assert(isa(filter,'trackingIMM') && numel(filter.TrackingFilters)==3);
tracker=create_autonex_gnn_tracker('IMM');
for t=0:.05:.15
    [tracks,~,~]=tracker({objectDetection(t,[10+5*t;3+t;0],'MeasurementNoise',.1*eye(3))},t);
end
assert(~isempty(tracks) && numel(tracks(1).State)==6);
assert(all(isfinite(tracks(1).State)));
fprintf('PROTOTYPE_COMPONENT_TESTS_PASS\n');
end
