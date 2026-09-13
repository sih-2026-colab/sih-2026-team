function [drivable,safeSpace,corridor] = build_autonex_safe_space(actors)
% Synthetic depth abstraction -> 3-D occupancy -> vehicle-safe 2-D corridor.
[points,pose] = simulate_autonex_depth_scan(actors);
map = occupancyMap3D(4);
insertPointCloud(map,pose,pointCloud(points),80,[0.30 0.75]);
geometry=autonex_road_geometry(actors);
drivable = extract_drivable_space_from_3d(map,actors(1),70,geometry.lateralBounds(1),geometry.lateralBounds(2),8);
if geometry.junction
    outside=~autonex_road_contains(drivable.X,drivable.Y,geometry);
    drivable.occupiedMask=drivable.occupiedMask | outside;
    drivable.freeMask=drivable.freeMask & ~outside;
    drivable.unknownMask=drivable.unknownMask & ~outside;
    drivable.classification(outside)=1;
    drivable.roadGeometry=geometry;
end
surface=simulate_road_surface_cloud(actors,drivable);
hazards=detect_road_surface_hazards(surface,drivable.xValues,drivable.yValues);
drivable.occupiedMask=drivable.occupiedMask | hazards.mask;
drivable.freeMask=drivable.freeMask & ~hazards.mask;
drivable.unknownMask=drivable.unknownMask & ~hazards.mask;
drivable.classification(hazards.mask)=1;
drivable.surfaceHazards=hazards;
safeSpace = inflate_drivable_space_for_vehicle(drivable,4.5,1.9);
corridor = extract_safe_corridor(drivable,safeSpace,actors(1));
end
