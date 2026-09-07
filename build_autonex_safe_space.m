function [drivable,safeSpace,corridor] = build_autonex_safe_space(actors)
% Synthetic depth abstraction -> 3-D occupancy -> vehicle-safe 2-D corridor.
[points,pose] = simulate_autonex_depth_scan(actors);
map = occupancyMap3D(4);
insertPointCloud(map,pose,pointCloud(points),80,[0.30 0.75]);
drivable = extract_drivable_space_from_3d(map,actors(1),70,-1.75,8.75,8);
safeSpace = inflate_drivable_space_for_vehicle(drivable,4.5,1.9);
corridor = extract_safe_corridor(drivable,safeSpace,actors(1));
end
