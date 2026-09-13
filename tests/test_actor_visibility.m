function test_actor_visibility()
a=configure_new_sih_scenario(create_highway_scenario(),'occluded_pedestrian');
origin=[a(1).x a(1).y];
for sensor={'camera','thermal','radar'}
    v=autonex_actor_visibility(origin,a,3,sensor{1}); assert(v.fullyOccluded);
end
a(1).id=901; a(2).id=77; a(3).id=503;
v=autonex_actor_visibility(origin,a,3,'camera'); assert(v.fullyOccluded);
a(3).y=5.2;
v=autonex_actor_visibility(origin,a,3,'camera'); assert(v.visibleFraction>0 && v.visibleFraction<1);
a(3).y=7;
v=autonex_actor_visibility(origin,a,3,'camera'); assert(v.visibleFraction==1);
% Changing the blocker's physical extent changes occlusion, not its identity.
a(2).length=.1; a(2).width=.1; a(3).y=4;
v=autonex_actor_visibility(origin,a,3,'camera'); assert(v.visible);
fprintf('ACTOR_VISIBILITY_GEOMETRY_PASS\n');
end
