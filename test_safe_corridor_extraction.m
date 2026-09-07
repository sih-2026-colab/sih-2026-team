function test_safe_corridor_extraction()
% Meaningful geometry regressions: connection, obstacle, unknown, map edges.
d.xValues=0:.5:20; d.yValues=-5:.25:5;
[d.X,d.Y]=meshgrid(d.xValues,d.yValues);
d.freeMask=true(size(d.X)); d.occupiedMask=false(size(d.X)); d.unknownMask=false(size(d.X));
s=inflate_drivable_space_for_vehicle(d,4.5,1.9);
assert(~any(s.safeFreeMask(:,1)),'Map edge must be treated as unknown');
ego=struct('x',4,'y',0);
c=extract_safe_corridor(d,s,ego);
assert(c.valid && c.length>10,'Open road must have a connected corridor');
% A full-width barrier cannot be bridged to the free island beyond it.
d.occupiedMask(:,23)=true; d.freeMask(:,23)=false;
s=inflate_drivable_space_for_vehicle(d,4.5,1.9);
c=extract_safe_corridor(d,s,ego);
assert(c.valid && c.x(end)<11,'Corridor must stop before inflated barrier');
tr=struct('x',[4 16],'y',[0 0]);
assert(~trajectory_in_safe_space(tr,d,s,ego),'Sparse trajectory must not jump a barrier');
d.occupiedMask(:)=false; d.unknownMask(:)=true; d.freeMask(:)=false;
s=inflate_drivable_space_for_vehicle(d,4.5,1.9);
c=extract_safe_corridor(d,s,ego);
assert(~c.valid,'Unknown map cannot authorize driving');
fprintf('SAFE_CORRIDOR_TESTS_PASS\n');
end
