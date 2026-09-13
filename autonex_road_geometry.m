function g=autonex_road_geometry(actors)
% Union of road rectangles; [xmin xmax ymin ymax]. No lane centre input.
g=struct('junction',false,'rectangles',[-inf inf -1.75 8.75], ...
    'segments',[-1e5 -1.75 1e5 -1.75;-1e5 8.75 1e5 8.75], ...
    'lateralBounds',[-1.75 8.75],'conflictZone',[]);
if isfield(actors,'roadGeometry') && ~isempty(actors(1).roadGeometry)
    g=actors(1).roadGeometry;
end
end
