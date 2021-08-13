using MeshCat, MeshCatMechanisms, RigidBodyDynamics, CoordinateTransformations
using GeometryTypes, MeshIO, FileIO, Rotations
using GeometryBasics: HyperRectangle, Vec, Point, Mesh, Line
using Colors: RGBA, RGB
using Plots


function draw_arm(link_endpoints)
    # vis = Visualizer()
    # render(vis)
    p = nothing
    for coords in link_endpoints
        # p1 = Point(coords[1][1], coords[1][2])
        # p2 = Point(coords[2][1], coords[2][2])
        # l = Cylinder(p1, p2)
        # setobject!(vis, l)
        xs = [coords[i][1] for i=1:length(coords)]
        ys = [coords[i][2] for i=1:length(coords)]
        p = plot!(xs, ys, xlims=(-2,2), ylims=(-2,2), aspect_ratio=:equal, legend=false)
    end
    p
end


# draw_arm([[[0,0,0], [1, 1, 0]]])
