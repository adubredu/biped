using MeshCat, MeshCatMechanisms, RigidBodyDynamics, CoordinateTransformations
using GeometryTypes, MeshIO, FileIO, Rotations
using GeometryBasics: HyperRectangle, Vec, Point, Mesh, Line
using Colors: RGBA, RGB
using Plots
import PyPlot

function draw_arm(link_endpoints) 
    p = nothing
    plot()
    for coords in link_endpoints
        xs = [coords[i][1] for i=1:length(coords)]
        ys = [coords[i][2] for i=1:length(coords)]
        p = plot!(xs, ys, xlims=(-2,2), ylims=(-2,2), aspect_ratio=:equal, legend=false)

    end
    p
end


# draw_arm([[[0,0,0], [1, 1, 0]]])
