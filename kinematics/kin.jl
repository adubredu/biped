include("fk.jl")
include("ik.jl")
using Plots
import Plots

phis = range(0, stop=2pi, length=50)
c = [1,0.5]
r = 0.5
xref = c[1].+r*cos.(phis)
yref = c[2].+r*sin.(phis)

anim = Plots.Animation()
traj = []
for i = 1:50
    th1, th2 = solve_ik(xref[i], yref[i])
    p, q = solve_fk(th1, th2)
    draw_arm([[[0,0], p], [p, q]])
    frame(anim)
    push!(traj, [th1, th2])
    closeall()
end

println(traj)
gif(anim, "circle.gif", fps=15)
