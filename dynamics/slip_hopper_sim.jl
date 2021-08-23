using DifferentialEquations
import DifferentialEquations
using Interpolations
using Plots

struct Slip{T}
    g::T
    ground::T
    l::T
    m::T
    control_k::T
    control_theta::T
end

robot = Slip(10.0, 0.0, 1.0, 1.0, 100.0, 10*(pi/180))

function flight!(du, u, p, t)
    robot = p
    du[1] = u[2]
    du[2] = 0
    du[3] = u[4]
    du[4] = -robot.g
end


function stance!(du, u, p, t)
    z = u
    robot = p
    x = z[1]; y = z[3]
    l = sqrt(x^2+y^2)
    F_spring = robot.control_k*(robot.l-l)
    Fx_spring = F_spring*(x/l)
    Fy_spring = F_spring*(y/l)
    Fy_gravity = robot.m*robot.g
    ẍ = (1/robot.m)*(Fx_spring)
    ÿ = (1/robot.m)*(-Fy_gravity+Fy_spring)

    du[1] = z[2]
    du[2] = ẍ
    du[3] = z[4]
    du[4] = ÿ

end


function release_condition(u, t, integrator)
    robot = integrator.p
    l = sqrt(u[1]^2 + u[3]^2)
    return l - robot.l
end

function release_affect!(integrator)
    # integrator.u[4] = -integrator.u[4]
    terminate!(integrator)
end


function apex_condition(u, t, integrator)
    return u[4]
end

function apex_affect!(integrator)
    terminate!(integrator)
end

function interp(ts, zs, fps)
    num_var = size(zs)[2]
    total_frames = Int(floor((ts[end]-ts[1])*fps))
    # println("ts delta ",ts)
    t = ts[1]:(ts[end]-ts[1])/total_frames:ts[end]
    z = zeros((length(t),num_var))

    for i = 1:num_var
        interp = LinearInterpolation(ts, zs[:,i])
        z[:,i] = interp.(t)
    end
    return t, z
end

function contact_condition(u, t, integrator)
     return u[3] - robot.l*cos(robot.control_theta)
end

function contact_affect!(integrator)
    integrator.u[4] = -integrator.u[4]
    terminate!(integrator)
end

contact_cb = ContinuousCallback(contact_condition, contact_affect!)
release_cb = ContinuousCallback(release_condition, release_affect!)
apex_cb = ContinuousCallback(apex_condition, apex_affect!)

function runsteps(z0, robot, steps, fps)
    x0 = 0; x0dot = z0[1]
    y0=z0[2]; y0dot = 0

    z0 = [x0 x0dot y0 y0dot]

    t0 = 0.0
    dt = 5.0

    t_ode = t0
    z_ode = [z0 x0+robot.l*sin(robot.control_theta) y0-robot.l*cos(robot.control_theta)]

    for i=1:steps
        println("Step: ",i)
        ### apex to ground
        tspan = (t0, t0+dt)
        prob = ODEProblem(flight!, z0, tspan, robot)
        sol = DifferentialEquations.solve(prob, Tsit5(), callback=contact_cb)
        z_temp1 = reduce(vcat, sol.u)
        t_temp1, z_temp1 = interp(sol.t, z_temp1, fps)

        z_temp1 = hcat(z_temp1, z_temp1[:,1].+ robot.l*sin(robot.control_theta),
        z_temp1[:,3].- robot.l*cos(robot.control_theta))
        t0 = t_temp1[end]
        z0[1:4] = z_temp1[end, 1:4]

        x_com = z0[1]
        z0[1] = -robot.l*sin(robot.control_theta)
        x_foot = x_com + robot.l*sin(robot.control_theta)
        y_foot = robot.ground


        ### stance phase
        tspan = (t0, t0+dt)
        prob2 = ODEProblem(stance!, z0, tspan, robot)
        sol2 = DifferentialEquations.solve(prob2, Tsit5(), callback=release_cb)
        z_temp2 = reduce(vcat, sol2.u)
        t_temp2 = sol2.t#; t_temp2[end]=t0+dt
        # println("Step ",i," tspan ",tspan," ",t_temp2)
        if t_temp2[1] != t_temp2[end]
            t_temp2, z_temp2 = interp(sol2.t, z_temp2, fps)
        end

        z_temp2[:,1] = z_temp2[:,1].+x_com .+ robot.l*sin(robot.control_theta)
        z_temp2 = hcat(z_temp2, x_foot*ones(size(z_temp2)[1]), y_foot*ones(size(z_temp2)[1]))
        t0 = t_temp2[end]
        z0[1:4] = z_temp2[end, 1:4]

        ### ground to apex
        tspan = (t0, t0+dt)
        prob3 = ODEProblem(flight!, z0, tspan, robot)
        sol3 = DifferentialEquations.solve(prob3, Tsit5(), callback=apex_cb)
        z_temp3 = reduce(vcat, sol3.u)
        t_temp3, z_temp3 = interp(sol3.t, z_temp3, fps)

        z_temp3 = hcat(z_temp3,
        z_temp3[1:end,1] .+ robot.l*sin(robot.control_theta),
        z_temp3[1:end,3] .- robot.l*cos(robot.control_theta))

        t0 = t_temp3[end]
        z0[1:4] = z_temp3[end, 1:4]

        t_ode = vcat(t_ode, t_temp1[2:end], t_temp2[2:end], t_temp3[2:end])
        z_ode = vcat(z_ode, z_temp1[2:end,:], z_temp2[2:end,:], z_temp3[2:end,:])

    end

    return [z_ode, t_ode]
end


function animate(ts, zs, robot, steps, fps)
    t_interp, z_interp = interp(ts, zs, fps)

    mm, nn = size(z_interp)
    min_xh = min(z_interp[:,1]...); max_xh = max(z_interp[:,1]...)

    window_xmin = -2.0*robot.l; window_xmax = 2*robot.l
    window_ymin = -0.1; window_ymax = 1.9*robot.l
    anim = Plots.Animation()
    rampref = [min_xh-2 max_xh+2; 0 0]

    plot(rampref[1,:], rampref[2,:], aspect_ratio=:equal, legend=false, linecolor=:black, linewidth=4,xlims=[window_xmin, window_xmax], ylims=[window_ymin, window_ymax])

    for i=1:mm
        plot(rampref[1,:], rampref[2,:], aspect_ratio=:equal, legend=false, linecolor=:black, linewidth=4,xlims=[window_xmin, window_xmax], ylims=[window_ymin, window_ymax])

        plot!([z_interp[i,1]], [z_interp[i,3]], markershape=:circle, markercolor=:red, markersize=15)
        plot!([z_interp[i,1], z_interp[i,5]], [z_interp[i,3], z_interp[i,6]], linewidth=2)

        sleep(0.01)
        frame(anim)
    end
    gif(anim, "dynamics/hopper.gif", fps=15)

end


z0 = [1.017, 1.18]
steps = 3
fps = 30
zs, ts = runsteps(z0, robot, steps, fps)
animate(ts, zs, robot, steps, fps)
