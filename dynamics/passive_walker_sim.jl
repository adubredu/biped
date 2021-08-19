using Ipopt
using JuMP
using DifferentialEquations

function fixedpt(z₀, walker)
    onestep(z₀, walker) - z₀
end

function partialder(f, z, walker)
    Δ = 1e-5
    n = length(z)
    J = zeros((n,n))
    for i=1:n
        ztemp1 = z; ztemp2 = z;
        ztemp1[i] = ztemp1[i]+Δ
        ztemp2[i] = ztemp2[i]-Δ
        J[:,i] = f(ztemp1, walker) - f(ztemp2, walker)
    end
    J = j/(2*Δ)
end

function collision_condition(u, t, integrator)
    # u[1]
end

function collision_affect!(integrator)
    # integrator.u[2] = -0.9*integrator.u[2]
end

collision_cb = ContinuousCallback(collision_condition, collision_affect!)

function single_stance(dz, z, walker, t)

end

function footstrike(t, z, walker)
end



function runsteps(z0, walker, steps...)
    l = walker.l
    flag = 1
    if length(steps) == 0
        flag = 0
        steps = [1]
    end
    theta1 = z0[1]
    xh = 0
    yh = l*cos(theta1)
    xh_start = xh

    t0 = 0
    dt = 4
    time_stamps = 100
    t_ode = t0
    z_ode = [z0 xh yh]

    for i=1:steps[1]
        tspan = (t0, timestamps)
        prob = ODEProblem(single_stance, z0, tspan, walker)
        sol = solve(prob, Tsit5(), callback=collison_cb)
        zplus = footstrike(sol.t[end], sol.u[end, :], walker)

        z0 = zplus
        t0 = sol.t[end]

        xh_temp = xh_start + l*sin(z_temp[1,1])-l*sin(z_temp[:,1])
        yh_temp = l*cos(z_temp[:,1])

        t_ode = [t_ode; t_temp[2:end]]
        z_ode = [z_ode; [z_temp[2:end-1, :]; zplus] xh_tem[2:end] yh_temp[2:end]]
        xh_start = xh_temp[end]
    end
    z = zplus[1:4]
    if flag == 1
        return (z_ode, t_ode)
    else
        return z
    end
end


function animate(ts, zs, walker, steps, fps)
end
