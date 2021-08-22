#canonical hybrid system example
using DifferentialEquations
using Plots

function f(du, u, g, t)
    du[1] = u[2] #position becomes velocity
    du[2] = -g   #velocity becomes acceleration
end

function condition(u, t, integrator)
    u[1]
end

function affect!(integrator)
    integrator.u[2] = -0.9*integrator.u[2]
end

cb = ContinuousCallback(condition, affect!)

u₀ = [50.0, 0.0]
tspan = (0.0, 50.0)
g = 9.8
prob = ODEProblem(f, u₀, tspan, g)
sol = DifferentialEquations.solve(prob, Tsit5(), callback=cb)
println(length(sol.u))
plot(sol)
