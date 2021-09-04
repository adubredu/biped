using JuMP
using Ipopt
using Plots
using Interpolations

struct Car{T}
    t_opt::T
    u_opt::T
end

function car(du, u, p, t)
    interp = LinearInterpolation(t_opt, u_opt)
    ct = interp.(t)
    du[1] = u[2]
    du[2] = ct
end

function f(T, u, z0, N)
    # T = value.(T)
    u_opt = u#value.(u)
    # t_opt = 0:T/(N-1):T
    t_opt = []
    i=0.0
    while i<T
        push!(t_opt, i)
        i+= T/(N-1)
    end
    zz = z0
    tt = 0
    p = Car(t_opt, u_opt)
    for i=1:N
        prob = ODEProblem(car, u₀, topt, p)
        sol = DifferentialEquations.solve(prob, Tsit5())
        zz = hcat(zz, sol.u[2:end, :])
        tt = hcat(tt, sol.t[2:end])
    end
    interp = LinearInterpolation(t_opt, u_opt)
    uu = interp.(tt)
    return (tt, zz, uu)
end



D = 5.0
N = 5

T_min = 1.0; T_max = 3.0
u_min = -5; u_max = 5;
z0 = [0,0] #x v
T_opt = 1
t_opt = 0.0:T_opt/(N+1):T_opt
u_opt = u_min .+ (u_max-u_min)*rand(1, N+1)

x0 = hcat(T_opt, u_opt)

shooting = Model(Ipopt.Optimizer)
@variable(shooting, T_min<=T<=T_max, start=T_opt)
@variable(shooting, u_min <=u[i=1:N+1]<=u_max)

@NLconstraint(shooting, f(T,u,z0,N)[2][end, 1]==D)
@NLconstraint(shooting, f(T,u,z0,N)[2][end, 2]==0)
