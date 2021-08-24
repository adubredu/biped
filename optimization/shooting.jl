using JuMP
using Ipopt
using Plots

struct Car::T
    t_opt::T
    p_opt::T
end

function car(du, u, p, t)

end

function f(T, u, z0, N)
    T = value.(T)
    u_opt = value.(u)
    t_opt = 0:T/(N-1):T
    zz = z0
    tt = 0
    g=9.81
    for i=1:N
        prob = ODEProblem(car, u₀, topt, g)


end
D = 5.0
N = 5

T_min = 1.0; T_max = 3.0
u_min = -5; u_max = 5;
z0 = [0,0] #x v
