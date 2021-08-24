using JuMP
using Ipopt
using Plots

D = 5.0
N = 5

T_min = 1.0; T_max = 3.0
x_min = 0; x_max = D
v_min = -10.0; v_max = 10.0
u_min = -5; u_max = 5;

T_init = 1.0
x_init = zeros(N+1)
v_init = zeros(N+1)
u_init = zeros(N+1)

collocation = Model(Ipopt.Optimizer)
@variable(collocation, T_min <=T<=T_max, start = T_init)
@variable(collocation, x_min <=x[i=1:N+1]<=x_max, start=x_init[i])
@variable(collocation, u_min <=u[i=1:N+1]<=u_max, start=u_init[i])
@variable(collocation, v_min <=v[i=1:N+1]<=v_max, start=v_init[i])

@constraint(collocation, x[1]==0.0)
@constraint(collocation, x[N+1]==D)
@constraint(collocation, v[1]==0.0)
@constraint(collocation, v[N+1]==0.0)
@constraint(collocation, x_con[i=1:N],
                x[i+1]-x[i]-v[i]*(T/(N-1)) == 0.0)
@constraint(collocation, v_con[i=1:N],
                v[i+1]-v[i]-0.5*(u[i]+u[i+1])*(T/(N-1)) == 0)
@objective(collocation, Min, T)
optimize!(collocation)

Topt = value.(T)
xopt = value.(x)
vopt = value.(v)
uopt = value.(u)

ns = 1:N+1
println("Topt: ", Topt)
plot(ns, xopt)
plot!(ns, vopt)
plot!(ns, uopt)
