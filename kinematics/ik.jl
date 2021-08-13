using JuMP
import Ipopt

# iksolver = Model(Ipopt.Optimizer)
# N = 2 #number of DOF
# xₐ = 0.5
# yₐ = 0
# l1 = 1
# l2 = 1
# @variable(iksolver, th1, start = -0.5)
# @variable(iksolver, th2, start = -0.5)
# @NLconstraint(iksolver, l1*cos(th1) + l2*cos(th1+th2) == xₐ)
# @NLconstraint(iksolver, l1*sin(th1) + l2*sin(th1+th2) == yₐ)
# @objective(iksolver, Max, 1.0)
# optimize!(iksolver)
# println(value.(th1), ", ", value.(th2))


function solve_ik(x, y)
    iksolver = Model(Ipopt.Optimizer)
    N = 2 #number of DOF
    l1 = 1
    l2 = 1
    @variable(iksolver, th1 <= 2pi, start = 0.5)
    @variable(iksolver, th2 <= 2pi, start = 0.5)
    @NLconstraint(iksolver, l1*cos(th1) + l2*cos(th1+th2) == x)
    @NLconstraint(iksolver, l1*sin(th1) + l2*sin(th1+th2) == y)
    @objective(iksolver, Max, 1.0)
    optimize!(iksolver)
    return [value.(th1), value.(th2)]
end


# solve_ik(0.398157, 1.357008)
