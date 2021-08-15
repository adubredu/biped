using Symbolics

#symbolic diff
@variables x
d = Differential(x)
f = x^2 + 2x + 2
dfdx = expand_derivatives(d(f))
soln = substitute(dfdx, (Dict(x=>1)))
println("symbolic diff answer: ", soln)
###

#finite differencing
function ff(x)
    x^2+2x+2
end

function central_difference(x, ff)
    dx = 1e-3
    (ff(x+dx) - ff(x-dx)) / 2*dx
end

soln2 = central_difference(1, ff)
println("central diff answer: ", soln2)
###

#Symbolic derivation of Lagrangian Equations of Motion of a 2D Projectile
@variables x, y, ẋ, ẏ, ẍ, ÿ, m, c, g
T = 0.5m*(ẋ^2 + ẏ^2)
V = m*g*y
L = T - V
Fₓ = -c*ẋ * (ẋ^2 + ẏ^2)^0.5
Fᵥ = -c*ẏ * (ẋ^2 + ẏ^2)^0.5

q = [x, y]
q̇ = [ẋ, ẏ]
q̈ = [ẍ, ÿ]
F = [Fₓ, Fᵥ]
N = length(q)
dif = Differential
solve = expand_derivatives
EOM = []

for ii=1:N
    δL_δq̇ = dif(q̇[ii])(L)
    ddt_δL_δq̇ = dif(q[1])(δL_δq̇)*q̇[1] +
                dif(q̇[1])(δL_δq̇)*q̈[1] +
                dif(q[2])(δL_δq̇)*q̇[2] +
                dif(q̇[2])(δL_δq̇)*q̈[2]
    δL_δq = dif(q[ii])(L)
    eom = ddt_δL_δq̇ - δL_δq - F[ii]
    push!(EOM, eom)
end
println("ẍ => 0 = ", solve(EOM[1]))
println("ÿ => 0 = ", solve(EOM[2]))
