using ODE
using Plots

#Projectile equations derived from Euler Lagrange
g = 9.81
c = 0.5
m = 1

x₀ = 0; ẋ₀ = 100
y₀ = 0; ẏ₀ = ẋ₀*tan(pi/3)
q₀ = [x₀; ẋ₀; y₀; ẏ₀]

t₀ = 0; tₙ = 5


function f(t, q)
    (x, ẋ, y, ẏ) = q
    xⁱ = ẋ
    yⁱ = ẏ
    vⁱₓ = -xⁱ*(c/m)*(xⁱ^2 + yⁱ^2)^0.5
    vⁱᵥ = -g - yⁱ*(c/m)*(xⁱ^2 + yⁱ^2)^0.5

    return [xⁱ; vⁱₓ; yⁱ; vⁱᵥ]
end

time = t₀:0.1:tₙ

t, q = ode45(f, q₀, time)

xs = map(q->q[1], q)
ys = map(q->q[3], q)
plot(xs, ys)
