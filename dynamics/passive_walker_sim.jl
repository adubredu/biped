using Ipopt
using JuMP
using DifferentialEquations
import DifferentialEquations

struct Walker{T}
    M::T
    m::T
    I::T
    l::T
    c::T
    g::T
    γ::T
end

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
    θ₁ = u[1]; θ₂ = u[3]
    θ₁
end

function collision_affect!(integrator)
    # integrator.u[2] = -0.9*integrator.u[2]
    terminate!(integrator)
end

collision_cb = ContinuousCallback(collision_condition, collision_affect!)

function single_stance(dz, z, walker, t)
    θ₁ = z[1]
    θ₂ = z[3]
    ω₁ = z[2]
    ω₂ = z[4]
    M = walker.M; m = walker.m; I=walker.I; l = walker.l; c = walker.c;
    g = walker.g; γ = walker.γ

    A11 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(θ₂);
    A12 = I + c^2*m - c*l*m*cos(θ₂);
    A21 = I + c^2*m - c*l*m*cos(θ₂);
    A22 = I + c^2*m;
    A_ss = [A11 A12; A21 A22];

    b1 = c*g*m*sin(γ - θ₁) - M*g*l*sin(γ - θ₁) - c*g*m*sin(θ₁ - γ + θ₂) - 2*g*l*m*sin(γ - θ₁) - c*l*m*ω₁^2*sin(θ₂) - 2*c*l*m*ω₁*ω₁*sin(θ₂);
    b2 = -c*m*(g*sin(θ₁ - γ + θ₂) - l*ω₁^2*sin(θ₂));
    b_ss = [b1; b2];

    alpha = A_ss\b_ss;

    dz = [ω₁ alpha[1] ω₁ alpha[2]]''

end

function footstrike(t, z, walker)
    z=z[1]
    theta1_n = z[1]
    theta2_n = z[3]
    omega1_n = z[2]
    omega2_n = z[4]

    theta1 = theta1_n + theta2_n
    theta2 = -theta2_n

    M = walker.M; m = walker.m; I = walker.I
    l = walker.l; c=walker.c

    J11 = 1;
    J12 = 0;
    J13 = l*(cos(theta1_n + theta2_n) - cos(theta1_n));
    J14 = l*cos(theta1_n + theta2_n);
    J21 = 0;
    J22 = 1;
    J23 = l*(sin(theta1_n + theta2_n) - sin(theta1_n));
    J24 = l*sin(theta1_n + theta2_n);
    J = [J11 J12 J13 J14; J21 J22 J23 J24];

    A11 = M + 2*m;
    A12 = 0;
    A13 = (m*(2*c*cos(theta1_n + theta2_n) - 2*l*cos(theta1_n)))/2 + m*cos(theta1_n)*(c - l) - M*l*cos(theta1_n);
    A14 = c*m*cos(theta1_n + theta2_n);
    A21 = 0;
    A22 = M + 2*m;
    A23 = (m*(2*c*sin(theta1_n + theta2_n) - 2*l*sin(theta1_n)))/2 - M*l*sin(theta1_n) + m*sin(theta1_n)*(c - l);
    A24 = c*m*sin(theta1_n + theta2_n);
    A31 = (m*(2*c*cos(theta1_n + theta2_n) - 2*l*cos(theta1_n)))/2 + m*cos(theta1_n)*(c - l) - M*l*cos(theta1_n);
    A32 = (m*(2*c*sin(theta1_n + theta2_n) - 2*l*sin(theta1_n)))/2 - M*l*sin(theta1_n) + m*sin(theta1_n)*(c - l);
    A33 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(theta2_n);
    A34 = I + c^2*m - c*l*m*cos(theta2_n);
    A41 = c*m*cos(theta1_n + theta2_n);
    A42 = c*m*sin(theta1_n + theta2_n);
    A43 = I + c^2*m - c*l*m*cos(theta2_n);
    A44 = I + c^2*m;
    A_n_hs = [A11 A12 A13 A14; A21 A22 A23 A24; A31 A32 A33 A34; A41 A42 A43 A44];

    X_n_hs = [0 0 omega1_n omega2_n]';
    b_hs = [A_n_hs*X_n_hs; 0; 0];
    A_hs = [A_n_hs -J' ; J zeros((2,2))];
    X_hs = A_hs\b_hs;
    omega = zeros(2)
    omega[1] = X_hs[3]+X_hs[4]; omega[2] = -X_hs[4];

    zplus = [theta1 omega[1] theta2 omega[2]]
end



function runsteps(z0, walker, steps...)
    l = walker.l
    flag = 1
    if length(steps) == 0
        flag = 0
        steps = [1]
    end
    θ₁ = z0[1]
    xh = 0.0
    yh = l*cos(θ₁)
    xh_start = xh

    t0 = 0.0
    dt = 4
    time_stamps = 10.0
    t_ode = t0
    z_ode = [z0, xh, yh]

    # collision_cb = ContinuousCallback(collision_condition, collision_affect!)
    for i=1:steps[1]
        tspan = (t0, time_stamps)
        prob = ODEProblem(single_stance, z0, tspan, walker)
        sol = DifferentialEquations.solve(prob, Tsit5(), callback=collision_cb)
        zplus = footstrike(sol.t[end], sol.u[end, :], walker)

        z0 = zplus
        t0 = sol.t[end]

        z_temp = sol.u
        t_temp = sol.t

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
    l = walker.l
    c = walker.c
    mm = size(z,1)


end


walker = Walker(1.0, 0.5, 0.02, 1.0, 0.5, 1.0, 0.01)
q1 = 0.2; u1 = -0.25;
q2 = -0.4; u2 = 0.2;

z0 = [q1 u1 q2 u2];

steps = 3;
fps = 20;
zstar = [0.162597833780041,  -0.231869638058930,  -0.325195667560083,   0.037978468073743]
z, t = runsteps(zstar,walker,steps)
# tspan = (0.0, 10.0)
# prob = ODEProblem(single_stance, zstar, tspan, walker)
# sol = DifferentialEquations.solve(prob, Tsit5())#, callback=collision_cb)
