using Symbolics
using LinearAlgebra

@variables M m I
@variables c l
@variables gam g
@variables theta1 theta2
@variables omega1 omega2
@variables alpha1 alpha2
@variables theta1_n theta2_n
@variables omega1_n omega2_n
@variables x y
@variables vx vy
@variables ax ay


#### Position vectors ##########
R01 = simplify.([cos(pi/2+theta1) -sin(pi/2+theta1);
                 sin(pi/2+theta1) cos(pi/2+theta1)])
R12 = simplify.([cos(-pi+theta2) -sin(-pi+theta2);
                sin(-pi+theta2) cos(-pi+theta2)])

O01 = [x; y]
O12 = [l; 0]
H01 = [R01 O01; 0 0 1]
H12 = [R12 O12; 0 0 1]

r_C1 = [x; y]

R_H = H01*[l; 0; 1]
r_H = R_H[1:2]
x_H = r_H[1]; y_H = r_H[2]

R_G1 = H01*[(l-c); 0; 1]
r_G1 = R_G1[1:2]
x_G1 = r_G1[1]; y_G1 = r_G1[2]

R_G2 = H01*H12*[c; 0; 1]
r_G2 = simplify.(R_G2[1:2])
x_G2 = r_G2[1]; y_G2 = r_G2[2]

R_C2 = H01*H12*[l; 0; 1]
r_C2 = simplify.(R_C2[1:2])
x_C2 = r_C2[1]; y_C2 = r_C2[2]


##### Velocity vectors #######
v_H_x= Symbolics.jacobian([x_H],[x, y, theta1, theta2])*[vx vy omega1 omega2]'
v_H_y= Symbolics.jacobian([y_H],[x, y, theta1, theta2])*[vx vy omega1 omega2]'
v_G1_x= Symbolics.jacobian([x_G1],[x, y, theta1, theta2])*[vx vy omega1 omega2]'
v_G1_y= Symbolics.jacobian([y_G1],[x, y, theta1, theta2])*[vx vy omega1 omega2]'
v_G2_x= Symbolics.jacobian([x_G2],[x, y, theta1, theta2])*[vx vy omega1 omega2]'
v_G2_y= Symbolics.jacobian([y_G2],[x, y, theta1, theta2])*[vx vy omega1 omega2]'
v_H = [v_H_x; v_H_y]
v_G1 = [v_G1_x; v_G1_y]
v_G2 = [v_G2_x; v_G2_y]


#### Position vectors for Potential energy ####

R = simplify.([cos(-gam) -sin(-gam);
               sin(-gam) cos(-gam)])
R_H = R*[x_H; y_H]
R_G1 = R*[x_G1; y_G1]
R_G2 = R*[x_G2; y_G2]

Y_H = R_H[2]
Y_G1 = R_G1[2]
Y_G2 = R_G2[2]


### Potential, Kinetic and Lagrangian ###

T = 0.5*(simplify.(m*dot(v_G1, v_G1) + m*dot(v_G2, v_G2) + M*dot(v_H, v_H)
    + I*(dot(omega1, omega1) + dot(omega1+omega2, omega1+omega2))))
V = simplify.(m*g*Y_G1 + m*g*Y_G2 + M*g*Y_H)
L = T-V


#### Deriving Euler-Lagrange Equations of Motion ####

q = [x y theta1 theta2]
q̇ = [vx vy omega1 omega2]
q̈ = [ax ay alpha1 alpha2]

dif = Differential
N = length(q)
EOM = []

for ii=1:N
    δL_δq̇ = dif(q̇[ii])(L)
    ddt_δL_δq̇ = dif(q[1])(δL_δq̇)*q̇[1] +
                dif(q̇[1])(δL_δq̇)*q̈[1] +
                dif(q[2])(δL_δq̇)*q̇[2] +
                dif(q̇[2])(δL_δq̇)*q̈[2] +
                dif(q[3])(δL_δq̇)*q̇[3] +
                dif(q̇[3])(δL_δq̇)*q̈[3] +
                dif(q[4])(δL_δq̇)*q̇[4] +
                dif(q̇[4])(δL_δq̇)*q̈[4]
    δL_δq = dif(q[ii])(L)
    eom = simplify.(ddt_δL_δq̇ - δL_δq)
    push!(EOM, eom)
end

# EOM = vec(EOM)
#### Single Strike equations ####
A_ss = Symbolics.jacobian(EOM, [ax, ay, alpha1, alpha2])
b_ss1 = substitute(EOM[1], (Dict(ax=>0, ay=>0, alpha1=>0, alpha2=>0)))
b_ss2 = substitute(EOM[2], (Dict(ax=>0, ay=>0, alpha1=>0, alpha2=>0)))
b_ss3 = substitute(EOM[3], (Dict(ax=>0, ay=>0, alpha1=>0, alpha2=>0)))
b_ss4 = substitute(EOM[4], (Dict(ax=>0, ay=>0, alpha1=>0, alpha2=>0)))

J_sw = Symbolics.jacobian([x_C2, y_C2], [x, y, theta1, theta2])
J_n_sw = substitute(J_sw, (Dict(theta1=>theta1_n, theta2=>theta2_n)))
A_n_hs = substitute(A_ss, (Dict(theta1=>theta1_n, theta2=>theta2_n)))
