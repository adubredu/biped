function [tout, xout] = ThreeLinkWalker_ODE45(t_end, x0)

refine=4;
RelTol = 10^-5;
AbsTol = 10^-6;

stepSizeAngle = pi/8;
wn = 10;
zeta = 0.8;
Kp = wn^2;
Kd = 2*zeta*wn;

options = odeset('Refine',refine, 'RelTol',RelTol, 'AbsTol',AbsTol);

options = odeset('Events',@ThreeLinkEndStepEvents, 'MaxStep',0.01);

t_start = 0;
[tout, xout] = ode45(@f, [t_start t_end], x0, options, stepSizeAngle, Kp, Kd);

end

function dx = f(t,x,stepSizeAngle, Kp, Kd)

[r,m,M_H, M_T, l,g, p_st_foot, Mtotal] = model_params_stiff_legs();
parameters = [r,m,M_H, M_T, l,g, p_st_foot, Mtotal];

dx = zeros(6,1);
q = x(1:3);
dq = x(4:6);
[D,C,G,B] = dyn_mod_ThreeLinkWalker(q, dq, parameters);
