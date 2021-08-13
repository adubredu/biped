function [D,C,G,B] = dyn_mod_ThreeLinkWalker(q, dq, parameters)

th1 = q(1);
th2 = q(2);
th3 = q(3);

dth1 = dq(1);
dth2 = dq(2);
dth3 = dq(3);

r = parameters(1);
m = parameters(2);
M_H = parameters(3);
M_T = parameters(4);
l = parameters(5);
g = parameters(6);

D = zeros(3,3);
D(1,1) = (r^2*(4*M_H + 4*M_T +  5*m))/4;
D(1,2) = -(m*r^2*cos(th1 - th2))/2;
D(1,3) = M_T*l*r*cos(th1 - th3);
D(2,1) = -(m*r^2*cos(th1 - th2))/2;
D(2,2) = (m*r^2)/4;
D(3,1) = M_T*l*r*cos(th1 - th3);
D(3,3) = M_T*l^2;

C = zeros(3,3);
C(1,2) = -(dth2*m*r^2*sin(th1 - th2))/2;
C(1,3) = M_T*dth3*l*r*sin(th1 - th3);
C(2,1) = (dth1*m*r^2*sin(th1 - th2))/2;
C(3,1) = -M_T*dth1*l*r*sin(th1 - th3);

G = zeros(3,1);
G(1) = -(g*r*sin(th1)*(2*M_H +  2*M_T + 3*m))/2;
G(2) = (g*m*r*sin(th2))/2;
G(3) = -M_T*g*l*sin(th3);

B = [1 0; 0 1; -1 -1];

return
