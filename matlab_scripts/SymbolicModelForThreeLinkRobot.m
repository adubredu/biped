syms r l m M_H M_T g
syms th1 th2 th3
syms dth1 dth2 dth3

q = [th1; th2; th3];
dq = [dth1; dth2; dth3];

p_st_foot = [0;0];
p_st_knee = p_st_foot + [r/2*sin(th1); r/2*cos(th1)];
p_hip = p_st_foot + [r*sin(th1); r*cos(th1)];
p_torso = p_hip + [l*sin(th3); l*cos(th3)];
p_sw_knee = p_hip + [-r/2*sin(th2); -r/2*cos(th2)];
p_sw_foot = p_hip + [-r*sin(th2); -r*cos(th2)];

v_st_knee = jacobian(p_st_knee, q)*dq;
v_hip = jacobian(p_hip, q)*dq;
v_torso = jacobian(p_torso, q)*dq;
v_sw_knee = jacobian(p_sw_knee, q)*dq;
v_sw_foot = jacobian(p_sw_foot, q)*dq;

KE_st_knee = simplify((1/2)*m*(v_st_knee.'*v_st_knee));
KE_hip = simplify((1/2)*M_H*(v_hip.'*v_hip));
KE_torso = simplify((1/2)*M_T*(v_torso.'*v_torso));
KE_sw_knee = simplify((1/2)*m*(v_sw_knee.'*v_sw_knee));

KE = KE_st_knee + KE_hip + KE_torso + KE_sw_knee;
KE = simplify(KE);

PE = m*g*p_st_knee(2) + M_H*g*p_hip(2) + M_T*g*p_torso(2) + m*g*p_sw_knee(2);
PE = simplify(PE);

G = jacobian(PE, q).';
G = simplify(G);
D = simplify(jacobian(KE, dq).');
D = simplify(jacobian(D, dq));

syms C real
n = max(size(q));
for k=1:n
    for j=1:n
        C(k,j)=0*g;
        for i=1:n
            C(k,j) = C(k,j)+(1/2)*(diff(D(k,j), q(i))+...
                diff(D(k,i), q(j))-diff(D(i,j), q(k)))*dq(i);
        end
    end
end
C = simplify(C);

syms x1 x2 f g th3desired u1 u2
u = [u1; u2];
B = [1 0; 0 1; -1 -1];
x1 = q;
x2 = dq;
x = [x1; x2];

f = [x2; -inv(D)*C*x2 - inv(D)*G];
g = [zeros(3,2); inv(D)*B];
h = [th3-th3desired; th1+th2];

Lgh = jacobian(h,x)*g;
Lfh = jacobian(h,x)*f;

L2fh = jacobian(Lfh, x)*f;
LgLfh = jacobian(Lfh, x)*g;

ddy = L2fh + LgLfh*u;