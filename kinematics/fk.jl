include("draw_links.jl")

#=
l1 = 1
l2 = 1
th1 = 0.5
th2 = pi/2

R01 = [cos(th1) -sin(th1); sin(th1) cos(th1)]
o01 = [0 0]
R12 = [cos(th2) -sin(th2); sin(th2) cos(th2)]
o12 = [l1 0]
p1 = [l1; 0; 1]
q2 = [l2; 0; 1]

H01 = zeros((3,3))
H01[1:2, 1:2] = R01
H01[1:2,3] = o01
H01[3,:] = [0,0,1]

H12 = zeros((3,3))
H12[1:2, 1:2] = R12
H12[1:2,3] = o12
H12[3,:] = [0,0,1]

p0 = H01*p1
q0 = H01*H12*q2

p = p0[1:2]
q = q0[1:2]
draw_arm([[o01, p], [p, q]])
=#

function solve_fk(th1, th2)
    l1 = 1
    l2 = 1
    R01 = [cos(th1) -sin(th1); sin(th1) cos(th1)]
    o01 = [0 0]
    R12 = [cos(th2) -sin(th2); sin(th2) cos(th2)]
    o12 = [l1 0]
    p1 = [l1; 0; 1]
    q2 = [l2; 0; 1]

    H01 = zeros((3,3))
    H01[1:2, 1:2] = R01
    H01[1:2,3] = o01
    H01[3,:] = [0,0,1]

    H12 = zeros((3,3))
    H12[1:2, 1:2] = R12
    H12[1:2,3] = o12
    H12[3,:] = [0,0,1]

    p0 = H01*p1
    q0 = H01*H12*q2

    p = p0[1:2]
    q = q0[1:2]

    return p,q
end


# solve_fk(0.5, pi/2)
