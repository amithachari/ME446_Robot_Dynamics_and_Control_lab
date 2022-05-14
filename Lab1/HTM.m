syms alpha1 theta1 a1 d1 alpha2 theta2 a2 d2 alpha3 theta3 a3 d3

A1 = T_i(0, -90, 10, theta1)

A2 = T_i(10, 0, 0, theta2)


A3 = T_i(10, 0, 0, theta3)


T03 = simplify(A1*A2*A3)


function T = T_i(a,alpha,d,theta)
T = [cos(theta) -sin(theta)*cosd(alpha) sin(theta)*sind(alpha) a*cos(theta);
    sin(theta) cos(theta)*cosd(alpha) -cos(theta)*sind(alpha) a*sin(theta);
    0 sind(alpha) cosd(alpha) d;
    0 0 0 1];
end
