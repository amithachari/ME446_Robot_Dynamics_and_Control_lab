syms ti tm tf a10 a11 a12 a13 a20 a21 a22 a23 v0 v1 v2

ti = 0.0;
tm = 0.5;
tf = 1.0;
Xdi = [6.5; 0.0; 17.0];
v0 = 0.0;
Xdm = [7.88; 9.03; 18.86];
Xdf = [1.62; 13.87; 12.0];
v2 = 0.0;

eqn1 = a0 + a1*ti + a2*ti*ti + a3*ti*ti*ti == q0;
eqn2 = a1 + 2*a2*ti + 3*a3*ti*ti == v0;
eqn3 = a0 + a1*tf + a2*tf*tf + a3*tf*tf*tf == q1;
eqn4 = a1 + 2*a2*tf + 3*a3*tf*tf == v1;

[A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4], [a0, a1, a2, a3])
X = vpa(linsolve(A,B))

% ti = 0.
% tf = 0.33
% q0 = 0.25
% v0 = 0.
% q1 = 0.75
% q2 = ?
% % v1 = 0.0
% v2 = 0.0

% eqn1 = a0 + a1*ti + a2*ti*ti + a3*ti*ti*ti == q0
% eqn2 = a1 + 2*a2*ti + 3*a3*ti*ti == v0
% eqn3 = a0 + a1*tf + a2*tf*tf + a3*tf*tf*tf == q1
% eqn4 = a1 + 2*a2*tf + 3*a3*tf*tf == v1

%% written by Harry starts
eqn1 = a10 + a11*ti + a12*ti*ti + a13*ti*ti*ti == Xdi(1) %position_ti
eqn2 = a11 + 2*a12*ti + 3*a13*ti*ti == v0%velocity_ti
eqn3 = a10 + a11*tf + a12*tf*tf + a13*tf*tf*tf == Xdm(1)%position_tm
eqn4 = a11 + 2*a12*tm + 3*a13*tm*tm - a21 - 2*a22*tm - 3*a23*tm*tm == 0%velocity_tm
eqn5 = 2*a12 + 6*a13*tm - 2*a22 - 6*a23*tm == 0%acceleration_tm
eqn6 = a20 + a21*tm + a22*tm*tm + a23*tm*tm*tm == Xdm(1)%position_tm
eqn7 = a20 + a21*tf + a22*tf*tf + a23*tf*tf*tf == Xdf(1)%position_tf
eqn8 = a21 + 2*a22*tf + 3*a23*tf*tf == v2%velocity_tf


[A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8], [a10, a11, a12, a13, a20, a21, a22, a23])
X = vpa(linsolve(A,B))

%% written by Harry ends

% 
% [A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4], [a0, a1, a2, a3])
% X = vpa(linsolve(A,B))

