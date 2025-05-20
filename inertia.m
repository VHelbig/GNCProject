clc, clear, close all;
restoredefaultpath
rehash toolboxcache

g = 9.81;
m = 1;

% quad drone inertia matrix
x = [sqrt(2)/2 -sqrt(2)/2 -sqrt(2)/2 sqrt(2)/2];
y = [sqrt(2)/2 sqrt(2)/2 -sqrt(2)/2 -sqrt(2)/2];
z = [0 0 0 0];
Ic2 = zeros(3,3);
for i=1:1:4
    H = [(y(i)^2+z(i)^2) -x(i)*y(i) -x(i)*z(i);
         -x(i)*y(i) (x(i)^2+z(i)^2) -y(i)*z(i);
         -x(i)*z(i) -y(i)*z(i) (x(i)^2+y(i)^2)];
    Ic2 = Ic2 + m*H;
end

m=4;
M_RB = [ m*eye(3) , zeros(3) ; zeros(3) , Ic2 ];

A = [zeros(3), inv(Ic2); zeros(3), -eye(3)];
B = [zeros(3); eye(3)];

Q = eye(6); 
R = eye(3) * 500000;

[K_lqr, ~, ~] = lqr(A, B, Q, R);