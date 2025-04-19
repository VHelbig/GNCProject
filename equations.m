clear
clc

g = 9.81;

syms t real;
syms Fx(t) Fy(t) Fz(t) Fp(t);
syms dFx(t) dFy(t) dFz(t) dFp(t);
syms ddFx(t) ddFy(t) ddFz(t) ddFp(t);
syms dddFx(t) dddFy(t) dddFz(t) dddFp(t);

% ----------------
% Angles
% ----------------

Fi = atan((Fx*sin(Fp) - Fy*cos(Fp)) / sqrt((Fz+g)^2 + (Fx*cos(Fp) + Fy*sin(Fp))));
dFi = diff(Fi, t)

Theta = atan((Fx*cos(Fp) + Fy*sin(Fp))/(Fz + g));
dTheta = diff(Theta, t)

Psi = Fp;
dPsi = diff(Fp, t)

% ----------------
% Angle velocity
% ----------------

wx = dFi - sin(Theta) * dPsi
wy = cos(Fi) * dTheta + cos(Theta)*sin(Fi) * dPsi
wz = -sin(Fi)*dTheta + cos(Theta)*cos(Fi) * dPsi

% ----------------
% Angle acceleration
% ----------------

dwx = diff(wx, t)
dwy = diff(wy, t)
dwz = diff(wz, t)