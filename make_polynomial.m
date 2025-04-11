%initial and final conditions

W_0=[30;30;-10;0];
d_W_0=[0;0;0;0];
dd_W_0=[0;0;0;0];


T=10;
W_1=[40;40;-20;pi/4];
d_W_1=[0;0;0;0];
dd_W_1=[0;0;0;0];

F_alphas=zeros([4,6]);

%compute polynomial for F1=x
for i=1:4
    IC=[W_0(i);
        d_W_0(i);
        dd_W_0(i)];
    FC=[W_1(i);
        d_W_1(i);
        dd_W_1(i)];
    F_alphas(i,:)=poly5traj(IC,FC,T);

end
F_alphas

t=linspace(0,T,10);
x=polyval(F_alphas(1,end:-1:1),t);
y=polyval(F_alphas(2,end:-1:1),t);
z=-polyval(F_alphas(3,end:-1:1),t);
figure
plot3(x,y,z)
xlabel("x")
ylabel("y")
zlabel("z")
title("desired trajectory")