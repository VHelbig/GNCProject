m_i=1;
m=4;
points=[[0.5,0.5];[0.5,-0.5];[-0.5,-0.5];[-0.5,0.5]];
I=0;
for i=1:length(points)
    x=points(i,1);
    y=points(i,2);
    z=0;
    Ii=m_i*[y^2+z^2,-x*y,-x*z;-x*y,x^2+z^2,-y*z;-x*z,-y*z,x^2+y^2];
    I=I+Ii;
end
Ic=I
M_RB=[m*eye(3),zeros(3);zeros(3),Ic]

W=zeros([4,5]);
W(:,1)=[20;20;-17;0];
W(:,2)=[10;10;-17;0];
W(:,3)=[10;20;-15;0];
W(:,4)=[20;10;-17;0];
W(:,5)=[20;20;-17;0];

step_size=0.01;
pos_init=[30;-30;10];
R_b2e_init=eye(3);
nu_init=zeros([6,1]);

pos_sig=Simulink.BusElement;
pos_sig.Name="pos";
pos_sig.Dimensions=[3,1];

R_sig=Simulink.BusElement;
R_sig.Name="R_b2e";
R_sig.Dimensions=[3,3];


ModelStateBus = Simulink.Bus;
ModelStateBus.Elements = [pos_sig, R_sig];
