clc;clear;
%Matriz DENAVIT-HARTEMBERG
%Parametros
L1 =1.678;
L2 =3.3345;
L3 =3.6369;
dh = [0 L1  0 -pi/2   0;
      0 0   L2  0     0;
      0 0   L3  0     0];

R=SerialLink(dh,'name','MK2');
%LIMITES ARTICULARES
R.qlim(1,1:2) = [-185,  185]*pi/180;
R.qlim(2,1:2) = [-155,  95]*pi/180;
R.qlim(3,1:2) = [-85, 228]*pi/180;

R.offset = [0 -pi/2 -pi/2];

R.tool = transl(1.848, 0, 0.506);
Tool = R.tool.double;

R.base = transl(0, 0, 0);
Base = R.base.double;
%ANGULOS INICIALES
q_ini=[0 0 0];

T=transl(1,0,2.75)*trotz(-pi/2)*trotx(-pi/4); %Posicion objetivo
T = Base * T * Tool

[q_obj, flag, qqs] = cInversa(q_ini, dh, T, R)

figure()
R.plot(qqs(:,1)')
%% Comparacion con Ikine 
L1 =1.678;
L2 =3.3345;
L3 =3.6369;
dh = [0 L1  0 -pi/2   0;
      0 0   L2  0     0;
      0 0   L3  0     0];

R1=SerialLink(dh,'name','MK2-b');
%LIMITES ARTICULARES
R1.qlim(1,1:2) = [-185,  185]*pi/180;
R1.qlim(2,1:2) = [-155,  95]*pi/180;
R1.qlim(3,1:2) = [-85, 228]*pi/180;

R1.offset = [0 -pi/2 -pi/2];

R1.tool = transl(1.848, 0, 0.506);
Tool = R1.tool.double;

R1.base = transl(0, 0, 0);
Base = R1.base.double;

Q = R1.ikine(T, 'mask', [1 1 1 0 0 0])
figure()
R1.plot(Q)
%figure()
%R.teach()