clc;clear;close all;
format long
%% Modelo del robot
L1 = 0.078; 
L2 = 0.135; 
L3 = 0.147; 
L4 = 0.07192;

dh = [0 L1 0  pi/2 0;
      0 0  L2  0   0;
      0 0  L3  0   0;
      0 0  L4  0   0];

R = SerialLink(dh,'name','Brazo Principal MK2'); %se asumen articulaciones R
%LIMITES ARTICULARES
R.qlim(1,1:2) = [-185,  185]*pi/180;
R.qlim(2,1:2) = [-65,  45]*pi/180;
R.qlim(3,1:2) = [-70, 20]*pi/180;
R.qlim(4,1:2) = [-90, 90]*pi/180;

R.offset = [0 pi/2 -pi/2 0];
Offset = R.offset;

R.tool = transl(0, 0, 0); 
tool_z = 0.04361;
Tool = R.tool.double;

R.base = transl(0, 0, 0);
Base = R.base.double;
R
%% ANGULOS INICIALES
q_ini = deg2rad([45 -10 -10 20]);
T_obj = double(R.fkine(q_ini));
T_obj(1:3, 1:3) = eye(3);
T_obj

T_posta = T_obj;
T_posta(3,4) = T_posta(3,4)-tool_z 
%MATLAB: Mi posicion objetivo esta sin tool
%Lo resolvemos así y despues le restamos la tool para verificar

%STM32: A mi posicion objetivo le tengo que sumar
%la altura de tool para que lo resuelva sin tool

[qf, flag] = cinv_geometrica(dh,q_ini, T_obj,Base,Tool,Offset,R.qlim);
T_realizada = double(R.fkine(qf));
T_realizada(3,4) = T_realizada(3,4) -tool_z

if (~flag)
    fprintf('Posicion no alcanzable\n');
else
    figure()
    R.plot(qf);
    hold on
    trplot(T_obj,'color','b','frame','0','length',1.8)
end
