clc;clear;close all;
format long
%% Modelo del robot
L1 = 0.078; 
L2 = 0.135; 
L3 = 0.147; 
L4 = 0.0253;

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
tool_z = 0.04115; 
Tool = R.tool.double;

R.base = transl(0, 0, 0);
Base = R.base.double;
R