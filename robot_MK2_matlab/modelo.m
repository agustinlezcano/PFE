clc;clear;
%% Modelo del robot
%Matriz DENAVIT-HARTEMBERG
%Parametros

%------------------Robot 1--------------------
%---------------------------------------------
%corregir las longiudes
L1 =0.057;
L2 =0.135;
L3 =0.147;
L1 =1;
L2 =1;
L3 =1;
%en el original es pi/2
dh = [0 L1  0 -pi/2   0;
      0 0   L2  0     0;
      0 0   L3  0     0];

R=SerialLink(dh,'name','Brazo Principal MK2');
%LIMITES ARTICULARES
R.qlim(1,1:2) = [-185,  185]*pi/180;
R.qlim(2,1:2) = [-185,  185]*pi/180;
R.qlim(3,1:2) = [-185, 228]*pi/180;

% R.offset = [pi -pi/2 -pi/2];
R.offset = [pi -pi/2 -pi/2];

%cambiar las dimensiones de tool
R.tool = transl(.0817, 0, 0) * trotz(-pi/2) * transl(0.0419, 0, 0);
R.tool = eye(4)
Tool = R.tool.double;
R.base = transl(0, 0, 0.0);
Base = R.base.double;
%------------------Robot 2--------------------
%---------------------------------------------
L1a = 0.0694;
L2a = 0.057;
L3a = 0.135;
L4a = 0.2043;
Xa = 0.025;
dh = [0 L1a  0 pi/2   0;
      0 Xa   L2a  0     0;
     -pi/2 0   L3a  0 0;
     -pi/2 -Xa L4a 0 0];

R2=SerialLink(dh,'name','Brazo Secundario MK2');
%LIMITES ARTICULARES
R2.qlim(1,1:2) = [-185,  185]*pi/180;
R2.qlim(2,1:2) = [-185,  185]*pi/180;
R2.qlim(3,1:2) = [-228, 228]*pi/180;
R2.qlim(4,1:2) = [-185, 228]*pi/180;

R2.offset = [0 pi -pi/2 -pi/2];
R2.tool = transl(.0817, 0, 0) * trotz(-pi/2) * transl(0.0419, 0, 0);
Tool2 = R2.tool.double;
R2.base = transl(0, 0, 0.0);
Base2 = R2.base.double;
%------------------Robot 3--------------------
%---------------------------------------------
L1c =0.057;
L2c =0.038;
L3c =0.155;
L4c =0.0685;
L5c =0.1468;
L6c =0.04345;
dh = [0 L1c  0 pi/2   0;
     0 0.0205   L2c  0     0;
      pi/2 -0.0075   L3c  0     0;
      0 0 L4c 0 0;
      0 -0.0125 L5c 0 0;
      .8406 0 L6c 0 0];

R3=SerialLink(dh, 'name','Soporte MK2');
%LIMITES ARTICULARES
R3.qlim(1,1:2) = [-185,  185]*pi/180;
R3.qlim(2,1:2) = [-195,  195]*pi/180;
R3.qlim(3,1:2) = [-85, 228]*pi/180;

R3.offset = [0 pi -pi/2 -pi/2 0 0];
R3.tool = transl(.0817, 0, 0) * trotz(-pi/2) * transl(0.0419, 0, 0);
Tool3 = R3.tool.double;
R3.base = transl(0, 0, 0.0);
Base3 = R3.base.double;

%% ANGULOS INICIALES
q_ini=[0 0 0];

%% Menu principal 
fprintf('Bienvenido a nuestro Proecto Final de Estudios\n')
fprintf('Realizado sobre brazo MK2 de 3 gdl\n\n')
fprintf('Las opciones son las siguientes:\n\n')
fprintf('1:Espacio de Trabajo \n')
fprintf('2: Cinematica inversa \n')
fprintf('3: Singularidades \n')
fprintf('4: Trayectoria \n\n')
fprintf('5: Robot completo \n\n')
x=input('Ingrese opcion: ');

switch x
    case 1
        figure()
        R.plot(q_ini,'trail', {'r'})
        hold on
        R.teach()
    case 2
        % Cinematica Inversa
        T = transl(0.43,-0.993,1.958);  %Posicion objetivo. z = 0.317
        T = Base * T * Tool;
        [q_obj, flag, qqs] = cInversa(q_ini, dh, T, R)
        
        % Comparacion con Ikine 
        L1 =0.057;
        L2 =0.135;
        L3 =0.147;
        L1 =1;
        L2 =1;
        L3 =1;
        dh = [0 L1  0 -pi/2   0;
              0 0   L2  0     0;
              0 0   L3  0     0];

        R1=SerialLink(dh,'name','MK2-b');
        %LIMITES ARTICULARES
        R1.qlim(1,1:2) = [-185,  185]*pi/180;
        R1.qlim(2,1:2) = [-185,  185]*pi/180;
        R1.qlim(3,1:2) = [-185, 228]*pi/180;

        R1.offset = [pi -pi/2 -pi/2];
        R1.tool = transl(.0817, 0, 0) * trotz(-pi/2) * transl(0.0419, 0, 0)
        R.tool = eye(4)
        Tool = R1.tool.double;
        
        R1.base = transl(0, 0, 0.0);
        Base = R1.base.double;
        T1 = transl(0.43,-0.993,1.958); %Posicion objetivo
        T1 = Base * T1 * Tool;

        Q = R1.ikine(T1, 'mask', [1 1 1 0 0 0])
        figure()
%         R.plot(q_obj)
        R.plot(qqs(:,1)')
        hold on
        R1.plot(Q)
        
        %comparacion de 4 posiciones
%         figure()
%         R.plot(qqs(:,1)')
%         for i=1:4
%             R.plot(qqs(:,i)')
%             hold on
%         end
    case 5
        q_a=[0 0 0];
        figure()
        R.plot(q_a,'scale', 0.6);
        hold on
        q_b=[0 0 0 0];
        R2.plot(q_b,'scale', 0.6); %4 parametros
        hold on
        q_c=[0 0 0 0 0 0];
        R3.plot(q_c,'scale', 0.6); %6 parametros
        
end
