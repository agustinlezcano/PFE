clc;clear;
%% Modelo del robot
%Matriz DENAVIT-HARTEMBERG
%Parametros
%syms q1 q2 q3 q4 real
%------------------Robot 1--------------------
%---------------------------------------------
%Longitudes de los eslabones
L1 =0.11715; %[m]
L2 =0.13525; %[m]
L3 =0.147; %[m]
L4 =0.07223; %[m]

dh = [0 L1 0 pi/2 0;
      0 0 L2  0 0;
      0 0 L3  0 0;
      0 0 L4 0 0];

R=SerialLink(dh,'name','Brazo Principal MK2'); %se asumen articulaciones R
%LIMITES ARTICULARES
R.qlim(1,1:2) = [-185,  185]*pi/180;
R.qlim(2,1:2) = [-45,  25]*pi/180;
R.qlim(3,1:2) = [-50, 0]*pi/180;
R.qlim(4,1:2) = [-90, 90]*pi/180;

L1a = 0.0694;
L2a = 0.057;
L3a = 0.135;
L4a = 0.2043;
Xa = 0.025;


% Funciona con offset = 0  --> revisar ángulos
R.offset = [0 pi/2 -pi/2 0]%[pi/12 -pi/12 -pi/12 0]; % [pi -pi/2 -pi/2 0]
Offset = R.offset;
%cambiar las dimensiones de tool
% R.tool = transl(.0817, 0, 0) * trotz(-pi/2) * transl(0.0419, 0, 0);
R.tool = transl(0, 0, 0.0);% * trotz(-pi/2);
% R.tool = eye(4);
Tool = R.tool.double;
% Herramienta: corregir segun altura del efector final
R.base = transl(0, 0.017, 0.0); % Desplazamiento, no cambio de orientación (no necesario)
Base = R.base.double;
%------------------Robot 2--------------------
%
dh_2 = [0 L1a  0 pi/2   0;
      0 Xa   L2a  0     0;
     -pi/2 0   L3a  0 0;
     -pi/2 -Xa L4a 0 0];

R2=SerialLink(dh_2,'name','Brazo Secundario MK2');
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

%% ANGULOS INICIALES
q_ini=[0 0 0 0]; % tiene referencia virtual

%% Menu principal 
fprintf('Bienvenido a nuestro Proecto Final de Estudios\n')
fprintf('Realizado sobre brazo MK2 de 3 gdl\n\n')
fprintf('Las opciones son las siguientes:\n\n')
fprintf('1:Espacio de Trabajo \n')
fprintf('2: Cinematica directa \n')
fprintf('3: Cinematica inversa \n')
fprintf('4: Singularidades \n')
fprintf('5: Trayectoria \n')
fprintf('6: Robot completo \n\n')
x=input('Ingrese opcion: ');

switch x
    case 1
        figure()
        R.plot(q_ini,'trail', {'r'})
        R.plot(q_ini)
        hold on
        R.teach('callback', @velelips)
        
    case 2
        % Cinemática directa - Comparación
        % q_ini = [1 1 1 0]; % [q1 q2 q3 0];
        q_ini = [0.4520    0.6458   -1.5865    0.0017];
        fprintf('Matriz Homogénea usando función de Toolbox: \n')
        Ttoolbox_ini = R.fkine(q_ini)
        
        fprintf('Matriz Homogénea usando función propia: \n')
        [T_ini,q,flag] = c_dir(q_ini, dh, R) % T1 COMPONENTES
                
    case 3
        % Cinematica Inversa --> Comparacion
        fprintf('Se parte de una posición deseada, obteniendo una matriz de transformación homogénea: \n')
        q_ini = deg2rad([45 -10 -10 20]); % 30    -10 1  -45    20
        %T_obj = double(R.fkine(q_ini));
%         T_obj = transl(0.187, 0, 0.14)
%         T_obj = transl(0.143, -0.154, 0.177)
        T_obj = transl(2, 2, 2)
        T_obj1 = transl(0.187, 0, 0.14) * trotx(pi/2);
        % La rotacion es por la herramienta
        fprintf('Mediante la Matriz de Transformación Homogénea del ejemplo, se llega a las posiciones articulares deseadas: \n')
        % T_test = transl(.132,.132,.140)*trotx(deg2rad(90))*troty(deg2rad(45))
        lim = R.qlim;
        [qf, flag] = cinv_geometrica(dh,q_ini, T_obj,Base,Tool,Offset,lim);
        if (~flag)
            fprintf('Posicion no alcanzable\n');
        else
            figure()
            R.plot(qf);
            hold on
            trplot(T_obj,'color','b','frame','0','length',1.8)
        end
    case 4
        %posicion deseada: obengo elipsoide
        T = transl(0.191,0.126,0.150);
        Q = R.ikine(T, 'mask', [1 1 1 0 0 0]);
        J = (R.jacob0(Q));
        J = J(1:3,:);
        R.maniplty(q_ini)
        figure()
        R.plot(Q);
        hold on
        plot_ellipse(J*J',transl(T),'edgecolor', 'b');
        %R.vellipse(Q, 'fillcolor','b','edgecolor', 'w', 'alpha', 0.5);
        
    case 5
        % Recorrido de posiciones obtenidas mediante función que cumplan
        % que el ultimo eslabon sea horizontal
        q4 = obtenerAngulos(R.qlim(2,:), R.qlim(3,:));
        result = tray_inicial(q4);
        n = height(result); % cantidad de filas
        Tt = zeros(4,4,n);
        % se obtienen n matrices de trayectoria
        for i=1:n
            Tt(:,:,i) = double(R.fkine(result(i,:)));
        end
        
        %Se calcula la inversa para cada matriz que define un punto de la
        %trayectoria
        T_1 = transl(.165,.182,.2)*trotx(deg2rad(90))*troty(deg2rad(45));
        T_2 = transl(.226,.099,.186)*trotx(deg2rad(90))*troty(deg2rad(20));

        Tt(:,:,1) = double(T_1);
        Tt(:,:,1) = double(T_2);
        qq=[0;0;0;0]; % variables articulares iniciales
        for i=1:size(Tt,3)
            [aux, flag] = cinv_geometrica(dh,qq(:,i)', Tt(:,:,i),Base,Tool, Offset);
            qq=[qq aux'];
        end
        QJ=[];  % matriz de angulos
        QJv = [];   % velocidades articulares
        QJa = [];   % aceleraciones articulares

%         %uso de Jtraj
%         for i=1:size(Tt,3)
%             [QQ, QQv, QQa]=jtraj(qq(:,i)',qq(:,i+1)',100);
%             QJ=[QJ;QQ];
%             QJv=[QJv;QQv];
%             QJa=[QJa;QQa];
%         end
%         
%         % graficar trayectoria
%         figure()
%         R.plot(qq(:,1)','scale',.6,'trail', {'r'});
%         hold on;
%         R.plot(QJ,'delay',0.01);

        %% Uso de Ctraj
        % HOMING
        TH = zeros(4,4,5); % maniobra de homing
        TH(:,:,1) = transl(.153,.017,.171)*trotx(deg2rad(90));
        TH(:,:,2) = transl(.158,.017,.163)*trotx(deg2rad(90));
        TH(:,:,3) = transl(.161,.017,.156)*trotx(deg2rad(90));
        TH(:,:,4) = transl(.167,.017,.14)*trotx(deg2rad(90));
        figure();
        R.plot(q_ini,'scale',0.6,'trail', {'r'});
        hold on;
        qqC=q_ini; % inicial
        for i=1:3
            N = 100;
            TC = ctraj(TH(:,:,i), TH(:,:,i+1), N);
            
            for j=1:N
                aux = cinv_geometrica(dh,qqC(j,:), TC(:,:,j),Base,Tool, Offset);
                qqC=[qqC; aux];
            end
        
            
        end
        R.plot(qqC,'delay',0.01,'trail', {'r'});
        hold off
        
        % SEGURIDAD
        TH = zeros(4,4,5); % maniobra de SEGURIDAD
        TH(:,:,1) = transl(.185,.017,.158)*trotx(deg2rad(90));
        TH(:,:,2) = transl(.2,.017,.179)*trotx(deg2rad(90));
        TH(:,:,3) = transl(.210,.017,.202)*trotx(deg2rad(90));
        TH(:,:,4) = transl(.217,.017,.227)*trotx(deg2rad(90));
        figure();
        R.plot(q_ini,'scale',0.6,'trail', {'g'});
        hold on;
        qqC=q_ini; % inicial
        for i=1:3
            N = 100;
            TC = ctraj(TH(:,:,i), TH(:,:,i+1), N);
            
            for j=1:N
                aux = cinv_geometrica(dh,qqC(j,:), TC(:,:,j),Base,Tool, Offset);
                qqC=[qqC; aux];
            end
        
            
        end
        R.plot(qqC,'delay',0.01,'trail', {'g'});
        hold off
        
        % PICK
        TH = zeros(4,4,5); % maniobra de pick
        TH(:,:,1) = transl(.257,.017,.194)*trotx(deg2rad(90));
        TH(:,:,2) = transl(.246,.017,.171)*trotx(deg2rad(90));
        TH(:,:,3) = transl(.239,.017,.160)*trotx(deg2rad(90));
        TH(:,:,4) = transl(.231,.017,.150)*trotx(deg2rad(90));
        TH(:,:,5) = transl(.213,.017,.132)*trotx(deg2rad(90));
        TH(:,:,6) = transl(.192,.017,.117)*trotx(deg2rad(90));
        TH(:,:,7) = transl(.167,.017,.098)*trotx(deg2rad(90));
        figure();
        R.plot(q_ini,'scale',0.6,'trail', {'g'});
        hold on;
        qqC=q_ini; % inicial
        for i=1:6
            N = 100;
            TC = ctraj(TH(:,:,i), TH(:,:,i+1), N);
            
            for j=1:N
                aux = cinv_geometrica(dh,qqC(j,:), TC(:,:,j),Base,Tool, Offset);
                qqC=[qqC; aux];
            end
        
            
        end
        R.plot(qqC,'delay',0.01,'trail', {'g'});
        hold off
        
        % PALCE
        TH = zeros(4,4,5); % maniobra de place
        TH(:,:,1) = transl(.252,.017,.092)*trotx(deg2rad(90));
        TH(:,:,2) = transl(.288,.017,.128)*trotx(deg2rad(90));
        TH(:,:,3) = transl(.301,.017,.151)*trotx(deg2rad(90));
        TH(:,:,4) = transl(.310,.017,.175)*trotx(deg2rad(90));
        TH(:,:,5) = transl(.316,.017,.133)*trotx(deg2rad(90));
        figure();
        R.plot(q_ini,'scale',0.6,'trail', {'m'});
        hold on;
        qqC=q_ini; % inicial
        for i=1:4
            N = 100;
            TC = ctraj(TH(:,:,i), TH(:,:,i+1), N);
            
            for j=1:N
                aux = cinv_geometrica(dh,qqC(j,:), TC(:,:,j),Base,Tool, Offset);
                qqC=[qqC; aux];
            end
        
            
        end
        R.plot(qqC,'delay',0.01,'trail', {'m'});
        hold off
        
        %% Ctraj: DEMO
        N = 100;
        TC = ctraj(T_2, T_1, N);
        qqC=q_ini; % inicial
        for i=1:N
            aux = cinv_geometrica(dh,qqC(i,:), TC(:,:,i),Base,Tool, Offset);
            qqC=[qqC; aux];
        end
        figure();
        R.plot(q_ini,'scale',0.6,'trail', {'b'});
        hold on;
        R.plot(qqC,'delay',0.01,'trail', {'b'});

        
    case 6
        figure()
        R.plot(q_ini,'trail', {'r'})
        hold on
        R2.plot(q_ini, 'trail', {'r'})
end
