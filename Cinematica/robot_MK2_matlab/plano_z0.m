DH_Robot %importa el robot y todos los parametros DH

%% Calculo de plano z=0 por método iterativo optimizado

delta_q = 1*pi/180;   % paso angular
tol = 0.001; %tolerancia de altura [m]

% Límites de motores [rad]
q2m_lim = [120 150]*pi/180;
q3m_lim = [30  65 ]*pi/180;

q2m_vec = q2m_lim(1) : delta_q : q2m_lim(2);
q3m_vec = q3m_lim(1) : delta_q : q3m_lim(2);

N = numel(q2m_vec) * numel(q3m_vec);
xyz = zeros(N, 3);
idx = 1;

tabla_ang = zeros(numel(q2m_vec), numel(q3m_vec));
fila = 1;
col = 1;

for q2m = q2m_vec
    q1 = 0;
    q2 = pi/2 - q2m;
    
    for q3m = q3m_vec
        q3 = -(q2 + q3m);
        q4 = -(q2 + q3);

        % Cinemática directa
        T = double(R.fkine([q1 q2 q3 q4]));
        p = T(1:3,4) - [0;0;tool_z];

        xyz(idx,:) = p';
        idx = idx + 1;
        
        if p(3) > tol 
            tabla_ang(fila, col) = 1;
        end
        col = col + 1;
        
        if p(3) >= -tol && p(3) <= tol
            %fprintf("xyz=(%.4f %.4f %.4f) q2m=%.4f q3m=%.4f\n", p(1), p(2), p(3),q2m*180/pi,q3m*180/pi);
        end
    end
    col=1;
    fila=fila+1;
end

tabla_ang = tabla_ang'; 
%Descomentar siguiente linea para cuando queramos exportar la tabla a excel
%(ojo con el angulo delta_q
%writematrix(tabla_ang, 'matriz.csv', 'Delimiter', ';'); %exporta csv para importarlo al excel de angulos

% Filtrado por plano z = 0
xyz_filtered = xyz( xyz(:,3) >= -tol & xyz(:,3) <= tol , : );
min_radius = min(xyz_filtered(:,1));
max_radius = max(xyz_filtered(:,1));
fprintf("Radio minimo = %.5f\nRadio maximo = %.5f\n", min_radius, max_radius);
fprintf("La ecuacion de estos arcos seria:\nx^2 + y^2 = %.5f\nx^2 + y^2 = %.5f\n",min_radius^2, max_radius^2);
%% Ploteos

%Ploteo de arcos que representan el area de trabajo del robot
%Surge a partir del radio minimo y maximo (a lo largo del eje X)
circr = @(radius,rad_ang)  [radius*cos(rad_ang);  radius*sin(rad_ang)]; % Circle Function For Angles In Radians
N = 200;                                            % Number Of Points In Complete Circle
r_angl = linspace(0, pi, N);                        % Angle Defining Arc Segment (radians)
xy_r1 = circr(min_radius, r_angl);                  % Matrix (2xN) Of (x,y) Coordinates
xy_r2 = circr(max_radius, r_angl);                  % Matrix (2xN) Of (x,y) Coordinates
figure(1)
plot(xy_r1(1,:), xy_r1(2,:), '.b')                  % Draw An Arc Of Blue Stars
hold on
plot(xy_r2(1,:), xy_r2(2,:), '.r')                  % Draw An Arc Of Blue Stars
axis([-max_radius  max_radius    0  max_radius])             % Set Axis Limits
axis equal                                          % No Distortion With ‘axis equal’