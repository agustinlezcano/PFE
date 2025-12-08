clear; clc; close all;
DH_Robot %importa el robot y todos los parametros DH

physical_workspace = readmatrix('Tabla_angulos_Robot.csv', 'Delimiter', ';'); %Filas q3m, Columnas q2m

[N_q3m_max, N_q2m_max] = size(physical_workspace);

% Guardo los valores xyz
N = N_q3m_max*N_q2m_max;
xyz = zeros(N,5); %puntos XYZ columnas 1 a 3, q2m y q3m columnas 4 y 5

tabla_ang = zeros(N_q3m_max-1, N_q2m_max-1);

i_total = 1;
for j=1:N_q2m_max
    for i=1:N_q3m_max
        value = physical_workspace(i,j);
        if i>1 && j>1 && value ~=0 %recorre valores de la tabla !=0 ya que los 0 son limites fisicos
            q2m = physical_workspace(1,j) * pi/180; %se lee el valor y se pasa a radianes
            q3m = physical_workspace(i,1) * pi/180;

            q1 = 0;
            q2 = pi/2 - q2m;
            q3 = -(q2 + q3m);
            q4 = -(q2 + q3);  

            % Cinemática directa
            T = double(R.fkine([q1 q2 q3 q4]));
            p = T(1:3,4) - [0;0;tool_z];  % p = [x;y;z]'

            if p(3)>0 %Guardar valores de z positivos
                xyz(i_total,1:3) = p';
                xyz(i_total,4:5) = [q2m q3m];
                tabla_ang(i-1, j-1) = 1; 
                %se va a agregar a la tabla leida, poner en 0 cuando z<0
            end
            i_total = i_total + 1;
        end
    end
end
 
%Descomentar siguiente linea para cuando queramos exportar la tabla a excel
%OJO! delta_q debe ser 1°
%writematrix(tabla_ang, 'matriz.csv', 'Delimiter', ';'); %exporta csv para importarlo al excel de angulos

figure;
plot(xyz(:,1), xyz(:,3), '.');
xlabel('X');
ylabel('Z');
title('Espacio de trabajo - plano XZ');
axis equal; grid on;