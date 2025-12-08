clear; clc; close all;
DH_Robot %importa el robot y todos los parametros DH

delta_q = 1*pi/180;   % paso angular

% Límites de motores [rad]
q2m_lim = [60 143]*pi/180;
q3m_lim = [-20  65 ]*pi/180;

q2m_vec = q2m_lim(1) : delta_q : q2m_lim(2);
q3m_vec = q3m_lim(1) : delta_q : q3m_lim(2);

% Preasignación para tabla angular
tabla_ang = zeros(numel(q3m_vec), numel(q2m_vec));
N = numel(q2m_vec) * numel(q3m_vec);
xyz = zeros(N,5); %puntos XYZ columnas 1 a 3, q2m y q3m columnas 4 y 5

i_col = 1;
i_total = 1;
for q2m = q2m_vec
    q1 = 0;
    q2 = pi/2 - q2m;
    j_row = 1;
    for q3m = q3m_vec
        q3 = -(q2 + q3m);
        q4 = -(q2 + q3);  

        % Cinemática directa
        T = double(R.fkine([q1 q2 q3 q4]));
        p = T(1:3,4) - [0;0;tool_z];  % p = [x;y;z]'
        
        if p(3)>=0 %Guardar valores de z positivos
            xyz(i_total,1:3) = p';
            xyz(i_total,4:5) = [q2m q3m];
            tabla_ang(j_row, i_col) = 1;
        end
        
        j_row = j_row + 1;
        i_total = i_total + 1;
    end
    i_col = i_col + 1;
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

