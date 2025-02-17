%Esta funcion toma los parámetros Denavit-Hartemberg y una Matriz de
%transformación deseada y devuelve el set de posiciones articulares q (no
%motor)

function [qf] = cinv_geometrica(dh,q_ini,T_obj,Base,Tool, offset)

% Desacopar herramienta y base
T_obj = Base \ T_obj * inv(Tool);

fprintf('q final para prueba: \n')
rad2deg(q_ini)
fprintf('Matriz Homogénea usando función de Toolbox: \n')

%Separar muñeca de extremo
Pf = T_obj(1:3,4);
% Normal del plano XY (plano del suelo)
% normal_XY = [0; 0; 1]; % Normal en dirección Z

% Verificar si el versor Z del efector final es paralelo al plano XY
% tol = 1e-3; % Tolerancia numérica
% for i=1:3
%     is_parallel = check_parallel_to_plane(Te, i, normal_XY, tol);
%     if(is_parallel == 0)
%         break;
%     end
% end
    
%Pm = Pf - dh(4,3) * T_obj(1:3,i); %posicion de la muñeca, resto el parametro L4
Pm = Pf - dh(4,3) * T_obj(1:3,1); %posicion de la muñeca, resto el parametro L4
% CONFIGURACIÓN ALCANZABLE: Codo arriba - derecha (limitaciones fisicas)
%% q1
q1 = atan2(Pm(2), Pm(1));

%% q2
T01 = A_dh(dh(1,:),q1);
p2m = invHomog(T01) * [Pm;1];
B = atan2(p2m(1), p2m(2));
d = sqrt(p2m(1)^2 + (p2m(2))^2); %hipotenusa P12 y Pm
C = acos((d^2 + dh(2,3)^2 - dh(3,3)^2) / (2*d*dh(2,3)));
%Gral: B +/- C
%configuracion codo arriba : B-C
q2 = pi/2 - (B - real(C)); % el pi/2 es el offset de dh(1,4)

%% q3
phi = acos((dh(2,3)^2 + dh(3,3)^2 - d^2) / (2*dh(2,3)*dh(3,3))); % ang. entre L2 y L3
q3 = -(pi - phi); % Valido para codo arriba

%% q4 (angulo ficticio)
T1 = A_dh(dh(1,:), q1(1));
T2 = T1 * A_dh(dh(2,:), q2);
p2f = T2 \ [Pf; 1];
r = sqrt(p2f(1)^2 + p2f(2)^2); %distancia euclidea entre p2 y Extremo

gamma = acos((dh(4,3)^2 + dh(3,3)^2 - r^2) / (2*dh(4,3)*dh(3,3)));
q4 = pi - gamma;

qf = real([q1 q2 q3 q4]) - offset;
rad2deg(qf)
end
