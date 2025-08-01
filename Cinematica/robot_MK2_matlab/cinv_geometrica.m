%Esta funcion toma los parámetros Denavit-Hartemberg y una Matriz de
%transformación deseada y devuelve el set de posiciones articulares q (no
%motor)

function [qf, flag] = cinv_geometrica(dh,q_ini,T_obj,Base,Tool, offset, lim)
%q_ini: arreglo de angulos inciales

% Desacopar herramienta y base
T_obj = Base \ T_obj * inv(Tool);

fprintf('Matriz Homogénea usando función de Toolbox: \n')

%Separar muñeca de extremo
Pf = T_obj(1:3,4);
% Normal del plano XY (plano del suelo)
% normal_XY = [0; 0; 1]; % Normal en dirección Z

ang_base = atan2(Pf(2), Pf(1));

versor = [cos(ang_base); sin(ang_base); 0];
    
%Pm = Pf - dh(4,3) * T_obj(1:3,i); %posicion de la muñeca, resto el parametro L4
Pm = Pf - dh(4,3) * versor; %posicion de la muñeca, resto el parametro L4
% CONFIGURACIÓN ALCANZABLE: Codo arriba - derecha (limitaciones fisicas)
%% q1
q1 = ang_base;

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
theta_m2 = pi/2 - q2;   %Eje horizontal hacia la izquierda 
theta_m3 = - (q2 + q3); %Eje horizontal hacia la izquierda 

flag = valida_angulos(qf,lim); %en uC: salida; qf en memoria (angulos) --> modificar ang. motor
end
