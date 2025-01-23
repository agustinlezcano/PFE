%Esta funcion toma los parámetros Denavit-Hartemberg y una Matriz de
%transformación deseada y devuelve el set de posiciones articulares q (no
%motor)
%function qf = c_inv_geom(dh,q_inicial, T_objetivo)
L1 =0.057;
L2 =0.135;
L3 =0.147;
L4 =0.147;

dh = [0 L1  0 pi/2   0;
    0 0   L2  0     0;
    0 0   L3  0     0;
    0 0   L4  0     0];

R=SerialLink(dh,'name','Brazo Principal MK2');

% q_ini = [0.5520    0.6458   1.5865    0.0021];
q_ini = [0.4520    0.6458   -1.5865    0.0017];
q_ini = deg2rad([30    10   -100    20])
fprintf('q inicial para prueba: \n')
rad2deg(q_ini)
fprintf('Matriz Homogénea usando función de Toolbox: \n')
T_obj = double(R.fkine(q_ini))

%Separar muñeca de extremo
Pf = T_obj(1:3,4);
Pm = Pf - dh(4,3) * T_obj(1:3,1); %posicion de la muñeca, resto el parametro L4
% CONFIGURACIÓN ALCANZABLE: Codo arriba - derecha (limitaciones fisicas)
%% q1
% q1(1) = atan2(Pm(2), Pm(1));
q1 = atan2(Pm(2), Pm(1));


%% q2
T01 = A_dh(dh(1,:),q1);
p2m = invHomog(T01) * [Pm;1];
B = atan2(p2m(1), p2m(2));
d = sqrt(p2m(1)^2 + (p2m(2))^2);
%d = sqrt(Pm(1)^2 + (Pm(3) - dh(1,2))^2); %hipotenusa P12 y Pm
C = acos((d^2 + dh(2,3)^2 - dh(3,3)^2) / (2*d*dh(2,3)))
%Gral: B +/- C
%configuracion codo arriba : B-C

q2 = pi/2 - (B - real(C)); % el pi/2 es el offset de dh(1,4)
%% q3
% se debe corregir el phi para que se pueda resolver
phi = acos((dh(2,3)^2 + dh(3,3)^2 - d^2) / (2*dh(2,3)*dh(3,3))); % ang. entre L2 y L3
rad2deg(phi)
% q3 deberia ser considerado segun al direccion
% el signo(-) lo agrego para que coincida --> a veces es con y otras sin signo
% para phi mayor a 180 se debe cambiar de signo
%corregir para multiplos
% q3 = -(pi - phi)
if (phi>0 && phi<pi)
    q3 = -(pi - phi)
else
    q3 = (pi - phi)
end
T1 = A_dh(dh(1,:), q1(1));
T2 = T1 * A_dh(dh(2,:), q2);
p2f = T2 \ [Pf; 1];

%distancia euclidea entre p2 y Extremo
r = sqrt(p2f(1)^2 + p2f(2)^2)

gamma = acos((dh(4,3)^2 + dh(3,3)^2 - r^2) / (2*dh(4,3)*dh(3,3)))
q4 = pi - gamma;

q_f = real([q1 q2 q3 q4])
rad2deg(q_f)
%end

%% funciones auxiliares
function T= A_dh(dh,q)
T=trotz(q)*transl(dh(3),0,dh(2))*trotx(dh(4));
end

function iT = invHomog(T)
iT = eye(4);
iT(1:3, 1:3) = T(1:3, 1:3)';
iT(1:3, 4) = - iT(1:3, 1:3) * T(1:3, 4);
end