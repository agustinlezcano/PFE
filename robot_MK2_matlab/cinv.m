function [q_obj, flag, qqs] = cinv(q_ant, dh, T, R)

T = invHomog(R.base.double) * T * invHomog(R.tool.double);

Pm = T(1:3,4); %posicion final deseada

%4 soluciones  --> revisar
qq=zeros(3,4);
 
%% q1

q1(1) = atan2(Pm(2), Pm(1));
%q1(2) = -atan2(Pm(2), Pm(1));

if q1(1) > 0
    q1(2) = q1(1) - pi; 
else
    q1(2) = q1(1) + pi; 
end

qq(1,1:4) = [1 1 1 1] * q1(1);
qq(1,5:8) = [1 1 1 1] * q1(2);
%% q2

T1 = A_dh(dh(1,:), q1(1));
p21 = invHomog(T1) * [Pm; 1];
B = atan2(p21(2), p21(1));
r = sqrt(p21(1)^2 + p21(2)^2);
L2 = dh(2, 3); % a2
L3 = sqrt(dh(4, 2)^2+dh(3,3)^2); % hipotenusa entre d4 y a3
G = (acos((L2^2 + r^2 - L3^2) / (2 * r * L2)));

q2(1) = B - real(G);
q2(2) = B + real(G);
img1=~isreal(G);

T1 = A_dh(dh(1,:), q1(2));
p22 = invHomog(T1) * [Pm; 1];
B = atan2(p22(2), p22(1));
r = sqrt(p22(1)^2 + p22(2)^2);
G = (acos((L2^2 + r^2 - L3^2) / (2 * r * L2)));

q2(3) = B - real(G);
q2(4) = B + real(G);
img2=~isreal(G);


qq(2,:)=[q2(1) q2(2) q2(3) q2(4)];

%% q3

ang=atan(dh(3,3)/dh(4,2)); %angulo fijo del triangulo entre d4 y a3
for i=1:8
    T1 = A_dh(dh(1,:), qq(1,i));
    T2 = T1 * A_dh(dh(2,:), qq(2,i));
    p3 = invHomog(T2) * [Pm; 1];
    qq(3,i) = atan2(p3(2), p3(1)) - pi/2 + ang;
end

%sacar offset
qq = qq - R.offset' * ones(1,4);

%% se desarrolla ordenar las soluciones en funcion a menores distancia y dando primero las opciones reales.
qdis=zeros(1,4);
num=[1,2,3,4];
qqimg=zeros(1,4);

if (img1==1)
    qqimg(1:2)=1;
end

if (img2==1)
    qqimg(3:4)=1;
end

for i=1:4
qdis(i)=sum(abs(qq(:,i)'-q_ant));
end


qposibles=[qdis;qqimg;num];%primer fila distancia, segunda fila 1 si es img 0 si es real, tercer fila numero de solucion.

%ordeno columnas en funcion primer fila para que quede de menor a mayor distancia
%luego ordeno en funcion de segunda fila para dejar valores imaginarios al final 
[~,data]=sort(qposibles(1,:));
qposibles=qposibles(:,data);

[~,data]=sort(qposibles(2,:));
qposibles=qposibles(:,data);
qqs=[];
for i=1:4
qqs=[qqs qq(:,qposibles(3,i))];
end
indice=qposibles(3,1);

q_obj=qq(:,indice)';

%% revisa si la respuesta es posible o no en funcion de si es imaginaria
fl=qposibles(2,1);
if (fl==1)
    flag=0; 
    fprintf('La posicion final  esta fuera del espacio de trabajo\n se ha dado la respuesta mas cercana\n\n')
else
    flag=1;
    fprintf('La posicion esta dentro del espacio de trabajo\n\n')
end
end

%% funciones auxiliares
function T= A_dh(dh,q)
T=trotz(q)*transl(dh(3),0,dh(2))*trotx(dh(4));
end

function iT = invHomog(T)
iT = eye(4);
iT(1:3, 1:3) = T(1:3, 1:3)';
iT(1:3, 4) = - iT(1:3, 1:3) * T(1:3, 4);
end