DH_Robot %importa el robot y todos los parametros DH

%% ANGULOS INICIALES
T0 = double(R.fkine([0 0 0 0])); % 0 90 0 son los angulos iniciales
T_real0 = T0;
T_real0(3,4) = T_real0(3,4) - tool_z

q_obj = deg2rad([45 -10 -10 20]);
T_obj = double(R.fkine(q_obj));
T_obj(1:3, 1:3) = eye(3);
T_obj
delta_cm = (T0(1:3,4) - T_obj(1:3,4))*100;

T_real1 = T_obj;
T_real1(3,4) = T_real1(3,4)-tool_z 
%MATLAB: Mi posicion objetivo esta sin tool
%Lo resolvemos así y despues le restamos la tool para verificar

%STM32: A mi posicion objetivo le tengo que sumar
%la altura de tool para que lo resuelva sin tool

[qf, flag] = cinv_geometrica(dh,q_obj, T_obj,Base,Tool,Offset,R.qlim);

if (~flag)
    fprintf('Posicion no alcanzable\n');
else
    figure()
    R.plot(qf);
    hold on
    trplot(T_obj,'color','b','frame','0','length',1.8)
end
