% Cinemática directa - Comparación
syms q1 q2 q3 q4 L1 L2 L3 L4 real
syms x_x y_x z_x x_y y_y z_y x_z y_z z_z X Y Z zero one
dh = [0 L1  0 pi/2   0;
      0 0   L2  0     0;
      0 0   L3  0     0;
      0 0   L4  0     0];
  
R=SerialLink(dh,'name','Brazo Principal MK2');

%         q_ini = [1 1 1 0]; % [q1 q2 q3 0]
q_ini = sym('q', [1 4])
% q_ini = [0.4520    0.6458   -1.5865    0.0017];
T_obj = [x_x y_x z_x X;
        x_y y_y z_y Y;
        x_z y_z z_z Z;
        zero zero zero one]
fprintf('Matriz Homogénea usando función de Toolbox: \n')
Ttoolbox_ini = R.fkine(q_ini)

fprintf('Matriz Homogénea usando función propia: \n')
[T_ini,T1,q,flag] = c_dir(q_ini, dh, R);
T_ini = simplify(T_ini)

T01 = T1(:,:,1); % inv: inv(T1(:,:,1))
T12 = T1(:,:,2);
T23 = T1(:,:,3);
T34 = T1(:,:,4);

% 2: T = T0 * T1 * T2 * T3 * Tool
%         T1_inv * T_ini = T1 * T2 * T3 * Tool;
p14 =  simplify(T01 \ T_obj);
pm14 = simplify(T12 * T23 * T34);
fprintf("Con la inversa de la matriz T01:\n");
fprintf("T01_inv * T_ini = T12 * T23 * T34\n");
fprintf("\n");
for i = 1:4
    for j = 1:4
        fprintf("%s == %s\n", p14(i,j), pm14(i,j));
    end
end
fprintf("De ec. 9: %s == %s\n", p14(1,3), pm14(1,3));
fprintf("\n\n\n");

% fprintf("           X*sin(q1) == Y*cos(q1) \n");
% fprintf("           Y/X == tan(q1) \n");

p24 =  simplify(T12 \ p14);
pm24 = simplify(T23 * T34);
fprintf("Con la inversa de la matriz T12:\n");
fprintf("T12_inv * T01_inv * T_ini = T23 * T34\n");
fprintf("\n");
for i = 1:4
    for j = 1:4
        fprintf("%s == %s\n", p24(i,j), pm24(i,j));
    end
end

fprintf("De ec. 3: %s == %s\n", p24(1,3), pm24(1,3));
fprintf("\n\n\n")
% fprintf("z_z*cos(q2) == z_x*cos(q1)*sin(q2) + z_y*sin(q1)*sin(q2)\n");
% fprintf("z_z*cos(q2) == sin(q2) * (z_x*cos(q1) + z_y*sin(q1))\n");
% fprintf("z_z/R*cos(q1-ϕ) == sin(q2)/cos(q2)\n");
% fprintf("z_z/R*cos(q1-ϕ) == tg(q2);R = sqr(z_x^2+z_y^2); ϕ = arctan(z_y/z_x)\n\n\n");

p34 =  simplify(T23 \ p24);
pm34 = simplify(T34);
fprintf("Con la inversa de la matriz T23:\n");
fprintf("T23_inv * T12_inv * T01_inv * T_ini = T34\n");
fprintf("\n");
for i = 1:4
    for j = 1:4
        fprintf("%s == %s\n", p34(i,j), pm34(i,j));
    end
end
fprintf("De ec. 7: %s == %s\n", p34(2,3), pm34(2,3));
fprintf("\n\n\n")
% fprintf("De ec. 3: %s == %s\n", p34(2,3), pm34(2,3)); %juno a 1,3
% fprintf("Ec.4,1 de T1_inv * B_inv * T_ini = T2 * T3 * T4 * Tool;\nZ*cos(q2) - L2*one - X*cos(q1)*sin(q2) - Y*sin(q1)*sin(q2) - L1*one*cos(q2) == L4*sin(q3 + q4) + L3*sin(q3)\n");
% fprintf("Z*cos(q2) - L2 - X*cos(q1)*sin(q2) - Y*sin(q1)*sin(q2) - L1*cos(q2) == L4*sin(q3 + q4) + L3*sin(q3); q4 = pi/2 - q3\n");
% fprintf("Z*cos(q2) - L2 - L1*cos(q2) - X*cos(q1)*sin(q2) - Y*sin(q1)*sin(q2)== L4*sin(pi/2) + L3*sin(q3)\n");
% fprintf("(Z*cos(q2) - L2 - L4 - L1*cos(q2) - R*sin(q2)*cos(q1−ϕ))/L3 == sin(q3) ; con R = sqr(X^2+Y^2) ϕ = arctan(Y/X)\n")
