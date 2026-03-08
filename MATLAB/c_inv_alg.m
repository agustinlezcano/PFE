function q = c_inv_alg(T_obj, dh)
% T_obj = [x_x y_x z_x X;
%         x_y y_y z_y Y;
%         x_z y_z z_z Z;
%         0 0 0 1]

%% Solucion q1
% Y/X == tan(q1) --> atan2
q1 = real(atan2(T_obj(2,4), T_obj(1,4)));

%% Solucion q2
% z_z/R*cos(q1-ϕ) == tg(q2); 

% ϕ = arctan(z_y/z_x)
phi = atan2(T_obj(2,3), T_obj(1,3));
% R = sqrt(z_x^2+z_y^2);
R = sqrt(T_obj(1,3)^2+T_obj(2,3)^2);

q2 = real(atan2(T_obj(3,3), R*cos(q1-phi)));
%% Solucion q3
% (Z*cos(q2) - L2 - L4 - L1*cos(q2) - R*sin(q2)*cos(q1−ϕ))/L3 == sin(q3) ;

% ϕ = arctan(Y/X)
phi_2 = atan2(T_obj(2,4), T_obj(1,4));
% R = sqrt(X^2+Y^2)
R_2 = sqrt(T_obj(1,4)^2+T_obj(2,4)^2);

% (Z*cos(q2) - L2 - L4 - L1*cos(q2) - R*sin(q2)*cos(q1−ϕ))/L3 == sin(q3)
% sin puede ser positivo o negativo --> ver relacion 
% TODO: hacer q3a, q3b
q3 = real(asin((T_obj(3,4)*cos(q2) - dh(2,3) - dh(4,3) - dh(1,2)*cos(q2) - R_2*sin(q2)*cos(q1-phi_2))/dh(3,3)));
%% Solucion q4
% q4 = pi/2 -q3
q4 = real(pi/2 - q3);
%% Solucion final
q = [q1 q2 q3 q4];

end