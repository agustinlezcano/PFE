function T= A_dh(dh,q)
T=trotz(q)*transl(dh(3),0,dh(2))*trotx(dh(4));
end