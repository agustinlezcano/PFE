function [T,q,flag] = c_dir(q1,dh, R)   
Base=R.base.double;
Tool=R.tool.double;
offset=R.offset;
lim = R.qlim;

if (length(q1) == 4)
        flag=true;
    
    valida_angulos(q1,lim);     
    T = zeros(4,4,4); %1 T homogenea por gdl

    dh(:,1) = q1;
    for j=1:4
        T(:,:,j) = A_dh(dh(j,:), (dh(j,1)+offset(j)));
    end

    T_total2 = Base * T(:,:,1);
    for k=2:4
        T_total2(:,:) = T_total2(:,:)*T(:,:,k);
    end

    T =  T_total2 * Tool;
    q=q1;

    return;
else
    disp('Longitud errónea, debe ser igual a 4\n')
end

end


