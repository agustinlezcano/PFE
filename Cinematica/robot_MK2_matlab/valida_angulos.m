function flag = valida_angulos(q1,lim)
    flag = true;
    %1
    if q1(1)<lim(1,1)
    fprintf('Error q1 gdl_1 valor menor al límite, el nuevo valor será %s\n',lim(1,1))
    q1(1)=deg2rad(-185);
    flag = false;
    end

    if q1(1)>lim(1,2)
    fprintf('Error q1 gdl_1 valor superior al límite, el nuevo valor será %s\n',lim(1,2))
    q1(1)=deg2rad(185);
    flag = false;
    end

    %2
    if q1(2)<lim(2,1)
    fprintf('Error q1 gdl_2 valor menor al límite,el nuevo valor será %s\n',lim(2,1))
    q1(2)=deg2rad(-65);
    flag = false;
    end

    if q1(2)>lim(2,2)
    fprintf('Error q1 gdl_2 valor mayor al límite, el nuevo valor será %s\n',lim(2,2))
    q1(2)=deg2rad(185);
    flag = false;
    end

    %3
    if q1(3)<lim(3,1)
    fprintf('Error q1 gdl_3 valor menor al límite, el nuevo valor será %s\n',lim(3,1))
    q1(2)=deg2rad(-175);
    flag = false;
    end

    if q1(3)>lim(3,2)
    fprintf('Error q1 gdl_3 valor mayor al límite, el nuevo valor será %s\n',lim(3,2))
    q1(3)=deg2rad(138);
    flag = false;
    end

    %4
    if q1(4)<lim(4,1)
    fprintf('Error q1 gdl_4 valor menor al límite, el nuevo valor será %s\n',lim(4,1))
    q1(4)=deg2rad(-165);
    flag = false;
    end

    if q1(4)>lim(4,2)
    fprintf('Error q1 gdl_4 valor mayor al límite, el nuevo valor será %s\n',lim(4,2))
    q1(4)=deg2rad(165);
    flag = false;
    end

end