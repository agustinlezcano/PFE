function [T,T1,q,flag] = c_dir(q1,dh, R)   
import ETS2.*
Base=R.base.double;
Tool=R.tool.double;
offset=R.offset;

if (length(q1) == 4)
    flag=true; 
   
    %1
%      if q1(1)>deg2rad(185)
%         flag=false;
%         fprintf('Error q1 gdl_1 valor superior a 185 el nuevo valor será 185\n')
%         q1(1)=deg2rad(185);
%      end
% 
%      if q1(1)<-deg2rad(185)
%         flag=false;
%         fprintf('Error q1 gdl_1 valor menor a -185 el nuevo valor será -185\n')
%         q1(1)=deg2rad(-185);
%      end
%      
%      %2
%       if q1(2)<-deg2rad(65)
%         flag=false;
%         fprintf('Error q1 gdl_2 valor menor a -65 el nuevo valor será -65\n')
%         q1(2)=deg2rad(-65);
%       end
% 
%      if q1(2)>deg2rad(185)
%         flag=false;
%         fprintf('Error q1 gdl_2 valor mayor a 185 el nuevo valor será 185\n')
%         q1(2)=deg2rad(185);
%      end
%      
%      %3
%       if q1(3)<-deg2rad(175)
%         flag=false;
%         fprintf('Error q1 gdl_3 valor menor a -175 el nuevo valor será -175\n')
%         q1(2)=deg2rad(-175);
%       end
% 
%      if q1(3)>deg2rad(138)
%         flag=false;
%         fprintf('Error q1 gdl_3 valor mayor a 185 el nuevo valor será 138\n')
%         q1(3)=deg2rad(138);
%      end
%      
%      %4
%       if q1(4)<-deg2rad(165)
%         flag=false;
%         fprintf('Error q1 gdl_4 valor menor a -165 el nuevo valor será -165\n')
%         q1(4)=deg2rad(-165);
%       end
% 
%      if q1(4)>deg2rad(165)
%         flag=false;
%         fprintf('Error q1 gdl_4 valor mayor a 165 el nuevo valor será 165\n')
%         q1(4)=deg2rad(165);
%      end
     
%      T1 = zeros(4,4,4); %1 T homogenea por gdl
     T1 = sym('a', [4 4 4]);

    dh(:,1) = q1;
    for j=1:4
        T1(:,:,j) = trotz(dh(j,1)+ offset(j))*transl(dh(j,3),0,dh(j,2))*trotx(dh(j,4));
        
%         Tz = [1 0 0 0;
%               0 1 0 0;
%               0 0 1 dh(j,2);
%               0 0 0 1];
%           
%         Tx = [1 0 0 dh(j,3);
%               0 1 0 0;
%               0 0 1 0;
%               0 0 0 1];
%           
%         T1(:,:,j) = Rz(dh(j,1)+ offset(j))* Tz * Tx * Rx(dh(j,4));
    end
    
    T_total = Base * T1(:,:,1);
    for k=2:4
        T_total(:,:) = T_total(:,:)*T1(:,:,k);
    end
 
    T =  T_total * Tool;
%     T = simplify(T);
    q=q1;
    
    return;
else
    disp('Longitud errónea, debe ser igual a 6\n')
end

end