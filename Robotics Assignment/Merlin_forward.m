%function to calculate forward kinematics of Merlin Robot
function [T_e]= Merlin_forward(d1, d2, a2, a3, THETA)
   THETA=[0.1 0.2 0.3 0.4 0.5 0.6];
   d1=1;
   d2=2;
   a2=5;
   a3=6;
   
   D=[d1 d2 0 0 0 0];
   A=[0 a2 a3 0 0 0];
   ALPA=[pi/2 0 0 pi/2 0 0];
   
   syms theta;
   syms alpa;
   syms Tz;
   syms Tx
   syms T_trans;
   syms a;
   syms d;
   
   Tz(theta)=[cos(theta) -sin(theta) 0 0;
              sin(theta) cos(theta) 0 0
              0 0 1 0;
              0 0 0 1];
   Tx(alpa)=[1 0 0 0;
              0 cos(alpa) -sin(alpa) 0;
              0 sin(alpa) cos(alpa) 0;
              0 0 0 1];
   T_trans(d,a)=[1 0 0 a;
                 0 1 0 0;
                 0 0 1 d;
                 0 0 0 1];
   for i=1:6
       T(:,:,i) = Tz(THETA(i))*T_trans(D(i),A(i))*Tx(ALPA(i));
       if(i==1)
           T_e=T(:,:,i);
       else
            T_e=T_e*T(:,:,i);
       end
   end  
   T_e=vpa(T_e,3);
  
end
