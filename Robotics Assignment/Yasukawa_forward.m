function [T_e]= Yasukawa_forward(d1, d4, d6, a2, THETA)
   THETA=[0.1 0.2 0.3 0.4 0.5 0.6];
   d1=1;
   d4=2;
   d6=3;
   a2=5;
   
   D=[d1 0 0 d4 0 d6];
   A=[0 a2 0 0 0 0];
   ALPA=[pi/2 0 pi/2 pi/2 pi/2 0];
   
   syms theta;
   syms alpa;
   syms Tz;
   syms Tx
   syms T_trans;
   syms a;
   syms d;
   
   Tz(theta)=[cos(theta) -sin(theta) 0 0;
              sin(theta) cos(theta) 0 0;
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
   T_e=vpa(T_e,3)
  
end