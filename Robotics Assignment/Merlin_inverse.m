%function to calculate inverse kinematics of Merlin Robot
function [THETA] = Merlin_inverse()
  d1=1;
  d2=2;
  a2=5;
  a3=6;
  D=[d1 d2 0 0 0 0];
  A=[0 a2 a3 0 0 0];
  ALPA=[pi/2 0 0 pi/2 0 0];
  T_des=[ 0.37 -0.506  0.779   10.3;
         -0.859 -0.507 0.0782 -0.975;
          0.355 -0.698 -0.622   4.87;
          0      0      0    1.0];

  syms theta;
  syms Tz;
  syms Tx;
  syms T_trans;
  syms alpa;
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

  %coordinates of common point P
  P=T_des*[0 0 0 1]';
  Xp=P(1);
  Yp=P(2);
  Zp=P(3);  
  
  if(Xp^2+Yp^2-d2^2<0)
      fprintf("Error. The configuration cannot be reached");
  end
  
  %formulae
  zeta= Zp-d1;
  eta(1)=sqrt(Xp^2+Yp^2-d2^2);
  eta(2)=-sqrt(Xp^2+Yp^2-d2^2);
  
  c3=(zeta^2+eta(1)^2-a3^2-a2^2)/(2*a2*a3);
  if(c3>1 || c3<-1)
      fprintf("Error. The configuration cannot be reached");
  end
  
  theta3(1)=acos(c3);
  theta3(2)=2*pi-acos(c3);
  
  s3(1)=sin(theta3(1));
  s3(2)=sin(theta3(2));
  
  c2(1)=1/(eta(1)^2+zeta^2)*((a3*c3+a2)*eta(1)+a3*s3(1)*zeta);
  s2(1)=1/(eta(1)^2+zeta^2)*(-a3*s3(1)*eta(1)+(a3*c3+a2)*zeta);
  
  c2(2)=1/(eta(1)^2+zeta^2)*((a3*c3+a2)*eta(2)+a3*s3(1)*zeta);
  s2(2)=1/(eta(1)^2+zeta^2)*(-a3*s3(1)*eta(2)+(a3*c3+a2)*zeta);
  
  c2(3)=1/(eta(1)^2+zeta^2)*((a3*c3+a2)*eta(1)+a3*s3(2)*zeta);
  s2(3)=1/(eta(1)^2+zeta^2)*(-a3*s3(2)*eta(1)+(a3*c3+a2)*zeta);
  
  c2(4)=1/(eta(1)^2+zeta^2)*((a3*c3+a2)*eta(2)+a3*s3(2)*zeta);
  s2(4)=1/(eta(1)^2+zeta^2)*(-a3*s3(2)*eta(2)+(a3*c3+a2)*zeta);
  
  theta2(1)=atan2(s2(1),c2(1));  % for theta 31
  theta2(2)=atan2(s2(2),c2(2));  % for theta 31
  theta2(3)=atan2(s2(3),c2(3));  % for theta 32
  theta2(4)=atan2(s2(4),c2(4));  % for theta 32
  
  c1(1)=1/(eta(1)^2+d2^2)*(((a3*c3(1)+a2)*c2(1)-a3*s3(1)*s2(1))*Xp-d2*Yp);
  s1(1)=1/(eta(1)^2+d2^2)*(d2*Xp+((a3*c3(1)+a2)*c2(1)-a3*s3(1)*s2(1))*Yp);
  
  c1(2)=1/(eta(1)^2+d2^2)*(((a3*c3(1)+a2)*c2(2)-a3*s3(1)*s2(2))*Xp-d2*Yp);
  s1(2)=1/(eta(1)^2+d2^2)*(d2*Xp+((a3*c3(1)+a2)*c2(2)-a3*s3(1)*s2(2))*Yp);
  
  c1(3)=1/(eta(1)^2+d2^2)*(((a3*c3+a2)*c2(3)-a3*s3(2)*s2(3))*Xp-d2*Yp);
  s1(3)=1/(eta(1)^2+d2^2)*(d2*Xp+((a3*c3+a2)*c2(3)-a3*s3(2)*s2(3))*Yp);
  
  c1(4)=1/(eta(1)^2+d2^2)*(((a3*c3+a2)*c2(4)-a3*s3(2)*s2(4))*Xp-d2*Yp);
  s1(4)=1/(eta(1)^2+d2^2)*(d2*Xp+((a3*c3+a2)*c2(4)-a3*s3(2)*s2(4))*Yp);
  
  theta1(1)=atan2(s1(1),c1(1));
  theta1(2)=atan2(s1(2),c1(2));
  theta1(3)=atan2(s1(3),c1(3));
  theta1(4)=atan2(s1(4),c1(4));
  
  THETA=[theta1(1) theta2(1) theta3(1) 0 0 0;
         theta1(2) theta2(2) theta3(1) 0 0 0;
         theta1(3) theta2(3) theta3(2) 0 0 0;
         theta1(4) theta2(4) theta3(2) 0 0 0];
     
     
 %Now to find theta4,5,6
 for i=1:4
        T1_0(:,:,i)= Tz(THETA(i,1))*T_trans(d1,0)*Tx(pi/2);
        T2_1(:,:,i)= Tz(THETA(i,2))*T_trans(d2,a2);
        T3_2(:,:,i)= Tz(THETA(i,3))*T_trans(0,a3);
        T3_0(:,:,i)= T1_0(:,:,i)*T2_1(:,:,i)*T3_2(:,:,i);
        T3_0_inv(:,:,i)=inv(T3_0(:,:,i));
        T_hat_des(:,:,i)= T3_0_inv(:,:,i)*T_des;
 end
 vpa(T_hat_des,3)
 for i=1:4
       c4(i)=-T_hat_des(2,3,i);
       s4(i)= T_hat_des(1,3,i);
       
       theta4(i)=atan2(s4(i),c4(i));
       
       %theta56= theta5+theta6, c56=cos(theta5+theta6),
       %s56=sin(theta5+theta6)
       
       c56(i)= T_hat_des(3,2,i);
       s56(i)= T_hat_des(3,1,i);
       
       theta56(i,1)=atan2(s56(i),c56(i));   
 end  
   THETA=[theta1(1) theta2(1) theta3(1) theta4(1) theta56(1) ;
         theta1(2) theta2(2) theta3(1) theta4(2) theta56(2) ;
         theta1(3) theta2(3) theta3(2) theta4(3) theta56(3) ;
         theta1(4) theta2(4) theta3(2) theta4(4) theta56(4) ];
   THETA=vpa(THETA,3)
end

%Instead of 8 solutions there will only be 4 solutions because theta4 will
%have 1 solution instead of 2.

%I have taken T_des matrix which is the orientation and coordinates of the
%end effector.
% Using the same T_des in this code, check the first row of output THETA-It
% is the same as the THETA in the forward kinematics code. This shows that
% the code is working good ! :)


