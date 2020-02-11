function [THETA]= Yasukawa_inverse(T_des, d1, d4, d6, a2)
  
  d1=1;
  d4=2;
  d6=3;
  a2=5;
   
  D=[d1 0 0 d4 0 d6];
  A=[0 a2 0 0 0 0];
  ALPA=[pi/2 0 pi/2 pi/2 pi/2 0];
  
  %T_des as obtained from forward_kinematics_code
  T_des=[  0.94 -0.342 -0.0144   5.79;
           0.333  0.924  -0.189, 0.0177;
           0.078  0.173   0.982   3.18;
           0      0       0    1.0];

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
  P=T_des*[0 0 -d6 1]';
  Xp=P(1);
  Yp=P(2);
  Zp=P(3);
  
  %formulae
  zeta= Zp-d1;
  eta=zeros(1,2);
  eta(1)=sqrt(Xp^2+Yp^2);
  eta(2)=-sqrt(Xp^2+Yp^2);
  
  s3=(eta(1)^2+zeta^2-(d4^2+a2^2))/(2*a2*d4);
  
  if(s3>1 || s3<-1)
    fprintf("Error. Desired configuration cannot be reached");
    disp(s3);
    disp(P);
    disp(Xp);
    disp(Yp);
    disp(Zp);
  end
  
  theta3(1)=asin(s3);
  theta3(2)=pi-asin(s3);
  
  c3(1)=cos(theta3(1));
  c3(2)=cos(theta3(2));
  
  
  s2(1)=1/(zeta^2+eta(1)^2)*((a2+d4*s3)*zeta+d4*c3(1)*eta(1));
  c2(1)=1/(zeta^2+eta(1)^2)*(-d4*c3(1)*zeta+(a2+d4*s3)*eta(1));
  
  s2(2)=1/(zeta^2+eta(1)^2)*((a2+d4*s3)*zeta+d4*c3(1)*eta(2));
  c2(2)=1/(zeta^2+eta(1)^2)*(-d4*c3(1)*zeta+(a2+d4*s3)*eta(2));
 
  
  s2(3)=1/(zeta^2+eta(1)^2)*((a2+d4*s3)*zeta+d4*c3(2)*eta(1));
  c2(3)=1/(zeta^2+eta(1)^2)*(-d4*c3(2)*zeta+(a2+d4*s3)*eta(1));
  
  s2(4)=1/(zeta^2+eta(1)^2)*((a2+d4*s3)*zeta+d4*c3(2)*eta(2));
  c2(4)=1/(zeta^2+eta(1)^2)*(-d4*c3(2)*zeta+(a2+d4*s3)*eta(2));
  
  theta2(1)=atan2(s2(1),c2(1));  % for theta 31
  theta2(2)=atan2(s2(2),c2(2));  % for theta 31
  theta2(3)=atan2(s2(3),c2(3));  % for theta 32
  theta2(4)=atan2(s2(4),c2(4));  % for theta 32
  
  c1(1)=Xp/((a2+d4*s3)*c2(1)+d4*c3(1)*s2(1));
  s1(1)=Yp/((a2+d4*s3)*c2(1)+d4*c3(1)*s2(1)); 
  
  c1(2)=Xp/((a2+d4*s3)*c2(2)+d4*c3(1)*s2(2));
  s1(2)=Yp/((a2+d4*s3)*c2(2)+d4*c3(1)*s2(2));
  
  c1(3)=Xp/((a2+d4*s3)*c2(3)+d4*c3(2)*s2(3));
  s1(3)=Yp/((a2+d4*s3)*c2(3)+d4*c3(2)*s2(3));
  
  c1(4)=Xp/((a2+d4*s3)*c2(4)+d4*c3(2)*s2(4));
  s1(4)=Yp/((a2+d4*s3)*c2(4)+d4*c3(2)*s2(4));
  
  theta1(1)=atan2(s1(1),c1(1));
  theta1(2)=atan2(s1(2),c1(2));
  theta1(3)=atan2(s1(3),c1(3));
  theta1(4)=atan2(s1(4),c1(4));
%  theta1=atan2(s1,c1);
  
  
  %update THETA for theta 1, 2, 3
  THETA=[theta1(1) theta2(1) theta3(1) 0 0 0;
         theta1(2) theta2(2) theta3(1) 0 0 0;
         theta1(3) theta2(3) theta3(2) 0 0 0;
         theta1(4) theta2(4) theta3(2) 0 0 0];
  
  
  % Now to find theta 4, 5 ,6
   for i=1:4
        T1_0(:,:,i)= Tz(THETA(i,1))*T_trans(d1,0)*Tx(pi/2);
        T2_1(:,:,i)= Tz(THETA(i,2))*T_trans(0,a2);
        T3_2(:,:,i)= Tz(THETA(i,3))*Tx(pi/2);
        T3_0(:,:,i)= T1_0(:,:,i)*T2_1(:,:,i)*T3_2(:,:,i);
        T3_0_inv(:,:,i)=inv(T3_0(:,:,i));
        T_hat_des(:,:,i)= T3_0_inv(:,:,i)*T_des;
        R_hat_des(:,:,i)= T_trans(-d4,0)*T_hat_des(:,:,i)*T_trans(-d6,0)*Tx(pi/2);
   end
   
   
   for i=1:4
       c5(i)=-R_hat_des(3,2,i);
       
       theta5(i,1)=acos(c5(i));
       theta5(i,2)=2*pi-acos(c5(i));
       
       s5(i,1)=sin(theta5(i,1));
       s5(i,2)=sin(theta5(i,2));
       
       c4(i,1)= R_hat_des(1,2,i)/s5(i,1);
       s4(i,1)= R_hat_des(2,2,i)/s5(i,1);
       
       c4(i,2)= R_hat_des(1,2,i)/s5(i,2);
       s4(i,2)= R_hat_des(2,2,i)/s5(i,2);
       
       theta4(i,1)=atan2(s4(i,1),c4(i,1));
       theta4(i,2)=atan2(s4(i,2),c4(i,2));
       
       c6(i,1)= R_hat_des(3,1,i)/s5(i,1);
       s6(i,1)= R_hat_des(3,3,i)/s5(i,1);
       
       c6(i,2)= R_hat_des(3,1,i)/s5(i,2);
       s6(i,2)= R_hat_des(3,3,i)/s5(i,2);
       
       theta6(i,1)=atan2(s6(i,1),c6(i,1));
       theta6(i,2)=atan2(s6(i,2),c6(i,2));       
       
   end
   THETA=[theta1(1) theta2(1) theta3(1) theta4(1,1) theta5(1,1) theta6(1,1);
          theta1(1) theta2(1) theta3(1) theta4(1,2) theta5(1,2) theta6(1,2);
          theta1(2) theta2(2) theta3(1) theta4(2,1) theta5(1,1) theta6(2,1);
          theta1(2) theta2(2) theta3(1) theta4(2,2) theta5(1,2) theta6(2,2);
          theta1(3) theta2(3) theta3(2) theta4(3,1) theta5(1,1) theta6(3,1);
          theta1(3) theta2(3) theta3(2) theta4(3,2) theta5(1,2) theta6(3,2);
          theta1(4) theta2(4) theta3(2) theta4(4,1) theta5(1,1) theta6(4,1);
          theta1(4) theta2(4) theta3(2) theta4(4,2) theta5(1,2) theta6(4,2)];

THETA =vpa(THETA,3);
end

%I have taken T_des matrix which is the orientation and coordinates of the
%end effector.
% Using the same T_des in this code, check the first row of output THETA-It
% is the same as the THETA in the forward kinematics code. This shows that
% the code is working good ! :)



