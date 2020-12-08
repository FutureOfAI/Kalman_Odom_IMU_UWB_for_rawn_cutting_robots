function [P00_z,K_z,z_update]=Kalman_Filter_update_radio(P00_z,R1m,R2m,H,R,R1m_h,R2m_h,k)

zxm_z1(k) = R1m(k)-R1m_h(k)' ;
zxm_z2(k) = R2m(k)-R2m_h(k)' ;

%Mu_x=zxm_x(k)-zxm_hatx(k);
Mu_z=[zxm_z1(k);zxm_z2(k)];

K_z=P00_z*H'/(H*P00_z*H'+R);    
z_update = K_z*Mu_z;
%xz_h=xz_h+z_update;                        % x(k|k)
I=eye(4);
P00_z=(I-K_z*H)*P00_z;                     % P(k|k)



end

