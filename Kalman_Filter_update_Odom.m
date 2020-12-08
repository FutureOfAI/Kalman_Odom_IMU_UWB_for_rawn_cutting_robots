function [P00_z,K_z,z_update]=Kalman_Filter_update_Odom(P00_z,v_B,H,R,Vxm_Bh,k)

zxm_z = v_B(k) - Vxm_Bh(k);

Mu_z= zxm_z';

K_z=P00_z*H'/(H*P00_z*H'+R);
z_update = K_z*Mu_z;

I=eye(2);
P00_z=(I-K_z*H)*P00_z;                     % P(k|k)

end

