function[phi_z,Q_z,F_z]=define_Dymamic_equation4_radio(F_z,Q_z,xvm_Nh,yvm_Nh,psi_h,sig_bx,sig_by,sig_arw_0,sig_rrw_0,dt,k)
%% 
F_z(1,3) = 1*(-yvm_Nh(k-1));
F_z(2,3) = 1*(xvm_Nh(k-1));
I=eye(4); % 4*4单位矩阵
%phi_z = expm(F_z*dt);
phi_z = I+(F_z*dt);  % phi�?*8矩阵
%phi_z = I + F_z*dt; 
Q_z(1,1) = ((cos(psi_h(k-1))).^2)*sig_bx + ((sin(psi_h(k-1))).^2)*sig_by;
Q_z(1,2) = sin(psi_h(k-1))*cos(psi_h(k-1))*(sig_bx-sig_by);
Q_z(2,1) = Q_z(1,2);
Q_z(2,2) = ((sin(psi_h(k-1))).^2)*sig_bx + ((cos(psi_h(k-1))).^2)*sig_by;

% Q_z(1,1) = ((cos(psi_h(k-1))).^2)*sig_bx^2 + ((sin(psi_h(k-1))).^2)*sig_by^2;
% Q_z(2,2) = ((sin(psi_h(k-1))).^2)*sig_bx^2 + ((cos(psi_h(k-1))).^2)*sig_by^2;
Q_z(3,3) = 1*sig_arw_0^2;
Q_z(4,4) = 1*sig_rrw_0^2;

end