function[H,R,Vm_h]=Odom_discrete_EKF(xvm_Bh, sig_x_r)

Vm_h = xvm_Bh;

H = [1 0]; % HΪ1*2����

R = 0.0005*sig_x_r^2; % R����Ϊ1*1

end