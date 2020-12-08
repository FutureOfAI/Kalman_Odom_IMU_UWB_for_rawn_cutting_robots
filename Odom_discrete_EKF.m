function[H,R,Vm_h]=Odom_discrete_EKF(xvm_Bh, sig_x_r)

Vm_h = xvm_Bh;

H = [1 0]; % HÎª1*2¾ØÕó

R = 0.0005*sig_x_r^2; % R¾ØÕóÎª1*1

end