function [x_p,x_v,x_a,y_p,y_v,y_a] = trajectory_mult(radius_x,radius_y,psi_0,wt,t0)

x_p = radius_x*sin(wt*t0)*cos(psi_0)+6.0;
x_v = radius_x*wt*cos(wt*t0)*cos(psi_0);
x_a = - radius_x*wt^2*sin(wt*t0)*cos(psi_0);
y_p = radius_y*sin(wt*t0)*sin(psi_0)+4.53;
y_v = radius_y*wt*cos(wt*t0)*sin(psi_0);
y_a = - radius_y*wt^2*sin(wt*t0)*sin(psi_0);
end
