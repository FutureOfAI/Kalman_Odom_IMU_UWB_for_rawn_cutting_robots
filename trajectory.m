function [x_p,x_v,x_a,y_p,y_v,y_a] = trajectory(radius,psi,wn)

x_p = radius*cos(psi);
x_v = - radius*wn*sin(psi);
x_a = - radius*wn^2*cos(psi);
y_p = radius*sin(psi);
y_v = radius*wn*cos(psi);
y_a = - radius*wn^2*sin(psi);
end
