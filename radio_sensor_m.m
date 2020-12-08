function [R1m,R2m,nvx_r,nvy_r] = radio_sensor_m(xr1,yr1,zr1,xr2,yr2,zr2,x_p_N,y_p_N,h,sig_x_r,sig_y_r,n,m)
%radiosensor_err_factor = 1.0;
%sig_x_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meter/sec
%sig_y_r=radiosensor_err_factor*0.2;              % radio sensor measurement noise in meter/sec in y-direction
nvx_r=normrnd(0,sig_x_r,1,n);
nvy_r=normrnd(0,sig_y_r,1,n);
R1m=zeros(m);
R2m=zeros(m);
for i=1:n
    R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2+(zr1-h)^2)+nvx_r(i);
    R2m(i) = sqrt((xr2-x_p_N(i))^2+(yr2-y_p_N(i))^2+(zr2-h)^2)+nvy_r(i);
end
end