function[H,R,R1m_h,R2m_h]=radio_discrete_EKF(xr1,yr1,xr2,yr2,xpm_Nh,ypm_Nh,sig_x_r,sig_y_r,h,k)

R1m_h(k) = sqrt((xr1-xpm_Nh(k))^2+(yr1-ypm_Nh(k))^2+h^2);
R2m_h(k) = sqrt((xr2-xpm_Nh(k))^2+(yr2-ypm_Nh(k))^2+h^2);


r1_partial_x =-1*(xr1-xpm_Nh(k))/R1m_h(k);
r1_partial_y =-1*(yr1-ypm_Nh(k))/R1m_h(k);
r2_partial_x =-1*(xr2-xpm_Nh(k))/R2m_h(k); 
r2_partial_y =-1*(yr2-ypm_Nh(k))/R2m_h(k); 

H = [r1_partial_x r1_partial_y 0 0
              r2_partial_x r2_partial_y 0 0];

R = 5*[sig_x_r^2 0;0 sig_y_r^2];
end