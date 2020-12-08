function[H,R,R1m_h,R2m_h,R3m_h,R4m_h]=radio_discrete_EKF_expm(num,xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,xpm_Nh,ypm_Nh,sig_x_r,sig_y_r,h,k)

R1m_h(k) = sqrt((xr1-xpm_Nh(k))^2+(yr1-ypm_Nh(k))^2+(zr1-h)^2);
R2m_h(k) = sqrt((xr2-xpm_Nh(k))^2+(yr2-ypm_Nh(k))^2+(zr2-h)^2);
R3m_h(k) = sqrt((xr3-xpm_Nh(k))^2+(yr3-ypm_Nh(k))^2+(zr3-h)^2);
R4m_h(k) = sqrt((xr4-xpm_Nh(k))^2+(yr4-ypm_Nh(k))^2+(zr4-h)^2);

r1_partial_x =-1*(xr1-xpm_Nh(k))/R1m_h(k);
r1_partial_y =-1*(yr1-ypm_Nh(k))/R1m_h(k);
r2_partial_x =-1*(xr2-xpm_Nh(k))/R2m_h(k); 
r2_partial_y =-1*(yr2-ypm_Nh(k))/R2m_h(k); 
r3_partial_x =-1*(xr3-xpm_Nh(k))/R3m_h(k);
r3_partial_y =-1*(yr3-ypm_Nh(k))/R3m_h(k);
r4_partial_x =-1*(xr4-xpm_Nh(k))/R4m_h(k); 
r4_partial_y =-1*(yr4-ypm_Nh(k))/R4m_h(k); 

if num == 2 % 2 anchors H and R matrix
    H = [r1_partial_x r1_partial_y 0 0
                  r2_partial_x r2_partial_y 0 0];

    R = 100*[sig_x_r^2 0;0 sig_y_r^2];
else if num == 3 % 3 anchors H and R matrix
        H = [r1_partial_x r1_partial_y 0 0
                      r2_partial_x r2_partial_y 0 0
                      r3_partial_x r3_partial_y 0 0];

        R = 100*[sig_x_r^2 0 0
            0 sig_y_r^2 0
            0 0 sig_x_r^2];        
    else % 4 anchors H and R matrix
        H = [r1_partial_x r1_partial_y 0 0
                      r2_partial_x r2_partial_y 0 0
                      r3_partial_x r3_partial_y 0 0
                      r4_partial_x r4_partial_y 0 0];

        R = 100*[sig_x_r^2 0 0 0
            0 sig_y_r^2 0 0
            0 0 sig_x_r^2 0
            0 0 0 sig_y_r^2];
    end 
end

end