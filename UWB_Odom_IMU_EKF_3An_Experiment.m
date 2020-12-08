% Program name:UWB_Odom_IMU_EKF_Simulation.m
% A new program that deal with non-constant psi angle where
% a lossely couped two 2-state Kalman filter and 4-state Kalman filter with UWB sensor

close all;
clear all;
clc;

UWB_IMU_ODOM_RawData;

format short e
r2d = (180/pi);
d2r = (pi/180);
g =9.8;

% Set motion profile flags
profile_flag = 2;

% positions of four ratio sensors's positions
xr1 = 0;% in meter
yr1 = 0;% in meter
zr1 = 2.03;
xr2 = 0;
yr2 = 10.3;
zr2 = 1.97;
xr4 = 5.7;% in meter
yr4 = 10.3;% in meter
zr4 = 1.97;
xr3 = 5.7;
yr3 = 0;
zr3 = 1.88;
h = 0.5;% ceiling high in meter    
% reference path
x = [3.84 3.84 1.84 1.84 3.54];
y = [0 10.3 10.3 0 0];
sum_anchor = 3;
%% Straight motion
%
if (profile_flag ==1)
    % =====================================================
    % Straight motion with psi = constant angle, or wn = 0
    % =====================================================
    %
    dt = 0.01;
    T = 200;
    t0 = 0:dt:T;
    t0_1 = t0';
    n = length(t0);
    m = size(t0_1);
    t00 = t0;
    %
    fn = 0*0.05;
    psi_0 = 1*45*d2r; % Slope of trajectory
    wn = 2*pi*fn;
    wz(1) = wn;
    psi(1) = psi_0 + wz(1)*t0(1);
    for i = 2:n,
        wz(i) = wn;
        psi(i) = psi(i-1)+ (wz(i) + wz(i-1))*dt/2;
        if (psi(i)>= 2*pi),
            psi(i) = psi(i) - 2*pi;
        else
            psi(i) = psi(i);
        end
    end
    % ====================================================
    ft =1* 0.05;
    wt = 2*pi*ft;
    radius_x = 5;
    radius_y = 2.5;
    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N] = trajectory_mult(radius_x,radius_y,psi_0,wt,t0);
else if (profile_flag ==2)
    %
    dt = 0.1;
    T = 176; % 228 198 176 174 113
    t0 = 0:dt:T;
    t0_1 = t0';
    n = length(t0);
    m = size(t0_1);
    t00 = t0;
    %
    fn = 1*0.05;
    psi_0 = 0*45*d2r;
    wn = 2*pi*fn;
    wz(1) = wn;
    psi(1) = psi_0 + wz(1)*t0(1);
    for i = 2:n,
    wz(i) = wn;
    psi(i) = psi(i-1) + (wz(i) + wz(i-1))*dt/2;
    if (psi(i) >= 2*pi),
        psi(i) = psi(i) - 2*pi;
    else
        psi(i) = psi(i);
    end
    end
    radius = 5;
    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N] = trajectory (radius,psi,wn);%round cycle
    % ====================================================
else if (profile_flag ==3)
    % ===================================================
    % Race track motion
    % ===================================================
    v = 1;                            % ground speed in m/sec
    roll_a = 0.1*1.25*d2r;                 % band-band acceleration in m/sec^2
    Ts = 4 + 0.0;
    T1 = 14.3 + 0.0;
    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,roll_angle,yaw_angle,yaw_rate,time] = trajectory2(v,roll_a,Ts,T1);
    %
    dt = 0.01;
    T = time(length(time)-69);
    t0 = 0:dt:T;
    t0_1 = t0';
    n = length(t0);
    m = size(t0_1);
    t00 = time;
    psi = yaw_angle;
    wz = yaw_rate;
    for i = 2:n,
        if (psi(i) >= 2*pi),
            psi(i) = psi(i) - 2*pi;
        else
            psi(i) = psi(i);
        end
    end
    end
    end
end
% ====================================================
% Need to convert into body frame (B-frame) for accelerometer sensings
% since they are mounted on the body frame
% ====================================================
x_v_B = zeros(1,n);
y_v_B = zeros(1,n);
%
x_a_B = zeros(1,n);
y_a_B = zeros(1,n);
% ================================
for i = 1:n,
    x_a_B(i) = [cos(psi(i)) sin(psi(i))]*[x_a_N(i);y_a_N(i)];
    y_a_B(i) = [-sin(psi(i)) cos(psi(i))]*[x_a_N(i);y_a_N(i)];
    x_v_B(i) = [cos(psi(i)) sin(psi(i))]*[x_v_N(i);y_v_N(i)];
    y_v_B(i) = [-sin(psi(i)) cos(psi(i))]*[x_v_N(i);y_v_N(i)];    
end
%     x_v_B(1) = [cos(psi(1)) sin(psi(1))]*[x_v_N(1);y_v_N(1)];
%     y_v_B(1) = [-sin(psi(1)) cos(psi(1))]*[x_v_N(1);y_v_N(1)];
% for i = 2:n,
%     x_v_B(i) = x_v_B(i-1) + (x_a_B(i) + x_a_B(i-1))*dt/2;
%     y_v_B(i) = y_v_B(i-1) + (y_a_B(i) + y_a_B(i-1))*dt/2;
% end
% ========================================================
% Define inertial sensor parameters "accelerate mpu9150"
% ========================================================
% gyroscope
bz0=1*0.5*d2r;                            % initial gyro bias in rad/sec
%=============================================================================================================
% gyroscope(bias & noise)
%=====================================
sig_arw_0 = 1*0.02;                       % gyro angle random walk = 0.02 deg/sqrt(sec)
sig_rrw_0 = 1*0.02/3600;                    % gyro rate random walk = 0.02 deg/3600/sec-sqrt(sec);
[bz]=Biasbg1(dt,n,m,bz0,d2r,sig_rrw_0);
[wzm]=Wzm1(wz,bz,m,n,d2r,sig_arw_0);

%=============================================================================================================
% accelerometer (biases and noises)
%=====================================
bx0=1*0.1*g;                             % initial accel bias in g in along-direction
by0=-1*0.1*g;                              % initial accel bias in g in perpenticular-direction
err_factor = 1.0;
%
sig_xr_0 = err_factor*0.1*g/3600;         % accel bias stability in g/sec-sqrt(sec) in along-direction
sig_yr_0 = err_factor*0.1*g/3600;         % accel bias stability in g/sec-sqrt(sec) in penpenticular-direction

% accelerate (noise)
sig_bx_0 = err_factor*0.01*g;              % accel noise in g in along-direction
sig_by_0 = err_factor*0.01*g;              % accel noise in g in penpenticular-direction

% accelerate (calculator bias)
[bx]=Biasba1(dt,n,m,bx0,sig_xr_0);
[by]=Biasbp1(dt,n,m,by0,sig_yr_0);
[axm,aym]=transform_m(x_a_B,bx,y_a_B,by,m,n,sig_bx_0,sig_by_0);

% calculator "Q" use bias & noise
sig_bx=sig_bx_0;%(noise)
sig_by=sig_by_0;%(noise)
sig_xr=sig_xr_0;%(bias)
sig_yr=sig_yr_0;%(bias)

% Define ""4 radio sensor"" parameters (noise)
radiosensor_err_factor = 1.0;
sig_x_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meters x-direction
sig_y_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meters y-direction
%=========================================================
[R1m,R2m,nvx_r,nvy_r] = radio_sensor_m(xr1,yr1,zr1,xr2,yr2,zr2,x_p_N,y_p_N,h,sig_x_r,sig_y_r,n,m);%4-state
% Define velocity parameters (noise)
Vxm_B = x_v_B + nvx_r;
Vym_B = y_v_B + nvy_r;
%=========================================================
delta_t = dt;                                   % delta time for simulating the true dynamics = 0.01 sec
delta_s = 1*delta_t;                            % sampling at every 0.1 second for the Kalman filter
%================================================================
[sensor_step,propagation_step]=propagate_step(T,delta_t,delta_s);
%================================================================
% Define the initial conditions for the inertial sensors and the navigation
% states
%===============================================================
% Introduce initial position and velocity estimate error in N-frame
xverr = 0.1;        % in meters/sec
yverr = -0.1;       % in meters/sec
xperr = 0.5;        % in m
yperr = -0.5;       % in m
xaerr = 0;          % in m/sec^2
yaerr = 0;          % in m/sec^2
psierr = 0.5;       % in deg
% [xvm_Bh,bx_h]=initial_estimate_value2_radio(m,x_v_B); % Vx initial state
% [yvm_Bh,by_h]=initial_estimate_value2_radio(m,y_v_B); % Vy initial state
% [xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,axm_h,aym_h,wzm_h,psi_h,bz_h]=initial_estimate_value4_radio(m,wzm,axm,aym,d2r,x_p_N(1),y_p_N(1),psi(1),bz0);
[xvm_Bh,bx_h]=initial_estimate_value2_radio(m,dv_x(1)); % Vx initial state
[yvm_Bh,by_h]=initial_estimate_value2_radio(m,dv_y(1)); % Vy initial state
[xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,axm_h,aym_h,wzm_h,psi_h,bz_h]=initial_estimate_value4_radio(m,groz,accx,accy,d2r,3.84,0,psi(1),bz0);
%================================================================
% Define the initial conditions for the 2-state Kalman Filter 
% ==============================================================
[Vx_xz_h,Vx_P00_z]=define_initial_condition_2(bx0,xverr);
[Vy_xz_h,Vy_P00_z]=define_initial_condition_2(by0,yverr);
[xz_h,P00_z]=define_initial_condition_4(bz0,xperr,yperr,psierr,d2r);
% ==============================================================
% for 2-state filter
Vx_F_z=zeros(2);
Vy_F_z=zeros(2);
%Vx_F_z = [0 -1
%          0 0];
Vx_F_z(1,2) = -1;
Vy_F_z(1,2) = -1;
% for 4-state filter
F_z=zeros(4);
%F_z = [0 0 -Vy 0
%       0 0 Vx 0
%       0 0 0 -1
%       0 0 0 0];
F_z(3,4) = -1;
Vx_Q_z=zeros(2);
Vy_Q_z=zeros(2);
Q_z=zeros(4);
% ============================================================
% Start the simulation run
% ============================================================
for i=1:sensor_step
    for j=1:propagation_step
        k=1+j+(i-1)*propagation_step;
        
        bx_h(k)=bx_h(k-1);
        by_h(k)=by_h(k-1);
        bz_h(k)=bz_h(k-1);
% =========================================
% Perform inertial navigation computations
% =========================================
%         [xvm_Bh,axm_h]=velosity_computation2_radio(xvm_Bh,axm,bx_h,k,dt);
%         [yvm_Bh,aym_h]=velosity_computation2_radio(yvm_Bh,aym,by_h,k,dt);
%         [xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,wzm_h,psi_h]=position_computation4_radio(xvm_Bh,yvm_Bh,xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,psi_h,wzm,bz_h,k,dt,0);
        [xvm_Bh,axm_h]=velosity_computation2_radio(xvm_Bh,accx,bx_h,k,dt);
        [yvm_Bh,aym_h]=velosity_computation2_radio(yvm_Bh,accy,by_h,k,dt);
        [xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,wzm_h,psi_h]=position_computation4_radio(xvm_Bh,yvm_Bh,xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,psi_h,groz,bz_h,k,dt,0);
% ===========================================================
% Perform Kalman filter propagation for 2-state Kalman filter
% ===========================================================
        [Vx_phi_z,Vx_Q_z]=define_Dymamic_equation2_radio(Vx_F_z,Vx_Q_z,sig_xr,sig_yr,dt);
        [Vx_xz_h,Vx_P00_z]=Kalman_Filter_estimate1_radio(Vx_xz_h,Vx_phi_z,Vx_P00_z,Vx_Q_z,dt);
        [Vy_phi_z,Vy_Q_z]=define_Dymamic_equation2_radio(Vy_F_z,Vy_Q_z,sig_xr,sig_yr,dt);
        [Vy_xz_h,Vy_P00_z]=Kalman_Filter_estimate1_radio(Vy_xz_h,Vy_phi_z,Vy_P00_z,Vy_Q_z,dt);
        [phi_z,Q_z,F_z]=define_Dymamic_equation4_radio(F_z,Q_z,xvm_Nh,yvm_Nh,psi_h,Vx_P00_z(1,1),Vy_P00_z(1,1),sig_arw_0,sig_rrw_0,dt,k);
%         [phi_z,Q_z,F_z]=define_Dymamic_equation4_radio(F_z,Q_z,xvm_Nh,yvm_Nh,psi_h,sig_bx,sig_by,sig_arw_0,sig_rrw_0,dt,k);
        [xz_h,P00_z]=Kalman_Filter_estimate1_radio(xz_h,phi_z,P00_z,Q_z,dt);
    end   % end of filter propagation step
% =======================================================
% Perform Kalman filter updates for 2-state filter
% =======================================================
    [Vx_H,Vx_R,Vxm_Bh]=Odom_discrete_EKF(xvm_Bh, sig_x_r);
    [Vy_H,Vy_R,Vym_Bh]=Odom_discrete_EKF(yvm_Bh, sig_y_r);
    [H,R,R1m_h,R2m_h,R3m_h,R4m_h]=radio_discrete_EKF_expm(sum_anchor,xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,xpm_Nh,ypm_Nh,sig_x_r,sig_y_r,h,k);
%   
%     [Vx_P00_z,Vx_K_z,Vx_z_update]=Kalman_Filter_update_Odom(Vx_P00_z,Vxm_B,Vx_H,Vx_R,Vxm_Bh,k);
%     [Vy_P00_z,Vy_K_z,Vy_z_update]=Kalman_Filter_update_Odom(Vy_P00_z,Vym_B,Vy_H,Vy_R,Vym_Bh,k);
%     [P00_z,K_z,z_update]=Kalman_Filter_update_radio(P00_z,R1m,R2m,H,R,R1m_h,R2m_h,k);
    [Vx_P00_z,Vx_K_z,Vx_z_update]=Kalman_Filter_update_Odom(Vx_P00_z,dv_x,Vx_H,Vx_R,Vxm_Bh,k);
    [Vy_P00_z,Vy_K_z,Vy_z_update]=Kalman_Filter_update_Odom(Vy_P00_z,dv_y,Vy_H,Vy_R,Vym_Bh,k);
    [P00_z,K_z,z_update]=Kalman_Filter_update_radio_expm(sum_anchor,P00_z,anchor_cal(:,1),anchor_cal(:,2),anchor_cal(:,4),anchor_cal(:,3),H,R,R1m_h,R2m_h,R3m_h,R4m_h,k);
%     
    [xvm_Bh,bx_h]=upon_radiosensor_measurement_Vx(xvm_Bh,bx_h,Vx_z_update,k);
    [yvm_Bh,by_h]=upon_radiosensor_measurement_Vx(yvm_Bh,by_h,Vy_z_update,k);
    [xpm_Nh,ypm_Nh,psi_h,bz_h]=upon_radiosensor_measurement_UWB(xpm_Nh,ypm_Nh,k,psi_h,bz_h,z_update);
    
% reset the errors after the filter updates
    Vx_xz_h=zeros(2,1);
    Vy_xz_h=zeros(2,1);
    xz_h=zeros(4,1);
end


plotUWBexpm;



