n1 = 10;
n2 = k-1;

figure (1)
plot(x,y,'--r',odom_x,odom_y,'g-',xpm_Nh,ypm_Nh,xr1,yr1,'r*')
hold on
plot(3.84,0,'diamond',3.54,0,'rsquare','MarkerSize',13,'LineWidth',2)
legend('Reference','Odometer','EKF','Anchor','Start point','End point');%'Reentry point',
hold on
plot(xr2,yr2,'r*',xr3,yr3,'r*',xr4,yr4,'r*')
title('EKF')
xlabel('X position in m')
ylabel('Y position in m')
% axis([-5 9 -1 11])
axis([-2 9 -1 11])
% axis equal
set(gca,'YDir','reverse')
grid
%
figure (2)
subplot(211)
% plot(t0(n1:n2),dv_x(n1:n2),'r',t0(n1:n2),xvm_Bh(n1:n2),'g--')
plot(t0(n1:n2),dv_x(n1:n2),'r')
xlabel('Time in sec')
ylabel('B-frame X velocity in m/s')
subplot(212)
% plot(t0(n1:n2),dv_y(n1:n2),'r',t0(n1:n2),yvm_Bh(n1:n2),'g--')
plot(t0(n1:n2),dv_y(n1:n2),'r')
xlabel('Time in sec')
ylabel('B-frame Y velocity in m/s')
%
% figure(3);
% subplot(3,1,1), plot(t0,bx/g,'r',t0,bx_h/g,'g');         
% xlabel('Time in seconds');ylabel('Accel bias along-axis in g');
% subplot(3,1,2), plot(t0,by/g,'r',t0,by_h/g,'g');         
% xlabel('Time in seconds');ylabel('Accel bias perp-axis in g');
% subplot(3,1,3), plot(t0,bz*r2d,'r',t0,bz_h*r2d,'g');
% xlabel('Time in seconds');ylabel('Gyro bias in deg');
%
figure (4)
subplot(2,1,1), plot(t0,bx_h/g,'r');         
xlabel('Time in seconds');ylabel('Accel bias x-axis in g');
subplot(2,1,2), plot(t0,by_h/g,'r');         
xlabel('Time in seconds');ylabel('Accel bias y-axis in g');
% subplot(3,1,3), plot(t0,bz_h*r2d,'g');
% xlabel('Time in seconds');ylabel('Gyro bias in deg/sec');
%
figure (5)
subplot(2,1,1), plot(t0(n1:n2),(bx(n1:n2)-bx_h(n1:n2))/g,'r');         
xlabel('Time in seconds');ylabel('Accel bias err in x-axis in g');
grid
subplot(2,1,2), plot(t0(n1:n2),(by(n1:n2)-by_h(n1:n2))/g,'r');         
xlabel('Time in seconds');ylabel('Accel bias err in y-axis in g');
grid
%
psi_err = zeros(1,n2);
psi_err = psi_h(1:n2)-yaw(1:n2);
psi_err_mean = mean(psi_err)*r2d
figure (6)
% subplot(211)
% plot(t0(n1:n2),(psi_h(n1:n2)')*r2d,'r',t0(n1:n2),yaw(n1:n2)*r2d,'g');
% xlabel('Time in seconds');ylabel('psi_h Angle in deg');
subplot(211)
plot(t0(n1:n2),psi_err(n1:n2)*r2d,'r');
xlabel('Time in seconds');ylabel('psi_h Angle err in deg');
grid
subplot(212), plot(t0(n1:n2),bz_h(n1:n2)*r2d,'r');         
xlabel('Time in seconds');ylabel('Gyro bias in z-axis in rad/s');
grid
% 
psi_err_std = zeros(1,n2);
psi_err_std = std(psi_err)
figure (9)
plot(t0(n1:n2),psi_h(n1:n2)*r2d,'r',t0(n1:n2),yaw(n1:n2)*r2d,'g',t0(n1:n2),omega_sum(n1:n2)*r2d,'b');
xlabel('Time in seconds');ylabel('Angle in deg');
legend('Psi_h','Yaw','Odom head');%'Reentry point',
% 
% figure (9)
% subplot(2,1,1), plot(t00(n1:n2),x_p_N(n1:n2),'r',t00(n1:n2),xpm_Nh(n1:n2),'g--');
% xlabel('Time in seconds');ylabel('N-frame Position in x-axis in m');
% subplot(2,1,2), plot(t00(n1:n2),y_p_N(n1:n2),'r',t00(n1:n2),ypm_Nh(n1:n2),'g--');
% xlabel('Time in seconds');ylabel('N-frame Position in y-axis in m');
% 
% figure (10)
% subplot(2,1,1), plot(t00(n1:n2),(x_p_N(n1:n2)-xpm_Nh(n1:n2)'),'r');
% xlabel('Time in seconds');ylabel('N-frame Position err in x-axis in m');
% subplot(2,1,2), plot(t00(n1:n2),(y_p_N(n1:n2)-ypm_Nh(n1:n2)'),'r');
% xlabel('Time in seconds');ylabel('N-frame Position err in y-axis in m');
% 
figure (7)
subplot(211)
plot(t0(n1:n2),xpm_Nh(n1:n2),'r',t0(n1:n2),odom_x(n1:n2),'g')
xlabel('Time in sec')
ylabel('x axis position in m')
subplot(212)
plot(t0(n1:n2),ypm_Nh(n1:n2),'r',t0(n1:n2),odom_y(n1:n2),'g')
xlabel('Time in sec')
ylabel('y axis position in m')
% 
posx_err = zeros(1,n2);
posy_err = zeros(1,n2);
posx_err = xpm_Nh(1:n2)-odom_x(1:n2);
posy_err = ypm_Nh(1:n2)-odom_y(1:n2);
posx_err_mean = mean(posx_err);
posy_err_mean = mean(posy_err);
figure (8)
subplot(211)
plot(t0(n1:n2),posx_err(n1:n2),'r')
xlabel('Time in sec')
ylabel('x position err in m')
subplot(212)
plot(t0(n1:n2),posy_err(n1:n2),'r')
xlabel('Time in sec')
ylabel('y position err in m')

