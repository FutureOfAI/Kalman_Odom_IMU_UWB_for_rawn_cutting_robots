n1 = 400;
n2 = k-1;

figure (1)
subplot(311)
plot(x_p_N,y_p_N,'r.',xpm_Nh,ypm_Nh,'g-.')
% plot(x_p_N,y_p_N,'g*')
xlabel('X position in m')
ylabel('Y position in m')
grid
subplot(312)
plot(t00,x_v_N,'r',t00,y_v_N, 'g')
xlabel('Time in sec')
ylabel('Velocity in m/s')
grid
subplot(313)
plot(t00,x_a_N/g,'r',t00,y_a_N/g,'g')
ylabel('accel in g')
xlabel('Time in sec')
grid
%
figure (2)
subplot(211)
plot(t0,x_v_B,'r',t0,xvm_Bh,'g--')
xlabel('Time in sec')
ylabel('B-frame X velocity in m/s')
subplot(212)
plot(t0,y_v_B,'r',t0,yvm_Bh,'g--')
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
subplot(3,1,1), plot(t0,bx/g,'r',t0,bx_h/g,'g');         
xlabel('Time in seconds');ylabel('Accel bias x-axis in g');
subplot(3,1,2), plot(t0,by/g,'r',t0,by_h/g,'g');         
xlabel('Time in seconds');ylabel('Accel bias y-axis in g');
subplot(3,1,3), plot(t0,bz*r2d,'r',t0,bz_h*r2d,'g');
xlabel('Time in seconds');ylabel('Gyro bias in deg/sec');
%
bx_err = zeros(1,n2);
by_err = zeros(1,n2);
bx_err = (bx(1:n2)-bx_h(1:n2))/g;
by_err = (by(1:n2)-by_h(1:n2))/g;
bx_err_mean = mean(bx_err)
by_err_mean = mean(by_err)
figure (5)
subplot(2,1,1), plot(t0(n1:n2),bx_err(n1:n2),'r');         
xlabel('Time in seconds');ylabel('Accel bias err in x-axis in g');
grid
subplot(2,1,2), plot(t0(n1:n2),by_err(n1:n2),'r');         
xlabel('Time in seconds');ylabel('Accel bias err in y-axis in g');
grid
%
psi_err = zeros(1,n2);
psi_err = psi(1:n2)-psi_h(1:n2)';
for i=2:n2
   if (psi_err(i) > 300*d2r) || (psi_err(i) < -300*d2r)
       psi_err(i) = psi_err(i-1);
   end
end
psi_err_mean = mean(psi_err)*r2d
figure (6)
% subplot(2,1,1),plot(t0(n1:n2),(psi_h(n1:n2)')*r2d,'r',t0(n1:n2),psi(n1:n2)*r2d,'g');
% xlabel('Time in seconds');ylabel('psi_h Angle in deg');
subplot(2,1,1),plot(t0(n1:n2),(psi_err(n1:n2))*r2d,'r');
xlabel('Time in seconds');ylabel('psi_h Angle error in deg');
% subplot(2,1,1),plot(t0(n1:n2),(psi(n1:n2) - psi_h(n1:n2)')*r2d,'r');
% xlabel('Time in seconds');ylabel('psi_h Angle error in deg');
subplot(2,1,2), plot(t0(n1:n2),(bz(n1:n2)-bz_h(n1:n2))*r2d,'r');         
xlabel('Time in seconds');ylabel('Gyro bias err in z-axis in rad/s');
grid
% 
% figure (7)
% subplot(2,1,1), plot(t0(n1:n2),dv_x(n1:n2),'r',t0(n1:n2),xvm_Bh(n1:n2),'g--');
% xlabel('Time in seconds');ylabel('B-frame Velocity in x-axis in m/s');
% subplot(2,1,2), plot(t0(n1:n2),dv_y(n1:n2),'r',t0(n1:n2),yvm_Bh(n1:n2),'g--');
% xlabel('Time in seconds');ylabel('B-frame Velocity in y-axis in m/s');
% 
figure (8)
subplot(2,1,1), plot(t0(n1:n2),x_v_N(n1:n2),'r',t0(n1:n2),xvm_Nh(n1:n2),'g--');
xlabel('Time in seconds');ylabel('N-frame Velocity in x-axis in m/s');
subplot(2,1,2), plot(t0(n1:n2),y_v_N(n1:n2),'r',t0(n1:n2),yvm_Nh(n1:n2),'g--');
xlabel('Time in seconds');ylabel('N-frame Velocity in y-axis in m/s');
% 
figure (9)
subplot(2,1,1), plot(t00(n1:n2),x_p_N(n1:n2),'r',t00(n1:n2),xpm_Nh(n1:n2),'g--');
xlabel('Time in seconds');ylabel('N-frame Position in x-axis in m');
subplot(2,1,2), plot(t00(n1:n2),y_p_N(n1:n2),'r',t00(n1:n2),ypm_Nh(n1:n2),'g--');
xlabel('Time in seconds');ylabel('N-frame Position in y-axis in m');
% 
posx_err = zeros(1,n2);
posy_err = zeros(1,n2);
posx_err = x_p_N(1:n2)-xpm_Nh(1:n2)';
posy_err = y_p_N(1:n2)-ypm_Nh(1:n2)';
posx_err_mean = mean(posx_err)
posy_err_mean = mean(posy_err)
figure (10)
subplot(2,1,1), plot(t00(n1:n2),posx_err(n1:n2),'r');
xlabel('Time in seconds');ylabel('N-frame Position err in x-axis in m');
subplot(2,1,2), plot(t00(n1:n2),posy_err(n1:n2),'r');
xlabel('Time in seconds');ylabel('N-frame Position err in y-axis in m');

