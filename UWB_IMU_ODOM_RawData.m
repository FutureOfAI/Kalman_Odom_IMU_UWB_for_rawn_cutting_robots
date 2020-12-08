% clc;
% clear all;
% close all;

data = load('2019-11-18-17-11-32sensor.txt');

[row, col] = size(data); % get data size
d2r = pi/180;
r2d = 180/pi;
L = 0.32; % m

start_bit = 10;
t = start_bit:row;

accx = data(:,10);
accy = data(:,11);
accz = data(:,12);

grox = data(:,7)*d2r;
groy = data(:,8)*d2r;
groz = data(:,9)*d2r + 0.00316;

roll = data(:,13)*d2r;
pitch = data(:,14)*d2r;
yaw = data(:,15)*d2r+186*d2r;

for i=1:1000
   if yaw(i) > 340*d2r
       yaw(i) = yaw(i)-2*pi;
   end
end

% uwb distance between anchors and tag
anchor = zeros(row, 4);
uwb_state = data(:,1);
anchor(:,1) = data(:,2);
anchor(:,2) = data(:,3);
anchor(:,3) = data(:,4);
anchor(:,4) = data(:,5);

% odom velocity and length
dist_l = data(:,16);
dist_r = data(:,17);
vel_l = data(:,18);
vel_r = data(:,19);
% odom delta length
delta_dis_l = zeros(row,1);
delta_dis_r = zeros(row,1);
delta_dis_l(1) = dist_l(1);
delta_dis_r(1) = dist_r(1);
for i = 2:row 
   delta_dis_l(i) =  dist_l(i) - dist_l(i-1);
   delta_dis_r(i) =  dist_r(i) - dist_r(i-1);
end
    
dist_avg = (delta_dis_r + delta_dis_l)/2;
vel_avg = (vel_r + vel_l)/2;

omega = (delta_dis_r - delta_dis_l)/L; % in rad

omega_sum = zeros(row,1);

radius = zeros(row,1);
delta_x_B = zeros(row,1);
delta_y_B = zeros(row,1);
delta_x_N = zeros(row,1);
delta_y_N = zeros(row,1);
dv_x = zeros(row,1);
dv_y = zeros(row,1);
delta_x_B = zeros(row,1);
delta_y_B = zeros(row,1);
odom_x = zeros(row,1);
odom_y = zeros(row,1);
% delta velocity x and y of vehicle in B-frame
odom_x(1) = 3.84;
odom_y(1) = 0;
for i=2:row
    omega_sum(i) = omega_sum(i-1) + omega(i);
    omega_sum(i) = yaw(i);
    if omega(i) ~= 0
        radius(i) = (delta_dis_r(i) + delta_dis_l(i))/(2*omega(i));
        delta_x_B(i) = radius(i)*(1-cos(omega(i)));
        delta_y_B(i) = radius(i)*sin(omega(i));
        delta_x_N(i) = cos(omega_sum(i))*delta_x_B(i)-sin(omega_sum(i))*delta_y_B(i);
        delta_y_N(i) = sin(omega_sum(i))*delta_x_B(i)+cos(omega_sum(i))*delta_y_B(i);
        dv_x(i) = delta_x_B(i)*10; % 10HZ
        dv_y(i) = delta_y_B(i)*10;
        odom_x(i) = odom_x(i-1) + delta_x_N(i);
        odom_y(i) = odom_y(i-1) + delta_y_N(i);
    else
        delta_x_N(i) = dist_avg(i)*cos(omega_sum(i)+pi/2);
        delta_y_N(i) = dist_avg(i)*sin(omega_sum(i)+pi/2);
%         delta_x_B(i) = cos(yaw(i))*delta_x_N(i)+sin(yaw(i))*delta_y_N(i);
%         delta_y_B(i) = -sin(yaw(i))*delta_x_N(i)+cos(yaw(i))*delta_y_N(i);
%         dv_x(i) = delta_x_B(i)*10; % 10HZ
%         dv_y(i) = delta_y_B(i)*10;
        odom_x(i) = odom_x(i-1) + delta_x_N(i);
        odom_y(i) = odom_y(i-1) + delta_y_N(i);
    end

% % calculate velocity of odometer in B-frame
%     if omega(i) ~= 0
%         radius(i) = (delta_dis_r(i) + delta_dis_l(i))/(2*omega(i));
%         dv_x(i) = -(vel_avg(i)/2)*omega(i)*0.1;
%         dv_y(i) = vel_avg(i);
%         delta_x_B(i) = dv_x(i)*0.1;
%         delta_y_B(i) = dv_y(i)*0.1;
%         odom_x(i) = odom_x(i-1) + cos(omega_sum(i))*delta_x_B(i) - sin(omega_sum(i))*delta_y_B(i);
%         odom_y(i) = odom_y(i-1) + sin(omega_sum(i))*delta_x_B(i) + cos(omega_sum(i))*delta_y_B(i);
%     else
%         delta_x_N(i) = dist_avg(i)*cos(omega_sum(i)+pi/2);
%         delta_y_N(i) = dist_avg(i)*sin(omega_sum(i)+pi/2);
%         odom_x(i) = odom_x(i-1) + delta_x_N(i);
%         odom_y(i) = odom_y(i-1) + delta_y_N(i);
%     end

% % calculate velocity of odometer in B-frame
%     dv_x(i) = -vel_avg(i)*sin(omega(i)*0.1);
%     dv_y(i) = vel_avg(i)*cos(omega(i)*0.1);
%     delta_x_B(i) = dv_x(i)*0.1;
%     delta_y_B(i) = dv_y(i)*0.1;
%     odom_x(i) = odom_x(i-1) + cos(omega_sum(i))*delta_x_B(i) - sin(omega_sum(i))*delta_y_B(i);
%     odom_y(i) = odom_y(i-1) + sin(omega_sum(i))*delta_x_B(i) + cos(omega_sum(i))*delta_y_B(i);
end

for i=2:row
    if(uwb_state(i) ~= 15)
        for j = 1:4
            if ~bitget(uwb_state(i),j)
                anchor(i,j) = anchor(i-1,j);
            end
        end
    end
end

% %  low-pass filter uwb distance to remove errors
% order = 1;
% filtCutoff = 0.1;
% [b, a] = butter(order, (2*filtCutoff)/10, 'low');
% anchor_LP = filtfilt(b,a,anchor);

anchor_cal = zeros(row, 4);
anchor_cal(:,1) = 0.9865*anchor(:,1) - 0.3350;
anchor_cal(:,2) = 0.9882*anchor(:,2) - 0.3713;
anchor_cal(:,3) = 0.9952*anchor(:,3) - 0.4922;
anchor_cal(:,4) = 0.9926*anchor(:,4) - 0.385;
% dt_uwb = data(:,7);
% dt_imu = data(:,20);
% dt_freq = data(:,21);

% for i = t
%     % acc
%     if data(i,10) == 0
%         accx(i) = data(i-1,10);
%     end
%     if data(i,11) == 0
%         accy(i) = data(i-1,11);
%     end
%     if data(i,12) == 0
%         accz(i) = data(i-1,12);
%     end
%     % gro
%     if data(i,7) == 0
%         grox(i) = data(i-1,7);
%     end
%     if data(i,8) == 0
%         groy(i) = data(i-1,8);
%     end
%     if data(i,9) == 0
%         groz(i) = data(i-1,9);
%     end    
%     % euler
%     if data(i,13) == 0
%         roll(i) = data(i-1,13);
%     end
%     if data(i,14) == 0
%         pitch(i) = data(i-1,14);
%     end
%     if data(i,15) == 0
%         yaw(i) = data(i-1,15);
%     end
%     % uwb
%     if data(i,2) == 0
%         anchor(i,1) = data(i-1,2);
%     end
%     if data(i,3) == 0
%         anchor(i,2) = data(i-1,3);
%     end
%     if data(i,4) == 0
%         anchor(i,3) = data(i-1,4);
%     end
%     if data(i,5) == 0
%         anchor(i,4) = data(i-1,5);
%     end
%     % odom
%     if data(i,16) == 0
%         dist_l(i) = data(i-1,16);
%     end
%     if data(i,17) == 0
%         dist_r(i) = data(i-1,17);
%     end
%     if data(i,18) == 0
%         vel_l(i) = data(i-1,18);
%     end
%     if data(i,19) == 0
%         vel_r(i) = data(i-1,19);
%     end 
%     % dt
%     if data(i,7) == 0
%         dt_uwb(i) = data(i-1,7);
%     end
%     if data(i,20) == 0
%         dt_imu(i) = data(i-1,20);
%     end
%     if data(i,21) == 0
%         dt_freq(i) = data(i-1,21);
%     end    
% end

% figure (1)
% subplot(311);
% plot(t,accx(t),'r')
% xlabel('Time in 10hz');ylabel('x accelerate in g');
% subplot(312);
% plot(t,accy(t),'r')
% xlabel('Time in 10hz');ylabel('y accelerate in g');
% subplot(313);
% plot(t,accz(t),'r')
% xlabel('Time in 10hz');ylabel('z accelerate in g');
% 
figure (10)
subplot(311);
plot(t,accx(t),'r')
xlabel('Time in 10hz');ylabel('x accelerate in g');
subplot(312);
plot(t,accy(t),'r')
xlabel('Time in 10hz');ylabel('y accelerate in g');
subplot(313);
plot(t,groz(t)*d2r,'r')
xlabel('Time in 10hz');ylabel('z groscope in rad/s');
% 
% figure (2)
% subplot(311);
% plot(t,grox(t)*d2r,'r')
% xlabel('Time in 10hz');ylabel('x groscope in rad/s');
% subplot(312);
% plot(t,groy(t)*d2r,'r')
% xlabel('Time in 10hz');ylabel('y groscope in rad/s');
% subplot(313);
% plot(t,groz(t)*d2r,'r')
% xlabel('Time in 10hz');ylabel('z groscope in rad/s');
% % 
% figure (3)
% subplot(311);
% plot(t,roll(t)*d2r,'r')
% xlabel('Time in 10hz');ylabel('roll in rad');
% subplot(312);
% plot(t,pitch(t)*d2r,'r')
% xlabel('Time in 10hz');ylabel('pitch in rad');
% subplot(313);
% plot(t,yaw(t)*d2r,'r')
% xlabel('Time in 10hz');ylabel('yaw in rad');
% % 
figure (3)
subplot(411);
plot(t,anchor(t,1),'r')
xlabel('Time in 10hz');ylabel('anchor1 distance in m');
subplot(412);
plot(t,anchor(t,2),'r')
xlabel('Time in 10hz');ylabel('anchor2 distance in m');
subplot(413);
plot(t,anchor(t,3),'r')
xlabel('Time in 10hz');ylabel('anchor3 distance in m');
subplot(414);
plot(t,anchor(t,4),'r')
xlabel('Time in 10hz');ylabel('anchor4 distance in m');
%
% figure (6)
% subplot(311);
% plot(t,delta_dis_l(t),'r',t,delta_dis_r(t),'g',t,dist_avg(t),'b')
% xlabel('Time in 10hz');ylabel('odom distance in m');
% subplot(312);
% plot(t,vel_l(t),'r',t,vel_r(t),'g',t,vel_avg(t),'b')
% xlabel('Time in 10hz');ylabel('odom velocity in m/s');
% subplot(313);
% plot(t,dv_x(t),'r',t,dv_y(t),'g')
% xlabel('Time in 10hz');ylabel('odom distance in x_y');
% % 
% figure (7)
% subplot(311);
% plot(t,dt_uwb(t),'r')
% xlabel('Time in 10hz');ylabel('uwb frequency in s');
% subplot(312);
% plot(t,dt_imu(t),'r')
% xlabel('Time in 10hz');ylabel('imu frequency in s');
% subplot(313);
% plot(t,dt_freq(t),'r')
% xlabel('Time in 10hz');ylabel('save frequency in s');

