% ===============================================================
% Program name: trajectory2.m
% 7/20/2016
% 11/22/2016
function [x_p,x_v,x_a,y_p,y_v,y_a,roll_angle,yaw_angle,yaw_rate,time] = trajectory2(v,roll_a,Ts,T1)
% ===============================================================
% Produce platform flight profile for calibrate the inertial sensor errors
% ===============================================================
r2d=(180/pi);
d2r=(pi/180);
g = 9.8;                        % gravity constant in m/sec^2
% ===============================================================
% Generate roll angle and introduce bank angles to create yaw terms
% ===============================================================
% define the platform constant ground speed, roll angle acceleration
% ===============================================================
%v = 10;                         % ground speed in m/sec
%roll_a = 1.25*d2r;                 % band-band acceleration in m/sec^2
%Ts = 4 + 0.5;
%T1 = 14.3 + 0.5;
T_1 = Ts + T1;
T2 = T1;
T_2 = Ts + T2;
T3 = T1;
T_3 = Ts + T3;
T4 = T1;
T_4 = Ts + T4;
T5 = T1;
T_5 = Ts + T5;
T6 = T1;
T_6 = Ts + T6;
T7 = T1;
T_7 = Ts + T7;
T8 = T1;
T_8 = Ts + T8;
T = 4*Ts + 2*T1 + T2;
%
px0 = 54.958;
py0 = -189;
% ===============================================================
t = 0;
dt = 0.01;
N = (Ts+T1)/dt;
N2 = (Ts+T2)/dt;
%N = Ts/dt;
% ===============================================================
for i = 1:N,
    time(i) = t;
% generate roll angle
if (t <= Ts),
    if (t <= Ts/2),
        roll_angle(i) = (roll_a/2)*t^2;
    else
        %roll_angle(i) = (roll_a/2)*Ts^2 - (5*roll_a*Ts/4)*t + (roll_a)*t^2;
        roll_angle(i) = -(roll_a/4)*Ts^2 + (roll_a*Ts)*t - (roll_a/2)*t^2;
    end
else
    roll_angle(i) = roll_angle(i-1);
end
% generate yaw angle rate
yaw_rate(i) = (g*tan(roll_angle(i)))/v;
% produce the yaw angle by integrating the above yaw rate
if (i == 1),
    yaw_angle(i) = 0;
else
    yaw_angle(i) = yaw_angle(i-1) + (yaw_rate(i) + yaw_rate(i-1))*dt/2;
end
% compute platform vecolities and accelerations
x_v(i) = v*cos(yaw_angle(i));
y_v(i) = v*sin(yaw_angle(i));
x_a(i) = -v*yaw_rate(i)*sin(yaw_angle(i));
y_a(i) = v*yaw_rate(i)*cos(yaw_angle(i));
% compute platform positions
if (i ==1),
    x_p(i) = px0;
    y_p(i) = py0;
else
x_p(i) = x_p(i-1) + x_v(i)*dt;
y_p(i) = y_p(i-1) + y_v(i)*dt;
end
    t = t + dt;
end
% ============================================
% end of first roll or bank turn (positive roll turn)
% ============================================
% start the next bank turn (negative bank turn)
% ============================================
%roll_a = - roll_a;
for k = i+1:i+N,
   time(k) = t;
   tt = t - T_1;
% generate roll angle
if (t <= T_1+Ts),
    if (t <= T_1+Ts/2),
        roll_angle(k) = roll_angle(i) - (roll_a/2)*tt^2;
    else
        %roll_angle(i) = (roll_a/2)*Ts^2 - (5*roll_a*Ts/4)*t + (roll_a)*t^2;
        roll_angle(k) = (roll_a/2)*Ts^2 - (roll_a*Ts)*tt + (roll_a/2)*tt^2;
    end
else
    roll_angle(k) = roll_angle(k-1);
end
% generate yaw angle rate
yaw_rate(k) = (g*tan(roll_angle(k)))/v;
% produce the yaw angle by integrating the above yaw rate
if (k == 1),
    yaw_angle(k) = 0;
else
    yaw_angle(k) = yaw_angle(k-1) + (yaw_rate(k) + yaw_rate(k-1))*dt/2;
end
% compute platform vecolities
x_v(k) = v*cos(yaw_angle(k));
y_v(k) = v*sin(yaw_angle(k));
x_a(k) = -v*yaw_rate(k)*sin(yaw_angle(k));
y_a(k) = v*yaw_rate(k)*cos(yaw_angle(k));
% compute platform positions
if (k ==1),
    x_p(k) = px0;
    y_p(k) = py0;
else
x_p(k) = x_p(k-1) + x_v(k)*dt;
y_p(k) = y_p(k-1) + y_v(k)*dt;
end
    t = t + dt;
end
% =========================================================
% end of second roll or bank turn 
% =========================================================
% Ready for next bank turn (positive band turn again)
% =========================================================
for k1 = k+1:k+N,
   time(k1) = t;
   tt = t - T_1 - T_2;
% generate roll angle
if (t <= T_1+T_2+Ts),
    if (t <= T_1+T_2+Ts/2),
        roll_angle(k1) = roll_angle(k) + (roll_a/2)*tt^2;
    else
        %roll_angle(i) = (roll_a/2)*Ts^2 - (5*roll_a*Ts/4)*t + (roll_a)*t^2;
        roll_angle(k1) = -(roll_a/4)*Ts^2 + (roll_a*Ts)*tt - (roll_a/2)*tt^2;
    end
else
    roll_angle(k1) = roll_angle(k1-1);
end
% generate yaw angle rate
yaw_rate(k1) = (g*tan(roll_angle(k1)))/v;
% produce the yaw angle by integrating the above yaw rate
if (k1 == 1),
    yaw_angle(k1) = 0;
else
    yaw_angle(k1) = yaw_angle(k1-1) + (yaw_rate(k1) + yaw_rate(k1-1))*dt/2;
end
% compute platform vecolities
x_v(k1) = v*cos(yaw_angle(k1));
y_v(k1) = v*sin(yaw_angle(k1));
x_a(k1) = -v*yaw_rate(k1)*sin(yaw_angle(k1));
y_a(k1) = v*yaw_rate(k1)*cos(yaw_angle(k1));
% compute platform positions
if (k1 ==1),
    x_p(k1) = px0;
    y_p(k1) = py0;
else
x_p(k1) = x_p(k1-1) + x_v(k1)*dt;
y_p(k1) = y_p(k1-1) + y_v(k1)*dt;
end
    t = t + dt;
end
% =========================================================
% end of third roll or bank turn 
% =========================================================
% Ready for next bank turn (negative band turn again)
% =========================================================
for k2 = k1+1:k1+N,
   time(k2) = t;
   tt = t - T_1 - T_2 - T_3;
% generate roll angle
if (t <= T_1+T_2+T_3+Ts),
    if (t <= T_1+T_2+T_3+Ts/2),
        roll_angle(k2) = roll_angle(k1) - (roll_a/2)*tt^2;
    else
        %roll_angle(i) = (roll_a/2)*Ts^2 - (5*roll_a*Ts/4)*t + (roll_a)*t^2;
        roll_angle(k2) = (roll_a/2)*Ts^2 - (roll_a*Ts)*tt + (roll_a/2)*tt^2;
    end
else
    roll_angle(k2) = roll_angle(k2-1);
end
% generate yaw angle rate
yaw_rate(k2) = (g*tan(roll_angle(k2)))/v;
% produce the yaw angle by integrating the above yaw rate
if (k2 == 1),
    yaw_angle(k2) = 0;
else
    yaw_angle(k2) = yaw_angle(k2-1) + (yaw_rate(k2) + yaw_rate(k2-1))*dt/2;
end
% compute platform vecolities
x_v(k2) = v*cos(yaw_angle(k2));
y_v(k2) = v*sin(yaw_angle(k2));
x_a(k2) = -v*yaw_rate(k2)*sin(yaw_angle(k2));
y_a(k2) = v*yaw_rate(k2)*cos(yaw_angle(k2));
% compute platform positions
if (k2 ==1),
    x_p(k2) = px0;
    y_p(k2) = py0;
else
x_p(k2) = x_p(k2-1) + x_v(k2)*dt;
y_p(k2) = y_p(k2-1) + y_v(k2)*dt;
end
    t = t + dt;
end
% =========================================
% end of fouth roll or bank turn 
% =========================================
% Ready for next bank turn (positive band turn again)
% ==================================================
for k3 = k2+1:k2+N,
   time(k3) = t;
   tt = t - T_1 - T_2 - T_3 - T_4;
% generate roll angle
if (t <= T_1+T_2+T_3+T_4+Ts),
    if (t <= T_1+T_2+T_3++T_4+Ts/2),
        roll_angle(k3) = roll_angle(k2) + (roll_a/2)*tt^2;
    else
        %roll_angle(i) = (roll_a/2)*Ts^2 - (5*roll_a*Ts/4)*t + (roll_a)*t^2;
        roll_angle(k3) = -(roll_a/4)*Ts^2 + (roll_a*Ts)*tt - (roll_a/2)*tt^2;
    end
else
    roll_angle(k3) = roll_angle(k3-1);
end
% generate yaw angle rate
yaw_rate(k3) = (g*tan(roll_angle(k3)))/v;
% produce the yaw angle by integrating the above yaw rate
if (k3 == 1),
    yaw_angle(k3) = 0;
else
    yaw_angle(k3) = yaw_angle(k3-1) + (yaw_rate(k3) + yaw_rate(k3-1))*dt/2;
end
% compute platform vecolities
x_v(k3) = v*cos(yaw_angle(k3));
y_v(k3) = v*sin(yaw_angle(k3));
x_a(k3) = -v*yaw_rate(k3)*sin(yaw_angle(k3));
y_a(k3) = v*yaw_rate(k3)*cos(yaw_angle(k3));
% compute platform positions
if (k3 ==1),
    x_p(k3) = px0;
    y_p(k3) = py0;
else
x_p(k3) = x_p(k3-1) + x_v(k3)*dt;
y_p(k3) = y_p(k3-1) + y_v(k3)*dt;
end
    t = t + dt;
end
% =========================================
% end of fifth roll or bank turn 
% =========================================
% Ready for next bank turn (negative band turn again)
% ===============================================
for k4 = k3+1:k3+N,
   time(k4) = t;
   tt = t - T_1 - T_2 - T_3 - T_4 - T_5;
% generate roll angle
if (t <= T_1+T_2+T_3+T_4+T_5+Ts),
    if (t <= T_1+T_2+T_3+T_4+T_5+Ts/2),
        roll_angle(k4) = roll_angle(k3) - (roll_a/2)*tt^2;
    else
        %roll_angle(i) = (roll_a/2)*Ts^2 - (5*roll_a*Ts/4)*t + (roll_a)*t^2;
        roll_angle(k4) = (roll_a/2)*Ts^2 - (roll_a*Ts)*tt + (roll_a/2)*tt^2;
    end
else
    roll_angle(k4) = roll_angle(k4-1);
end
% generate yaw angle rate
yaw_rate(k4) = (g*tan(roll_angle(k4)))/v;
% produce the yaw angle by integrating the above yaw rate
if (k4 == 1),
    yaw_angle(k4) = 0;
else
    yaw_angle(k4) = yaw_angle(k4-1) + (yaw_rate(k4) + yaw_rate(k4-1))*dt/2;
end
% compute platform vecolities and accelerations
x_v(k4) = v*cos(yaw_angle(k4));
y_v(k4) = v*sin(yaw_angle(k4));
x_a(k4) = -v*yaw_rate(k4)*sin(yaw_angle(k4));
y_a(k4) = v*yaw_rate(k4)*cos(yaw_angle(k4));
% compute platform positions
if (k4 ==1),
    x_p(k4) = px0;
    y_p(k4) = py0;
else
x_p(k4) = x_p(k4-1) + x_v(k4)*dt;
y_p(k4) = y_p(k4-1) + y_v(k4)*dt;
end
    t = t + dt;
end
% =========================================
% end of six roll or bank turn 
% ========================================
% Ready for next bank turn (positive band turn again)
% ==================================================
for k5 = k4+1:k4+N,
   time(k5) = t;
   tt = t - T_1 - T_2 - T_3 - T_4 - T_5 - T_6;
% generate roll angle
if (t <= T_1+T_2+T_3+T_4+T_5+T_6+Ts),
    if (t <= T_1+T_2+T_3++T_4+T_5+T_6+Ts/2),
        roll_angle(k5) = roll_angle(k4) + (roll_a/2)*tt^2;
    else
        %roll_angle(i) = (roll_a/2)*Ts^2 - (5*roll_a*Ts/4)*t + (roll_a)*t^2;
        roll_angle(k5) = -(roll_a/4)*Ts^2 + (roll_a*Ts)*tt - (roll_a/2)*tt^2;
    end
else
    roll_angle(k5) = roll_angle(k5-1);
end
% generate yaw angle rate
yaw_rate(k5) = (g*tan(roll_angle(k5)))/v;
% produce the yaw angle by integrating the above yaw rate
if (k5 == 1),
    yaw_angle(k5) = 0;
else
    yaw_angle(k5) = yaw_angle(k5-1) + (yaw_rate(k5) + yaw_rate(k5-1))*dt/2;
end
% compute platform vecolities
x_v(k5) = v*cos(yaw_angle(k5));
y_v(k5) = v*sin(yaw_angle(k5));
x_a(k5) = -v*yaw_rate(k5)*sin(yaw_angle(k5));
y_a(k5) = v*yaw_rate(k5)*cos(yaw_angle(k5));
% compute platform positions
if (k5 ==1),
    x_p(k5) = px0;
    y_p(k5) = py0;
else
x_p(k5) = x_p(k5-1) + x_v(k5)*dt;
y_p(k5) = y_p(k5-1) + y_v(k5)*dt;
end
    t = t + dt;
end
% =========================================
% end of seventh roll or bank turn 
% =========================================
% Ready for next bank turn (negative band turn again)
% ===============================================
for k6 = k5+1:k5+N,
   time(k6) = t;
   tt = t - T_1 - T_2 - T_3 - T_4 - T_5 - T_6 - T_7;
% generate roll angle
if (t <= T_1+T_2+T_3+T_4+T_5+T_6+T_7+Ts),
    if (t <= T_1+T_2+T_3+T_4+T_5+T_6+T_7+Ts/2),
        roll_angle(k6) = roll_angle(k5) - (roll_a/2)*tt^2;
    else
        %roll_angle(i) = (roll_a/2)*Ts^2 - (5*roll_a*Ts/4)*t + (roll_a)*t^2;
        roll_angle(k6) = (roll_a/2)*Ts^2 - (roll_a*Ts)*tt + (roll_a/2)*tt^2;
    end
else
    roll_angle(k6) = roll_angle(k6-1);
end
% generate yaw angle rate
yaw_rate(k6) = (g*tan(roll_angle(k6)))/v;
% produce the yaw angle by integrating the above yaw rate
if (k6 == 1),
    yaw_angle(k6) = 0;
else
    yaw_angle(k6) = yaw_angle(k6-1) + (yaw_rate(k6) + yaw_rate(k6-1))*dt/2;
end
% compute platform vecolities
x_v(k6) = v*cos(yaw_angle(k6));
y_v(k6) = v*sin(yaw_angle(k6));
x_a(k6) = -v*yaw_rate(k6)*sin(yaw_angle(k6));
y_a(k6) = v*yaw_rate(k6)*cos(yaw_angle(k6));
% compute platform positions
if (k6 ==1),
    x_p(k6) = px0;
    y_p(k6) = py0;
else
x_p(k6) = x_p(k6-1) + x_v(k6)*dt;
y_p(k6) = y_p(k6-1) + y_v(k6)*dt;
end
    t = t + dt;
end
% =========================================
% end of eighth roll or bank turn 
% =========================================
% Ready for next bank turn (positive band turn again)
% ==================================================
for k7 = k6+1:k6+N,
   time(k7) = t;
   tt = t - T_1 - T_2 - T_3 - T_4 - T_5 - T_6 - T_7 - T_8;
% generate roll angle
if (t <= T_1+T_2+T_3+T_4+T_5+T_6+T_7+T_8+Ts),
    if (t <= T_1+T_2+T_3++T_4+T_5+T_6+T_7+T_8+Ts/2),
        roll_angle(k7) = roll_angle(k6) + (roll_a/2)*tt^2;
    else
        %roll_angle(i) = (roll_a/2)*Ts^2 - (5*roll_a*Ts/4)*t + (roll_a)*t^2;
        roll_angle(k7) = -(roll_a/4)*Ts^2 + (roll_a*Ts)*tt - (roll_a/2)*tt^2;
    end
else
    roll_angle(k7) = roll_angle(k7-1);
end
% generate yaw angle rate
yaw_rate(k7) = (g*tan(roll_angle(k7)))/v;
% produce the yaw angle by integrating the above yaw rate
if (k5 == 1),
    yaw_angle(k7) = 0;
else
    yaw_angle(k7) = yaw_angle(k7-1) + (yaw_rate(k7) + yaw_rate(k7-1))*dt/2;
end
% compute platform vecolities
x_v(k7) = v*cos(yaw_angle(k7));
y_v(k7) = v*sin(yaw_angle(k7));
x_a(k7) = -v*yaw_rate(k7)*sin(yaw_angle(k7));
y_a(k7) = v*yaw_rate(k7)*cos(yaw_angle(k7));
% compute platform positions
if (k7 ==1),
    x_p(k7) = px0;
    y_p(k7) = py0;
else
x_p(k7) = x_p(k7-1) + x_v(k7)*dt;
y_p(k7) = y_p(k7-1) + y_v(k7)*dt;
end
    t = t + dt;
end
% =========================================
% end of nineth roll or bank turn 
% =========================================
end