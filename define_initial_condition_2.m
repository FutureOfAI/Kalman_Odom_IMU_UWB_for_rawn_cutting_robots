function [xz_h,P00_z]=define_initial_condition_2(bx0,xverr)

xz_h=zeros(2,1);      

P00_z = zeros(2,2);
P00_z(1,1) = xverr^2;
P00_z(2,2) = 1*bx0^2;
end