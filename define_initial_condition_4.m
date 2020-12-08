function [xz_h,P00_z]=define_initial_condition_4(bz0,xperr,yperr,psierr,d2r)

xz_h=zeros(4,1);
xz_h(4,1) = bz0;       

P00_z = zeros(4,4);
P00_z(1,1) = xperr^2;
P00_z(2,2) = yperr^2;
P00_z(3,3) = (1*psierr*d2r)^2;
P00_z(4,4) =100* bz0^2;

end