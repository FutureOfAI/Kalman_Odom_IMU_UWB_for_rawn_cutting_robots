function [xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,axm_h,aym_h,wzm_h,psi_h,bz_h]=initial_estimate_value4_radio(m,wzm,axm,aym,d2r,position_x,position_y,psi,bz0)
%
axm_h=zeros(m);
aym_h=zeros(m);

xpm_Nh=zeros(m);
ypm_Nh=zeros(m);
xvm_Nh=zeros(m);
yvm_Nh=zeros(m);

wzm_h=zeros(m);
psi_h=zeros(m);
bz_h=zeros(m);

axm_h(1)=axm(1);
aym_h(1)=aym(1);

xvm_Nh(1)=0;
yvm_Nh(1)=0;

xpm_Nh(1)=position_x;%4.7158;%4.8;%14s4.538;%x_p_N(1); %+ xperr*rand;
ypm_Nh(1)=position_y;%6.6467;%6.4;%14s6.066;%y_p_N(1) ;% + yperr*rand;

wzm_h(1)=wzm(1);
psi_h(1)=psi(1);
bz_h(1)=bz0;
% bz_h(1)=-4.37*d2r;

end