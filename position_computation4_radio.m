function[xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,wzm_h,psi_h]=position_computation4_radio(xvm_Bh,yvm_Bh,xpm_Nh,ypm_Nh,xvm_Nh,yvm_Nh,psi_h,wzm,bz_h,k,dt,scale_factor_err)

wzm_h(k-1) = (1-scale_factor_err)*(wzm(k-1) - bz_h(k-1));
wzm_h(k) = (1-scale_factor_err)*(wzm(k) - bz_h(k-1));

psi_h(k) = psi_h(k-1) + (wzm_h(k-1)+wzm_h(k))*dt/2;
% psi_h(k) = psi_h(k-1) + wzm_h(k)*dt;
if (psi_h(k)>= 2*pi),
    psi_h(k) = psi_h(k) - 2*pi;
else
    psi_h(k) = psi_h(k);
end
%
% compute N-frame accelerations, velocities, and positions
xvm_Nh(k-1) = cos(psi_h(k-1))*xvm_Bh(k-1)-sin(psi_h(k-1))*yvm_Bh(k-1);
yvm_Nh(k-1) = sin(psi_h(k-1))*xvm_Bh(k-1)+cos(psi_h(k-1))*yvm_Bh(k-1);
xvm_Nh(k) = cos(psi_h(k))*xvm_Bh(k)-sin(psi_h(k))*yvm_Bh(k);
yvm_Nh(k) = sin(psi_h(k))*xvm_Bh(k)+cos(psi_h(k))*yvm_Bh(k);
xpm_Nh(k) = xpm_Nh(k-1)+ (xvm_Nh(k)+xvm_Nh(k-1))*dt/2;
ypm_Nh(k) = ypm_Nh(k-1)+ (yvm_Nh(k)+yvm_Nh(k-1))*dt/2;

end