function[xvm_Bh,axm_h]=velosity_computation2_radio(xvm_Bh,axm,bx_h,k,dt)

axm_h(k)=axm(k)-bx_h(k);
axm_h(k-1)=axm(k-1)-bx_h(k-1);

xvm_Bh(k) = xvm_Bh(k-1)+(axm_h(k)+axm_h(k-1))*dt/2;

end