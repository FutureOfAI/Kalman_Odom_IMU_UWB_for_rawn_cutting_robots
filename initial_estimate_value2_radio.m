function [xvm_Bh,bx_h]=initial_estimate_value2_radio(m,x_v_B)

bx_h=zeros(m);

xvm_Bh=zeros(m);

bx_h(1)=0*9.8;

xvm_Bh(1)=0;%14s-1.5609;%2.2313%0*x_v_N(1) ;%+ xverr*rand;

end