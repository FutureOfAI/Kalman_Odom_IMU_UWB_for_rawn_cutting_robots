function [wzm]=Wzm1(wz,bz,m,n,d2r,sig_arw_0)
wzm=zeros(m);
sig_arw=sig_arw_0*d2r;
narw=normrnd(0,sig_arw,1,n);
for i=1:n
    wzm(i) = wz(i)+bz(i)+narw(i); 
end
end