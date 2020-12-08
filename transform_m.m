function [axm,aym]=transform_m(x_a,bx,y_a,by,m,n,sig_bx_0,sig_by_0)
sig_bx=sig_bx_0;
sig_by=sig_by_0;
nx=normrnd(0,sig_bx,1,n);
ny=normrnd(0,sig_by,1,n);
axm=zeros(m);
aym=zeros(m);
for i=1:n
    axm(i)=x_a(i)+bx(i)+nx(i);
    aym(i)=y_a(i)+by(i)+ny(i);
end
end