function [bz]=Biasbg1(dt,n,m,bz0,d2r,sig_rrw_0)
sig_rrw=sig_rrw_0*d2r;
nrrw=normrnd(0,sig_rrw,1,n);
bz=zeros(m);
bz(1)=bz0;
for i=1:(n-1)
    bz(i+1)=bz(i)+nrrw(i)*dt;  %nab ��1*n ����ƥ��� nab(i) �O���w�β� i ����� �������ܷ|�X���]�� nab �O�x�}���O�ܼ�
end
end