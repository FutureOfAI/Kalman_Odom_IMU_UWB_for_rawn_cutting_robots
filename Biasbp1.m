function [bp]=Biasbp1(dt,n,m,bp0,sig_pr_0)
sig_pr=sig_pr_0;
npb=normrnd(0,sig_pr,1,n);
bp=zeros(m);
bp(1)=bp0;
for i=1:(n-1)
    bp(i+1)=bp(i)+npb(i)*dt;  %nab ��1*n ����ƥ��� nab(i) �O���w�β� i ����� �������ܷ|�X���]�� nab �O�x�}���O�ܼ�
end
end