function [ba]=Biasba1(dt,n,m,ba0,sig_ar_0)
sig_ar =sig_ar_0;
nba=normrnd(0,sig_ar,1,n);
ba=zeros(m);
ba(1)=ba0;
for i=1:(n-1)
    ba(i+1)=ba(i)+nba(i)*dt;  %nab ��1*n ����ƥ��� nab(i) �O���w�β� i ����� �������ܷ|�X���]�� nab �O�x�}���O�ܼ�
end
end