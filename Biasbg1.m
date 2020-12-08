function [bz]=Biasbg1(dt,n,m,bz0,d2r,sig_rrw_0)
sig_rrw=sig_rrw_0*d2r;
nrrw=normrnd(0,sig_rrw,1,n);
bz=zeros(m);
bz(1)=bz0;
for i=1:(n-1)
    bz(i+1)=bz(i)+nrrw(i)*dt;  %nab 有1*n 筆資料打成 nab(i) 是指定用第 i 筆資料 不打的話會出錯因為 nab 是矩陣不是變數
end
end