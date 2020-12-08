function [bp]=Biasbp1(dt,n,m,bp0,sig_pr_0)
sig_pr=sig_pr_0;
npb=normrnd(0,sig_pr,1,n);
bp=zeros(m);
bp(1)=bp0;
for i=1:(n-1)
    bp(i+1)=bp(i)+npb(i)*dt;  %nab 有1*n 筆資料打成 nab(i) 是指定用第 i 筆資料 不打的話會出錯因為 nab 是矩陣不是變數
end
end