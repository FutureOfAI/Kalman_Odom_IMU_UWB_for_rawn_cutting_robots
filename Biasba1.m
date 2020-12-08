function [ba]=Biasba1(dt,n,m,ba0,sig_ar_0)
sig_ar =sig_ar_0;
nba=normrnd(0,sig_ar,1,n);
ba=zeros(m);
ba(1)=ba0;
for i=1:(n-1)
    ba(i+1)=ba(i)+nba(i)*dt;  %nab 有1*n 筆資料打成 nab(i) 是指定用第 i 筆資料 不打的話會出錯因為 nab 是矩陣不是變數
end
end