function[phi_z,Q_z]=define_Dymamic_equation2_radio(F_z,Q_z,sig_xr,sig_yr,dt)

I=eye(2);
%phi_z = expm(F_z*dt);
phi_z = I+(F_z*dt);

Q_z(1,1) = 150000*sig_xr^2;
Q_z(2,2) = 150000*sig_yr^2;

end