function[xvm_Nh,bx_h]=upon_radiosensor_measurement_Vx(xvm_Nh,bx_h,z_update,k)

xvm_Nh(k)=xvm_Nh(k)+z_update(1);
bx_h(k) = bx_h(k) + z_update(2);

end