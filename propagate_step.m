function [sensor_step,propagation_step]=propagate_step(T,delta_t,delta_s)
t_simulation = T;            % 10 seconds simulation time
simulation_step = t_simulation/delta_t;
sensor_step = t_simulation/delta_s;
propagation_step = simulation_step/sensor_step;             % =delta_s/delta_t=10*dt/dt=10
end