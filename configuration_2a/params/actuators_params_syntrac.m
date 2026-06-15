% ** parameters of actuator dynamics ** 
%
% typical values see Brockhaus et al. (2011). Flugregelung. Springer on
% page 418.

infEmb = realmax('double');

% natural frequency rudder
param.rudder.omega = 30;
% damping ratio rudder
param.rudder.D = 1;
% maximum deflections and deflection rates rudder
param.rudder.defl_max = deg2rad(25);
param.rudder.defl_min = deg2rad(-25);
param.rudder.defl_rate_max = infEmb;
param.rudder.defl_rate_min = -infEmb;

% natural frequency elevator
param.elevator.omega = 30;
% damping ratio elevator
param.elevator.D = 1;
% maximum deflections and deflection rates elevator
param.elevator.defl_max = deg2rad(13);
param.elevator.defl_min = deg2rad(-23);
param.elevator.defl_rate_max = infEmb;
param.elevator.defl_rate_min = -infEmb;

% natural frequency aileron
param.aileron.omega = 30;
% damping ratio aileron
param.aileron.D = 1;
% maximum deflections and deflection rates aileron
param.aileron.defl_max = deg2rad(14);
param.aileron.defl_min = deg2rad(-14);
param.aileron.defl_rate_max = infEmb;
param.aileron.defl_rate_min = -infEmb;

% natural frequency propeller pitch
% PT2 critically damped (D=1), time constant tau is approx. 2/omega_n
% Target tau = 0.2s -> omega_n = 2/0.2 = 10 rad/s
param.prop_pitch.omega = 10;
% damping ratio propeller pitch
param.prop_pitch.D = 1;
% maximum deflections 
param.prop_pitch.defl_max = deg2rad(15);
param.prop_pitch.defl_min = deg2rad(15);
% maximum pitch rates
param.prop_pitch.defl_rate_max =  deg2rad(20);
param.prop_pitch.defl_rate_min = -deg2rad(20);

% htp trim dynamics
param.htp_trim.omega = 1;
param.htp_trim.D = 1;
param.htp_trim.defl_max = deg2rad(10);
param.htp_trim.defl_min = deg2rad(-10);
param.htp_trim.defl_rate_max = infEmb;
param.htp_trim.defl_rate_min = -infEmb;