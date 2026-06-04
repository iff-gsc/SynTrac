clear;
addPathConfig2a();

% Aircraft parameters
aircraft = aircraftSyntracCreate();

% Environmental parameters
envir = envirLoadParams('envir_params_default','envir',ft2m(22000));

% Reference position
pos_ref = loadParams( 'flightgearRefPos_params_default' );
pos_ref.alt = ft2m(22000);

% Inital conditons of rigid-body motion
ic.q_bg = euler2Quat([0;0;-1.4]);
ic.V_Kb = [0.44*envir.atmosphere.a;0;0];
ic.s_Kg = [0; 0; 0];
ic.omega_Kb = [0; 0; 0];
% Initial conditions of actuators
ic.act.aileron = [0,0];
ic.act.elevator = 0;
ic.act.rudder = 0;
ic.act.prop = zeros(1,10);
ic.act.htp_trim = 0;

% Joystick parameters
jystck = joystickLoadParams('joystick_params_Xbox_360',2,-1);

% Open Simulink model
open('sim_config2a.slx')


