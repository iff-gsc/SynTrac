clear;
addPathConfig2a();

% Aircraft parameters
aircraft = aircraftSyntracCreate();

% Environmental parameters
envir = envirLoadParams('envir_params_default','envir',ft2m(22000));

% Reference position
pos_ref = loadParams( 'flightgearRefPos_params_default' );
pos_ref.alt = ft2m(22000);

% Trim aircraft
[ic,u0,~,lin,lin_red] = trimConfig2a(aircraft, 125, pos_ref.alt);

% Joystick parameters
jystck = joystickLoadParams('joystick_params_Xbox_360',2,-1);

% Open Simulink model
open('sim_config2a.slx')


