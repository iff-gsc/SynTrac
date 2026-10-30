clear;
addPathConfig2a();

% Define operating point
V = 136.33;
h = ft2m(22000);

% Environmental parameters
envir = envirLoadParams('envir_params_default','envir',h);

Ma = V/envir.atmosphere.a;

% Aircraft parameters
aircraft = aircraftSyntracCreate('Mach',Ma);

% Reference position
pos_ref = loadParams( 'flightgearRefPos_params_default' );
pos_ref.alt = h;

% Trim aircraft
[ic,u0,~,lin,lin_red] = trimConfig2a(aircraft, V, pos_ref.alt);

% Joystick parameters
jystck = joystickLoadParams('joystick_params_Xbox_360',2,-1);

% Open Simulink model
open('sim_config2a.slx')


