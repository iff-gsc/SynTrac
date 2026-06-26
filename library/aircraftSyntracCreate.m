function aircraft = aircraftSyntracCreate( varargin )
% aircraftSyntracCreate returns a struct with all flight dynamics model
% parameters
% 
% Syntax:
%   aircraft = aircraftSyntracCreate()
%   aircraft = aircraftSyntracCreate( name, value )
% 
% Example 1:
%   aircraft = aircraftSyntracCreate()
% 
% Example 2:
%   aircraft = aircraftSyntracCreate( 'MassCase', 'OEM' )
% 
% Inputs:
%   Available name-value arguments:
%       - 'MassCase':   Desired mass case with options: 
%                           'MTOM'  (maximum takeoff mass, default)
%                           'OEM'   (operating empty mass)
%       - 'PropConfig': Desired propeller configuration with options:
%                           'Inner' (inner configuration - propellers are
%                                   located closer to fuselage, default)
%                           'Outer' (outer configuration - propellers are
%                                   located closer to wing tip)
%       - 'NumPanelWing':   Number of wing panels (default: 58)
%       - 'NumPanelHtp':    Number of horizontal tailplane panels (default:
%                           16)
%       - 'NumPanelVtp':    Number of vertical tailplane panels (default:
%                           8)
% 
% Outputs:
%   aircraft        Aircraft parameters struct (as defined by this function
%                   and as needed for the Simulink block "SynTrac
%                   Configuration 2a")
% 
% *************************************************************************

p = inputParser;

default_n_panel_wing = 58;
default_n_panel_htp = 16;
default_n_panel_vtp = 8;
% default_n_seg_fuse = 15;
default_mass_case = 'MTOM';
default_prop_config = 'Inner';

valid_panel_number = @(x) isnumeric(x) && isscalar(x) && (x>1) && (mod(x,1)==0);
valid_mass_case = @(x) ischar(x) && ismember(x, {'MTOM','OEM'});
valid_prop_config = @(x) ischar(x) && ismember(x, {'Inner','Outer'});

addParameter(p,'NumPanelWing',default_n_panel_wing,valid_panel_number);
addParameter(p,'NumPanelHtp',default_n_panel_htp,valid_panel_number);
addParameter(p,'NumPanelVtp',default_n_panel_vtp,valid_panel_number);
% addParameter(p,'NumSegFuse',default_n_seg_fuse,valid_panel_number);
addParameter(p,'MassCase',default_mass_case,valid_mass_case);
addParameter(p,'PropConfig',default_prop_config,valid_prop_config);

parse(p,varargin{:})

%% init rigid body
if strcmp(p.Results.MassCase, 'MTOM')
    body.m = 24533;
    body.I = diag( [ 217205, 612834, 705546 ]);
elseif strcmp(p.Results.MassCase, 'OEM')
    body.m = 15839;
    body.I = diag( [ 154459, 435801, 501731 ]);
end

%% configuration parameters
% center of gravity in body-fixed c frame (NED located at the nose)
config.xyz_cg_c = [ -14.79; 0; 0 ];

%% propellers
if strcmp(p.Results.PropConfig, 'Inner')
    prop(1).pos_c = [ 13.189;   -13.003;  1.793 ];
    prop(2).pos_c = [ 13.189;   -10.371;  1.793 ];
    prop(3).pos_c = [ 13.189;   -7.739;   1.793 ];
    prop(4).pos_c = [ 13.189;   -5.107;   1.793 ];
    prop(5).pos_c = [ 13.189;   -2.475;   1.793 ];
    prop(6).pos_c = [ 13.189;   +2.475;   1.793 ];
    prop(7).pos_c = [ 13.189;   +5.107;   1.793 ];
    prop(8).pos_c = [ 13.189;   +7.739;   1.793 ];
    prop(9).pos_c = [ 13.189;   +10.371;  1.793 ];
    prop(10).pos_c = [ 13.189;  +13.003;  1.793 ];
elseif strcmp(p.Results.PropConfig, 'Outer')
    prop(1).pos_c = [ 13.186;   -13.500;  1.833 ];
    prop(2).pos_c = [ 13.186;   -10.868;  1.833 ];
    prop(3).pos_c = [ 13.186;   -8.236;   1.833 ];
    prop(4).pos_c = [ 13.186;   -5.604;   1.833 ];
    prop(5).pos_c = [ 13.186;   -2.972;   1.833 ];
    prop(6).pos_c = [ 13.186;   +2.972;   1.833 ];
    prop(7).pos_c = [ 13.186;   +5.604;   1.833 ];
    prop(8).pos_c = [ 13.186;   +8.236;   1.833 ];
    prop(9).pos_c = [ 13.186;   +10.868;  1.833 ];
    prop(10).pos_c = [ 13.186;  +13.500;  1.833 ];
end

% From A03: The orientation is inward up.
% So, the propeller's part close to the fuselage is going up.
% If you look at the left half of the aircraft coming towards you,
% it is counter-clockwise. 
for i = 1:length(prop)
    prop(i).angle = deg2rad(-5);
    prop(i).diameter = 2.35;
    if i <= length(prop)/2
        prop(i).dir = -1;
    else
        prop(i).dir = 1;
    end
    prop(i).RPM = 1225;
end

prop_table = xprop_load_table('data/xprop_data.mat');

%% aerodynamics
wing_params = wingLoadParameters( 'wing_params_2a' );
eta_edges = [];
p_per_prop = 4;
for i = length(prop)/2+1:length(prop)
    r = prop(i).diameter/2;
    eta_edges(end+1:end+p_per_prop+1) = ( prop(i).pos_c(2) + ...
        linspace( -r, r, p_per_prop+1 ) ) ...
        / (0.5*wing_params.b);
end
eta_edges(eta_edges>1) = [];
eta_edges(eta_edges<0) = [];
wing_main = wingCreate( 'wing_params_2a', p.Results.NumPanelWing, ...
    'EtaEdges', eta_edges );
wing_htp = wingCreate( 'wing_params_htp2a', p.Results.NumPanelHtp );
wing_vtp = wingCreate( 'wing_params_vtp2a', p.Results.NumPanelVtp );

%% wing downwash
downwash = wingGetDownwashDerivs( wing_main, wing_htp );
derivs = wingGetDerivs(wing_main);
downwash.C_Lalpha = -derivs.alpha(3);
downwash.C_Leta = -derivs.eta(3,:);

%% actuator dynamics
act = loadParams('actuators_params_syntrac');

%% assign
aircraft.body = body;
aircraft.wing_main = wing_main;
aircraft.wing_htp = wing_htp;
aircraft.wing_vtp = wing_vtp;
% aircraft.fuselage = fuselage;
aircraft.downwash = downwash;
aircraft.prop = prop;
aircraft.prop_table = prop_table;
aircraft.actuators = act;
aircraft.config = config;
aircraft.vtp_sideslip_factor = 1.7;


end
