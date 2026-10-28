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
%       - 'NumPanelWing':   Number of wing panels (default: 58), in case of
%                           more panels, you should only use 58+44 or 
%                           58+44+88+6 to assure that the number of panels
%                           behind all propellers are equal
%       - 'NumPanelHtp':    Number of horizontal tailplane panels (default:
%                           16)
%       - 'NumPanelVtp':    Number of vertical tailplane panels (default:
%                           8)
%       - 'Mach':           Mach number for computing the influence
%                           coefficients matrix (default: 0.4349)
%       - 'KSlipstream':    contraction factor of propeller slipstream 
%                           (default: 1)
%       - 'ApplyVi':        boolean that decides whether axial slipstream 
%                           component is applied to wing inflow
%                           (default: 1)
%       - 'ApplySwirl':    boolean that decides whether tangential 
%                           slipstream component is applied to wing inflow
%                           (default: 1)
%       - 'PropData':   XROTOR propeller data set with options:
%                           'Old' (data/xprop_data.mat, default)
%                           'New' (data/xprop_data_new.mat, updated
%                                 XROTOR run, filtered)
%       - 'PropExtrap': extrapolation of the propeller table beyond zero
%                       thrust (no data there) with options:
%                           'XROTOR' (shape of the unfiltered XROTOR run
%                                    incl. negative stall, default)
%                           'Linear' (linear over pitch, no stall,
%                                    data/xprop_data*_lin.mat)
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
default_k_slipstream = 1;
default_apply_vi = 1;
default_apply_swirl = 1;
default_Ma = 0.4349;
default_prop_data = 'Old';
default_prop_extrap = 'XROTOR';

valid_panel_number_wing = @(x) ismember(x, [58, 102, 196]);
valid_panel_number = @(x) isnumeric(x) && isscalar(x) && (x>1) && (mod(x,1)==0);
valid_mass_case = @(x) ischar(x) && ismember(x, {'MTOM','OEM'});
valid_prop_config = @(x) ischar(x) && ismember(x, {'Inner','Outer'});
valid_k_slipstream = @(x) isnumeric(x) && isscalar(x) && (x>=1) && (x<=2);
valid_apply_vi = @(x) isnumeric(x) && isscalar(x) && ((x==1) || (x==0));
valid_apply_swirl = @(x) isnumeric(x) && isscalar(x) && ((x==1) || (x==0));
valid_Ma = @(x) (x>=0) && (x<=0.8);
valid_prop_data = @(x) ischar(x) && ismember(x, {'Old','New'});
valid_prop_extrap = @(x) ischar(x) && ismember(x, {'XROTOR','Linear'});

addParameter(p,'NumPanelWing',default_n_panel_wing,valid_panel_number_wing);
addParameter(p,'NumPanelHtp',default_n_panel_htp,valid_panel_number);
addParameter(p,'NumPanelVtp',default_n_panel_vtp,valid_panel_number);
% addParameter(p,'NumSegFuse',default_n_seg_fuse,valid_panel_number);
addParameter(p,'MassCase',default_mass_case,valid_mass_case);
addParameter(p,'PropConfig',default_prop_config,valid_prop_config);
addParameter(p,'Kslipstream',default_k_slipstream,valid_k_slipstream);
addParameter(p,'ApplyVi',default_apply_vi,valid_apply_vi);
addParameter(p,'ApplySwirl',default_apply_swirl,valid_apply_swirl);
addParameter(p,'Mach',default_Ma,valid_Ma);
addParameter(p,'PropData',default_prop_data,valid_prop_data);
addParameter(p,'PropExtrap',default_prop_extrap,valid_prop_extrap);

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
    prop(1).pos_c = [ -13.189;   -13.003;  -1.793 ];
    prop(2).pos_c = [ -13.189;   -10.371;  -1.793 ];
    prop(3).pos_c = [ -13.189;   -7.739;   -1.793 ];
    prop(4).pos_c = [ -13.189;   -5.107;   -1.793 ];
    prop(5).pos_c = [ -13.189;   -2.475;   -1.793 ];
    prop(6).pos_c = [ -13.189;   +2.475;   -1.793 ];
    prop(7).pos_c = [ -13.189;   +5.107;   -1.793 ];
    prop(8).pos_c = [ -13.189;   +7.739;   -1.793 ];
    prop(9).pos_c = [ -13.189;   +10.371;  -1.793 ];
    prop(10).pos_c = [ -13.189;  +13.003;  -1.793 ];
elseif strcmp(p.Results.PropConfig, 'Outer')
    prop(1).pos_c = [ -13.186;   -13.500;  -1.833 ];
    prop(2).pos_c = [ -13.186;   -10.868;  -1.833 ];
    prop(3).pos_c = [ -13.186;   -8.236;   -1.833 ];
    prop(4).pos_c = [ -13.186;   -5.604;   -1.833 ];
    prop(5).pos_c = [ -13.186;   -2.972;   -1.833 ];
    prop(6).pos_c = [ -13.186;   +2.972;   -1.833 ];
    prop(7).pos_c = [ -13.186;   +5.604;   -1.833 ];
    prop(8).pos_c = [ -13.186;   +8.236;   -1.833 ];
    prop(9).pos_c = [ -13.186;   +10.868;  -1.833 ];
    prop(10).pos_c = [ -13.186;  +13.500;  -1.833 ];
end

% From A03: The orientation is inward up.
% So, the propeller's part close to the fuselage is going up.
% If you look at the left half of the aircraft coming towards you,
% it is counter-clockwise. 
for i = 1:length(prop)
    prop(i).angle = deg2rad(-5);
    prop(i).diameter = 2.35;
    prop(i).k = p.Results.Kslipstream;
    prop(i).apply_vi = p.Results.ApplyVi;
    prop(i).apply_swirl = p.Results.ApplySwirl;
    if i <= length(prop)/2
        prop(i).dir = -1;
    else
        prop(i).dir = 1;
    end
    prop(i).RPM = 1225;
end

switch p.Results.PropData
    case 'Old'
        prop_file = 'data/xprop_data';
    case 'New'
        prop_file = 'data/xprop_data_new';
end
if strcmp(p.Results.PropExtrap, 'Linear')
    prop_file = [prop_file '_lin'];
end
prop_table = xprop_load_table([prop_file '.mat']);

%% aerodynamics
wing_params = wingLoadParameters( 'wing_params_2a' );
eta_edges = [];
p_per_prop = 4;
for i = length(prop)/2+1:length(prop)
    r = prop(i).diameter/2; 
    r_k = 1/sqrt(prop(i).k)*r;
    eta_edges(end+1:end+p_per_prop+1) = ( prop(i).pos_c(2) + ...
        linspace( -r_k, r_k, p_per_prop+1 ) ) ...
        / (0.5*wing_params.b);
end
eta_edges(eta_edges>1) = [];
eta_edges(eta_edges<0) = [];
wing_main = wingCreate( 'wing_params_2a', p.Results.NumPanelWing, ...
    'EtaEdges', eta_edges, 'Mach', p.Results.Mach );
wing_htp = wingCreate( 'wing_params_htp2a', p.Results.NumPanelHtp, ...
    'Mach', p.Results.Mach );
wing_vtp = wingCreate( 'wing_params_vtp2a', p.Results.NumPanelVtp, ...
    'Mach', p.Results.Mach );

%% wing downwash
downwash = wingGetDownwashDerivs( wing_main, wing_htp );
derivs = wingGetDerivs(wing_main);
downwash.C_Lalpha = -derivs.alpha(3);
downwash.C_Leta = -derivs.eta(3,:);

%% Fuselage (3450N at V=136.33m/s, rho=0.61, Ma=0.435)
% Fuselage frontal area (m^2)
fuselage.S = pi*1.5^2;
% Drag coefficient at Ma=0
fuselage.C_D = 0.0775;

%% actuator dynamics
act = loadParams('actuators_params_syntrac');

%% assign
aircraft.body = body;
aircraft.wing_main = wing_main;
aircraft.wing_htp = wing_htp;
aircraft.wing_vtp = wing_vtp;
aircraft.fuselage = fuselage;
aircraft.downwash = downwash;
aircraft.prop = prop;
aircraft.prop_table = prop_table;
aircraft.actuators = act;
aircraft.config = config;
aircraft.vtp_sideslip_factor = 1.7;

end
