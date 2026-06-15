function aircraft = aircraftSyntracCreate( varargin )

p = inputParser;

default_n_panel_wing = 40;
default_n_panel_htp = 16;
default_n_panel_vtp = 8;
% default_n_seg_fuse = 15;
default_mass_case = 'MTOM';

valid_panel_number = @(x) isnumeric(x) && isscalar(x) && (x>1) && (mod(x,1)==0);
valid_mass_case = @(x) ischar(x) && ismember(x, {'MTOM','OEM'});

addParameter(p,'NumPanelWing',default_n_panel_wing,valid_panel_number);
addParameter(p,'NumPanelHtp',default_n_panel_htp,valid_panel_number);
addParameter(p,'NumPanelVtp',default_n_panel_vtp,valid_panel_number);
% addParameter(p,'NumSegFuse',default_n_seg_fuse,valid_panel_number);
addParameter(p,'MassCase',default_mass_case,valid_mass_case)

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

%% aerodynamics
wing_main = wingCreate( 'wing_params_2a', p.Results.NumPanelWing );
wing_htp = wingCreate( 'wing_params_htp2a', p.Results.NumPanelHtp );
wing_vtp = wingCreate( 'wing_params_vtp2a', p.Results.NumPanelVtp );

%% wing downwash
downwash = wingGetDownwashDerivs( wing_main, wing_htp );
derivs = wingGetDerivs(wing_main);
downwash.C_Lalpha = -derivs.alpha(3);
downwash.C_Leta = -derivs.eta(3,:);

%% propellers
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
