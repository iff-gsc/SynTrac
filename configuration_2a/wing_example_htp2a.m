% Example script

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% Create vortex lattice method wing
htp = wingCreate('wing_params_htp2a',16);

%% define current wing state

% define current rigid body state
alpha = deg2rad(2);
beta = deg2rad(0);
h = ft2m(22000);
atm = isAtmosphere(h);
V = atm.a*0.44;
omega = [0;0;0];

% cg position in wing coordinate system
% xyz_cg = zeros(3,1);
xyz_cg = [-28+28.499;0;0];

% define actuator states
actuators_pos = [ 0 0 ];
actuators_rate = [ 0 0 ];

%% compute wing state

htp = wingSetState(htp, alpha, beta, V, omega, ...
    actuators_pos, actuators_rate, xyz_cg, 'atmosphere', isAtmosphere(h), ...
    'V_Wb', zeros( 3, htp.n_panel ), 'V_Wb_dt', zeros( 3, htp.n_panel ) );


%% plot results
figure
wingPlotGeometry(htp,3)

figure
y = -htp.state.aero.coeff_loc.c_XYZ_b(3,:);
plot( htp.geometry.ctrl_pt.pos(2,:)/htp.params.b*2, y );
xlabel('dimensionless span')
ylim([min(0,min(y)) inf])
ylabel('Lift coefficient')
grid on