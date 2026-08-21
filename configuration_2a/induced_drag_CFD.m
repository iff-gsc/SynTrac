% This script computes the CFD induced drag with lifting line logic

% Set path
addPathConfig2a();

% Angle of attack (deg)
AoA = 3; % Available: 0, 0.7, 1, 3 | AoA3 has most available validation data

% Load CFD data
CFD_path_DP = ['distributions_AOA',num2str(AoA),'.dat'];
CFD_path_clean = ['distributions_AOA',num2str(AoA),'_clean.dat'];
CFD_DP = readTecplotDistribution(CFD_path_DP);
CFD_clean = readTecplotDistribution(CFD_path_clean);

% Define thrust (for induced velocity computation)
T = [1524.5, 1504.0, 1492.5, 1463.5, 1417.3];
T = [flip(T),T];

% Define operating point
h = ft2m(22000);
atm = isAtmosphere(h);
Ma = 0.4349;
V = atm.a*Ma;

% Propeller induced velocity
A = pi/4*2.35^2;
V_prop = -V/2 + sqrt((V/2)^2 + T / (2*atm.rho*A));

q = atm.rho/2*V*V;
q_prop = atm.rho/2*(V+V_prop).*(V+V_prop);

% Local chord
wing = wingCreate( 'wing_params_2a', 40 );
c_i = interp1(wing.params.eta_segments_wing*wing.params.b/2,wing.params.c,CFD_DP.y,'linear','extrap');



% Plot lift distribution
figure
hold on
grid on; box on;
plot( CFD_DP.y, CFD_clean.dL_dy )
plot( CFD_DP.y, CFD_DP.dL_dy )
xlabel('y (m)')
ylabel('L/y (N/m)')
legend('Clean','Propellers on')


% Compute lift coefficient distribution
CFD_clean.c_L = CFD_clean.dL_dy./c_i/q;
% SIMPLIFICATION: Assume that the whole wing sees the same increased q_prop
CFD_DP.c_L = CFD_DP.dL_dy./c_i/mean(q_prop);

% Plot lift coefficient distribution
figure
hold on
grid on; box on;
plot( CFD_DP.y, CFD_clean.c_L )
plot( CFD_DP.y, CFD_DP.c_L )
xlabel('y (m)')
ylabel('c_L')
legend('Clean','Propellers on')


% Compute effective angle of attack distribution
c_L_alpha = 6.9964 * 1./sqrt(1-Ma^2);
CFD_clean.alpha_eff = CFD_clean.c_L/c_L_alpha * 180/pi;
CFD_DP.alpha_eff = CFD_DP.c_L/c_L_alpha * 180/pi;

% Plot effective angle of attack distribution
figure
hold on
grid on; box on;
plot( CFD_DP.y, CFD_clean.alpha_eff )
plot( CFD_DP.y, CFD_DP.alpha_eff )
xlabel('y (m)')
ylabel('\alpha_{eff} (deg)')
legend('Clean','Propellers on')


% Plot difference in effective angle of attack distribution
figure
hold on
grid on; box on;
Delta_alpha_eff = CFD_DP.alpha_eff - CFD_clean.alpha_eff;
plot( CFD_DP.y, Delta_alpha_eff )
xlabel('y (m)')
ylabel('\Delta \alpha_{eff} (deg)')


% Compute induced drag distribution
CFD_DP.Delta_dDi_dy = -CFD_DP.dL_dy.*sind(Delta_alpha_eff);

% Plot induced angle of attack distriubtion
figure
hold on
grid on; box on;
plot( CFD_DP.y, CFD_DP.dD_dy - CFD_clean.dD_dy )
plot( CFD_DP.y, CFD_DP.Delta_dDi_dy )
xlabel('y (m)')
ylabel('\Delta D/y (N/m)')
legend('Total','Induced (like LL)')




%% Run lifting line model

% Aircraft parameters
aircraft = aircraftSyntracCreate('NumPanelWing',56*4, 'Kslipstream', 1);

% Extract main wing
wing = aircraft.wing_main;

AoA_offset = -0.4; % Offset of LLT compared to CFD
swirl_factor = 1.4; % Correction factor for swirl

% to match the non-uniform thrust values from CFD results (AOA=3°)
pitches = [-1.73,-1.31,-1.03,-0.87,-0.64,-0.64,-0.87,-1.03,-1.31,-1.73]; 

% Run propellers

% Propeller map
[T, Q, P, vi, v_slipstream, swirls, eff] = ...
    propeller_model_sfcn( aircraft.prop_table, [aircraft.prop.RPM], ...
        pitches, repmat(V,size(pitches)), atm.rho, [aircraft.prop.diameter]'/2 );

% Propeller slipstream
prop_pos = [aircraft.prop.pos_c];
prop_y_pos = prop_pos(2,:);
V_Wb = wingApplySlipstream( wing, prop_y_pos, [aircraft.prop.angle], ...
    [aircraft.prop.diameter], vi, swirl_factor*[aircraft.prop.dir]'.*swirls, ...
    [aircraft.prop.k] );


% Current angle of attack
alpha = deg2rad( AoA + AoA_offset );

% Run LLT with propeller slipstream
wing = wingSetState(wing, alpha, 0, V, zeros(3,1), ...
    [0,0], [0,0], zeros(3,1), 'atmosphere', isAtmosphere(h), ...
    'V_Wb', V_Wb, 'V_Wb_dt', zeros( 3, wing.n_panel ) );

% Run LLT without propeller slipstream
wing0 = wing;
wing0 = wingSetState(wing0, alpha, beta, V, omega, ...
    actuators_pos, actuators_rate, xyz_cg, 'atmosphere', isAtmosphere(h), ...
    'V_Wb', V_Wb*0, 'V_Wb_dt', zeros( 3, wing.n_panel ) );


[ ~, ~, drag, drag_i ] = wingGetDrag(wing);
[ ~, ~, drag0, drag0_i ] = wingGetDrag(wing0);

Delta_drag = drag_i - drag0_i;
y = wing.geometry.ctrl_pt.pos(2,end/2+1:end);
Delta_y = diff(wing.geometry.vortex.pos(2,:,1));
Delta_y = Delta_y(end/2+1:end);


% Plot lifting line difference in induced drag distribution
plot( y, Delta_drag(end/2+1:end)./Delta_y, 'DisplayName', 'Lifting line' )

