% Validation of configuration 2a lift and drag distribution with propellers

clear;
addPathConfig2a();

% Validation angles of attack, in deg
AoA = [0,3]; % Available: 0, 0.7, 1, 3
AoA_offset = -0.4; % Offset of LLT compared to CFD

% Aircraft parameters
aircraft = aircraftSyntracCreate('NumPanelWing',56*2);

% Extract main wing
wing = aircraft.wing_main;


%% Define current wing and propeller state

% define current rigid body state
beta = deg2rad(0);
h = ft2m(22000);
atm = isAtmosphere(h);
V = atm.a*0.4349;
omega = [0;0;0];

% cg position in wing coordinate system
xyz_cg = [-15.27+14.38;0;0];

% define actuator states
actuators_pos = [ 0 0 ];
actuators_rate = [ 0 0 ];

% Propeller Delta pitches, in deg
pitches = [1,1,1,1,1,1,1,1,1,1]*-1;

%% Run propellers

% Propeller map
[T, Q, P, vi, v_slipstream, swirls, eff] = ...
    propeller_model_sfcn( aircraft.prop_table, [aircraft.prop.RPM], ...
        pitches, repmat(V,size(pitches)), atm.rho, [aircraft.prop.diameter]'/2 );

% Propeller slipstream
swirls_mod = 1.3 * swirls; % Increase swirl for better agreement with CFD
prop_pos = [aircraft.prop.pos_c];
prop_y_pos = prop_pos(2,:);
V_Wb = wingApplySlipstream( wing, prop_y_pos, [aircraft.prop.angle], ...
    [aircraft.prop.diameter], vi, swirls_mod );


%% Compute and plot wing lift and drag distribution for all specified AoA

figure(1)
hold on
grid on
box on
Legend = {};

figure(2)
hold on
grid on
box on

for i = 1:length(AoA)
    
    % Current angle of attack
    alpha = deg2rad( AoA(i) + AoA_offset );
    
    % Run LLT with propeller slipstream
    wing = wingSetState(wing, alpha, beta, V, omega, ...
        actuators_pos, actuators_rate, xyz_cg, 'atmosphere', isAtmosphere(h), ...
        'V_Wb', V_Wb, 'V_Wb_dt', zeros( 3, wing.n_panel ) );

    % Run LLT without propeller slipstream
    wing0 = wing;
    wing0 = wingSetState(wing0, alpha, beta, V, omega, ...
        actuators_pos, actuators_rate, xyz_cg, 'atmosphere', isAtmosphere(h), ...
        'V_Wb', V_Wb*0, 'V_Wb_dt', zeros( 3, wing.n_panel ) );

    % Force components in body frame
    X_i_b = wing.state.aero.force_loc.R_i_b(1,end/2+1:end);
    Z_i_b = wing.state.aero.force_loc.R_i_b(3,end/2+1:end);
    y = wing.geometry.ctrl_pt.pos(2,end/2+1:end);
    Delta_y = diff(wing.geometry.line_25.pos(2,end/2+0.5:end));

    % Load CFD results
    CFD = readTecplotDistribution(['distributions_AOA',num2str(AoA(i)),'.dat']);

    figure(1) % Plot lift distribution
    plot( CFD.y, CFD.dFx_dy*-sind(AoA(i))+CFD.dFz_dy*cosd(AoA(i)), '-' )
    plot( y, (-X_i_b*-sin(alpha)-Z_i_b*cos(alpha))./Delta_y, '-' )

    figure(2) % Plot drag distribution
    plot( CFD.y, CFD.dFx_dy*cosd(AoA(i))+CFD.dFz_dy*sind(AoA(i)), '-' )
    plot( y, (-X_i_b*cos(alpha)-Z_i_b*sin(alpha))./Delta_y, '-' )

    Legend{end+1} = ['CFD: ','$\alpha= $',num2str(AoA(i)),' deg'];
    Legend{end+1} = ['LLT: ','$\alpha= $',num2str(rad2deg(alpha)),' deg'];


end

Legend{end+1} = 'Pylon positions';

figure(1)
plot(prop_y_pos(end/2+1:end),zeros(size(prop_y_pos(end/2+1:end))),'ko')
xlabel('$y$ (m)','interpreter','latex')
ylabel('$L/y$ (N/m)','interpreter','latex')
legend(Legend{:},'interpreter','latex')

figure(2)
plot(prop_y_pos(end/2+1:end),zeros(size(prop_y_pos(end/2+1:end))),'ko')
xlabel('$y$ (m)','interpreter','latex')
ylabel('$D/y$ (N/m)','interpreter','latex')
legend(Legend{:},'interpreter','latex')

