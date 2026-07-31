% Validation of configuration 2a lift and drag distribution with propellers

close all

clear;
addPathConfig2a();

% Validation angles of attack, in deg
AoA = [3]; % Available: 0, 0.7, 1, 3 | AOA3 has most available validation data
AoA_offset = -0.4; % Offset of LLT compared to CFD

% Aircraft parameters
aircraft = aircraftSyntracCreate('NumPanelWing',56*4, 'Kslipstream', 1);

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

% for uniform thrust values
% pitches = [1,1,1,1,1,1,1,1,1,1]*-1;

% to match the non-uniform thrust values from CFD results (AOA=3°)
pitches = [-1.73,-1.31,-1.03,-0.87,-0.64,-0.64,-0.87,-1.03,-1.31,-1.73]; 

%% Run propellers

% Propeller map
[T, Q, P, vi, v_slipstream, swirls, eff] = ...
    propeller_model_sfcn( aircraft.prop_table, [aircraft.prop.RPM], ...
        pitches, repmat(V,size(pitches)), atm.rho, [aircraft.prop.diameter]'/2 );

% Propeller slipstream
prop_pos = [aircraft.prop.pos_c];
prop_y_pos = prop_pos(2,:);
V_Wb = wingApplySlipstream( wing, prop_y_pos, [aircraft.prop.angle], ...
    [aircraft.prop.diameter], vi, [aircraft.prop.dir]'.*swirls, [aircraft.prop.k] );


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

% Designed as loop to iterate through all given AoA in list.
% However only for AoA=3° case there is extensive validation data provided. 
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

    % Run LLT only with 2nd and 9th prop running (only works with num-panel
    % == 112)
    wing1 = wing;
    V_Wb_2nd_9th_running = V_Wb;
    V_Wb_2nd_9th_running(:, [1:31, 40:73, 82:end]) = 0;
    wing1 = wingSetState(wing1, alpha, beta, V, omega, ...
        actuators_pos, actuators_rate, xyz_cg, 'atmosphere', isAtmosphere(h), ...
        'V_Wb', V_Wb_2nd_9th_running, 'V_Wb_dt', zeros( 3, wing.n_panel ) );

    % Force components in body frame
    X_i_b = wing.state.aero.force_loc.R_i_b(1,end/2+1:end);
    Z_i_b = wing.state.aero.force_loc.R_i_b(3,end/2+1:end);
    y = wing.geometry.ctrl_pt.pos(2,end/2+1:end);
    Delta_y = diff(wing.geometry.line_25.pos(2,end/2+0.5:end));

    X_i_b_clean = wing0.state.aero.force_loc.R_i_b(1,end/2+1:end);
    Z_i_b_clean = wing0.state.aero.force_loc.R_i_b(3,end/2+1:end);
    y_clean = wing0.geometry.ctrl_pt.pos(2,end/2+1:end);
    Delta_y_clean = diff(wing0.geometry.line_25.pos(2,end/2+0.5:end));

    X_i_b_one_prop = wing1.state.aero.force_loc.R_i_b(1,end/2+1:end);
    Z_i_b_one_prop = wing1.state.aero.force_loc.R_i_b(3,end/2+1:end);
    y_one_prop = wing1.geometry.ctrl_pt.pos(2,end/2+1:end);
    Delta_y_one_prop = diff(wing1.geometry.line_25.pos(2,end/2+0.5:end));
    
    if AoA(i) == 3
        % Allocate CFD paths
        CFD_path_1 = ['distributions_AOA',num2str(AoA(i)),'.dat'];
        CFD_path_2 = ['distributions_AOA',num2str(AoA(i)),'_clean.dat'];
        CFD_path_3 = ['distributions_AOA',num2str(AoA(i)),'_2nd_on.dat'];
        CFD_path_4 = ['distributions_AOA',num2str(AoA(i)),'_3rd_on.dat'];
        CFD_path_5 = ['distributions_AOA',num2str(AoA(i)),'_2nd+3rd_on.dat'];    

        % Read out CFD paths 
        CFD1 = readTecplotDistribution(CFD_path_1);
        CFD2 = readTecplotDistribution(CFD_path_2);
        CFD3 = readTecplotDistribution(CFD_path_3);
        CFD4 = readTecplotDistribution(CFD_path_4);
        CFD5 = readTecplotDistribution(CFD_path_5);

        cl_total_all_props_CFD = sum((CFD1.dFx_dy.*diff(CFD1.y(1:2))*-sind(AoA(i))+CFD1.dFz_dy.*diff(CFD1.y(1:2))*cosd(AoA(i))))/(0.5*atm.rho*V^2*55.76);
        cd_total_all_props_CFD = sum((CFD1.dFx_dy.*diff(CFD1.y(1:2))*cosd(AoA(i))+CFD1.dFz_dy.*diff(CFD1.y(1:2))*sind(AoA(i))))/(0.5*atm.rho*V^2*55.76);
        glide_ratio_all_props_CFD = cl_total_all_props_CFD/cd_total_all_props_CFD;
        case1CFD = [cl_total_all_props_CFD,cd_total_all_props_CFD,glide_ratio_all_props_CFD];
        cl_total_clean_CFD = sum((CFD2.dFx_dy.*diff(CFD2.y(1:2))*-sind(AoA(i))+CFD2.dFz_dy.*diff(CFD2.y(1:2))*cosd(AoA(i))))/(0.5*atm.rho*V^2*55.76);
        cd_total_clean_CFD = sum((CFD2.dFx_dy.*diff(CFD2.y(1:2))*cosd(AoA(i))+CFD2.dFz_dy.*diff(CFD2.y(1:2))*sind(AoA(i))))/(0.5*atm.rho*V^2*55.76);
        glide_ratio_clean_CFD = cl_total_clean_CFD/cd_total_clean_CFD;
        case2CFD = [cl_total_clean_CFD,cd_total_clean_CFD,glide_ratio_clean_CFD]; 
        cl_total_one_prop_CFD = sum((CFD3.dFx_dy.*diff(CFD3.y(1:2))*-sind(AoA(i))+CFD3.dFz_dy.*diff(CFD3.y(1:2))*cosd(AoA(i))))/(0.5*atm.rho*V^2*55.76);
        cd_total_one_prop_CFD = sum((CFD2.dFx_dy.*diff(CFD3.y(1:2))*cosd(AoA(i))+CFD3.dFz_dy.*diff(CFD3.y(1:2))*sind(AoA(i))))/(0.5*atm.rho*V^2*55.76);
        glide_ratio_one_prop_CFD = cl_total_one_prop_CFD/cd_total_one_prop_CFD;
        case3CFD = [cl_total_one_prop_CFD,cd_total_one_prop_CFD,glide_ratio_one_prop_CFD];

        prop_angle = aircraft.prop.angle;
        T_z = sum(T)*sin(prop_angle);
        T_x = sum(T)*cos(prop_angle);
    
        cl_total_all_props_LLT = (sum(-X_i_b*-sin(alpha)-Z_i_b*cos(alpha))+T_z)/(0.5*atm.rho*V^2*55.76);
        cd_total_all_props_LLT = sum(-X_i_b*cos(alpha)-Z_i_b*sin(alpha))/(0.5*atm.rho*V^2*55.76);
        glide_ratio_all_props_LLT = cl_total_all_props_LLT/cd_total_all_props_LLT;
        case1LLT = [cl_total_all_props_LLT,cd_total_all_props_LLT,glide_ratio_all_props_LLT];
        cl_total_clean_LLT = (sum(-X_i_b_clean*-sin(alpha)-Z_i_b_clean*cos(alpha))+T_z*0)/(0.5*atm.rho*V^2*55.76);
        cd_total_clean_LLT = (sum(-X_i_b_clean*cos(alpha)-Z_i_b_clean*sin(alpha)))/(0.5*atm.rho*V^2*55.76);
        glide_ratio_clean_LLT = cl_total_clean_LLT/cd_total_clean_LLT;
        case2LLT = [cl_total_clean_LLT,cd_total_clean_LLT,glide_ratio_clean_LLT];
        cl_total_one_prop_LLT = (sum(-X_i_b_one_prop*-sin(alpha)-Z_i_b_one_prop*cos(alpha))+T_z*0.2)/(0.5*atm.rho*V^2*55.76);
        cd_total_one_prop_LLT = (sum(-X_i_b_one_prop*cos(alpha)-Z_i_b_one_prop*sin(alpha)))/(0.5*atm.rho*V^2*55.76);
        glide_ratio_one_prop_LLT = cl_total_one_prop_LLT/cd_total_one_prop_LLT;
        case3LLT = [cl_total_one_prop_LLT,cd_total_one_prop_LLT,glide_ratio_one_prop_LLT];

        cases = [case1CFD,case2CFD,case3CFD,case1LLT,case2LLT,case3LLT];
        % add table for coefficients and glide ratio overview
        T = table(...
            {'All props on CFD';'All props off CFD';'One prop on CFD';'All props on LLT';'All props off LLT';'One prop on LLT'}, ...
            [case1CFD(1); case2CFD(1); case3CFD(1); case1LLT(1); case2LLT(1); case3LLT(1)], ... 
            [case1CFD(2); case2CFD(2); case3CFD(2); case1LLT(2); case2LLT(2); case3LLT(2)], ...
            [case1CFD(3); case2CFD(3); case3CFD(3); case1LLT(3); case2LLT(3); case3LLT(3)], ...
            'VariableNames', {'Case name', 'CL', 'CD', 'Glide ratio'});
        disp(' ')
        disp('Coefficients and glide ratios for several cases with an 3° AOA')
        disp(T)

        %legend names
        nameCFD1 = ['CFD: ','$\alpha= $',num2str(AoA(i)),' deg + all props on'];
        nameCFD2 = ['CFD: ','$\alpha= $',num2str(AoA(i)),' deg + all props off'];
        nameCFD3 = ['CFD: ','$\alpha= $',num2str(AoA(i)),' deg + 2nd prop on'];
        nameCFD4 = ['CFD: ','$\alpha= $',num2str(AoA(i)),' deg + 3rd prop on'];
        nameCFD5 = ['CFD: ','$\alpha= $',num2str(AoA(i)),' deg + 2nd and 3rd prop on'];
        nameLLT1 = ['LLT: ','$\alpha= $',num2str(rad2deg(alpha)),' deg + all props on'];
        nameLLT2 = ['LLT: ','$\alpha= $',num2str(rad2deg(alpha)),' deg + all props off'];
        nameLLT3 = ['LLT: ','$\alpha= $',num2str(rad2deg(alpha)),' deg + 2nd prop on'];

        figure(1) % Plot lift distribution
        plot( CFD1.y, CFD1.dFx_dy*-sind(AoA(i))+CFD1.dFz_dy*cosd(AoA(i)), '-', DisplayName=nameCFD1 )
        plot( CFD2.y, CFD2.dFx_dy*-sind(AoA(i))+CFD2.dFz_dy*cosd(AoA(i)), '-', DisplayName=nameCFD2 )
        % plot( CFD3.y, CFD3.dFx_dy*-sind(AoA(i))+CFD3.dFz_dy*cosd(AoA(i)), '-', DisplayName=nameCFD3 )
        % plot( CFD4.y, CFD4.dFx_dy*-sind(AoA(i))+CFD4.dFz_dy*cosd(AoA(i)), '-', DisplayName=nameCFD4 )
        % plot( CFD5.y, CFD5.dFx_dy*-sind(AoA(i))+CFD5.dFz_dy*cosd(AoA(i)), '-', DisplayName=nameCFD5 )
        plot( y, (-X_i_b*-sin(alpha)-Z_i_b*cos(alpha))./Delta_y, '-', DisplayName=nameLLT1 )
        plot( y_clean, (-X_i_b_clean*-sin(alpha)-Z_i_b_clean*cos(alpha))./Delta_y_clean, '-', DisplayName=nameLLT2 )
        %plot( y_one_prop, (-X_i_b_one_prop*-sin(alpha)-Z_i_b_one_prop*cos(alpha))./Delta_y_one_prop, '-', DisplayName=nameLLT3 )
    
        figure(2) % Plot drag distribution
        plot( CFD1.y, CFD1.dFx_dy*cosd(AoA(i))+CFD1.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD1 )
        plot( CFD2.y, CFD2.dFx_dy*cosd(AoA(i))+CFD2.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD2 )
        % plot( CFD3.y, CFD3.dFx_dy*cosd(AoA(i))+CFD3.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD3 )
        % plot( CFD4.y, CFD4.dFx_dy*cosd(AoA(i))+CFD4.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD4 )
        % plot( CFD5.y, CFD5.dFx_dy*cosd(AoA(i))+CFD5.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD5 )
        plot( y, (-X_i_b*cos(alpha)-Z_i_b*sin(alpha))./Delta_y, '-', DisplayName=nameLLT1 )
        plot( y_clean, (-X_i_b_clean*cos(alpha)-Z_i_b_clean*sin(alpha))./Delta_y_clean, '-', DisplayName=nameLLT2 )
        %plot( y_one_prop, (-X_i_b_one_prop*cos(alpha)-Z_i_b_one_prop*sin(alpha))./Delta_y_one_prop, '-', DisplayName=nameLLT3 )

        % Plot difference in lift between the all props on vs. off case
        figure(3)
        plot( CFD1.y, (CFD1.dFx_dy*-sind(AoA(i))+CFD1.dFz_dy*cosd(AoA(i)))-(CFD2.dFx_dy*-sind(AoA(i))+CFD2.dFz_dy*cosd(AoA(i))), '-', DisplayName='Differnce in Lift from CFD' )
        hold on
        plot( y, ((-X_i_b*-sin(alpha)-Z_i_b*cos(alpha))./Delta_y)-((-X_i_b_clean*-sin(alpha)-Z_i_b_clean*cos(alpha))./Delta_y_clean), '-', DisplayName='Differnce in Lift from LLT' )
        legend show
        title('Difference in lift (propellers on minus off)')

        % Plot difference in drag between the all props on vs. off case
        figure(4)
        plot( CFD1.y, (CFD1.dFx_dy*cosd(AoA(i))+CFD1.dFz_dy*sind(AoA(i)))-(CFD2.dFx_dy*cosd(AoA(i))+CFD2.dFz_dy*sind(AoA(i))), '-', DisplayName='Differnce in Drag from CFD' )
        hold on
        plot( y, ((-X_i_b*cos(alpha)-Z_i_b*sin(alpha))./Delta_y)-((-X_i_b_clean*cos(alpha)-Z_i_b_clean*sin(alpha))./Delta_y_clean), '-', DisplayName='Differnce in Drag from LLT' )
        legend show
        title('Difference in drag (propellers on minus off)')

    else
        % All other AOA cases than 3° (props-on available for all other AOA;
        % one further props-off case for AOA 0° currently not integrated here)
        CFD_path_1 = ['distributions_AOA',num2str(AoA(i)),'.dat'];
        CFD1 = readTecplotDistribution(CFD_path_1);

        nameCFD = ['CFD: ','$\alpha= $',num2str(AoA(i)),' deg'];
        nameLLT = ['LLT: ','$\alpha= $',num2str(rad2deg(alpha)),' deg'];
        
        figure(1) % Plot lift distribution
        plot( CFD1.y, CFD1.dFx_dy*-sind(AoA(i))+CFD1.dFz_dy*cosd(AoA(i)), '-' , DisplayName=nameCFD)
        plot( y, (-X_i_b*-sin(alpha)-Z_i_b*cos(alpha))./Delta_y, '-', DisplayName=nameLLT )

        figure(2) % Plot drag distribution
        plot( CFD1.y, CFD1.dFx_dy*cosd(AoA(i))+CFD1.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD )
        plot( y, (-X_i_b*cos(alpha)-Z_i_b*sin(alpha))./Delta_y, '-', DisplayName=nameLLT )

    end

end

%finalize figure(1) and figure(2) and add pylon positions

figure(1)
plot(prop_y_pos(end/2+1:end),zeros(size(prop_y_pos(end/2+1:end))),'ko', DisplayName='Pylon positions')
xlabel('$y$ (m)','interpreter','latex')
ylabel('$L/y$ (N/m)','interpreter','latex')
legend(Legend{:}, ...
       'Interpreter', 'latex', ...
       'Location', 'south')
legend show

figure(2)
plot(prop_y_pos(end/2+1:end),zeros(size(prop_y_pos(end/2+1:end))),'ko', DisplayName='Pylon positions')
xlabel('$y$ (m)','interpreter','latex')
ylabel('$D/y$ (N/m)','interpreter','latex')
legend(Legend{:}, ...
       'Interpreter', 'latex', ...
       'Location', 'south')
legend show
