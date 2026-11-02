% Validation of configuration 2a lift and drag distribution with propellers

close all

clear;
addPathConfig2a();

% Boolean for writing mat2tikz plots
do_write = false;

% Validation angles of attack, in deg
AoA = [0.5]; % Available: 0, 0.7, 1, 3 | AOA3 has most available validation data
AoA_offset = 0; % (not anymore necessary ... Offset of LLT compared to CFD / -0.4 makes a good fit with CFD)

% Aircraft parameters
aircraft = aircraftSyntracCreate('NumPanelWing',102);

% Extract main wing
wing = aircraft.wing_main;


%% Define current wing and propeller state

% define current rigid body state
beta = deg2rad(0);
h = ft2m(22000);
atm = isAtmosphere(h);
%V = 137;
V = atm.a*0.4349;
omega = [0;0;0];

% cg position in wing coordinate system
xyz_cg = [-15.27+14.38;0;0];

% define actuator states
actuators_pos = [ 0 0 ];
actuators_rate = [ 0 0 ];

% Propeller Delta pitches, in deg

% for uniform thrust values
pitches = [1,1,1,1,1,1,1,1,1,1]*-1.5; % reference is 53.5°

% to match the non-uniform thrust values from CFD results (AOA=3°)
% pitches = [-1.73,-1.31,-1.03,-0.87,-0.64,-0.64,-0.87,-1.03,-1.31,-1.73]; 

%% Run propellers

% Propeller map
[Thrust, Q, P, vi, v_slipstream, vt, eff] = ...
    propeller_model_sfcn( aircraft.prop_table, [aircraft.prop.RPM], ...
        pitches, repmat(V,size(pitches)), atm.rho, [aircraft.prop.diameter]'/2 );

% Propeller slipstream
prop_pos = [aircraft.prop.pos_c];
prop_y_pos = prop_pos(2,:);
V_Wb = wingApplySlipstream( wing, prop_y_pos, [aircraft.prop.angle], ...
    [aircraft.prop.diameter], vi, vt, [aircraft.prop.dir], ...
    [aircraft.prop.k_vi], [aircraft.prop.k_vt] );
% For analyses with only prop 7 running (second to inner most prop on right
% side
vi_prop2 = zeros(size(vi));
vi_prop2(7) = vi(7);
vt_prop2 = zeros(size(vi));
vt_prop2(7) = vt(7);
V_Wb_prop2 = wingApplySlipstream( wing, prop_y_pos, [aircraft.prop.angle], ...
    [aircraft.prop.diameter], vi_prop2, vt_prop2, [aircraft.prop.dir], ...
    [aircraft.prop.k_vi], [aircraft.prop.k_vt]);

%% Compute and plot wing lift and drag distribution for all specified AoA

figure(1)
hold off
grid on
hold on
box on
Legend = {};

figure(2)
hold off
grid on
hold on
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

    % Run LLT only with 7th prop running
    wing1 = wing;
    wing1 = wingSetState(wing1, alpha, beta, V, omega, ...
        actuators_pos, actuators_rate, xyz_cg, 'atmosphere', isAtmosphere(h), ...
        'V_Wb', V_Wb_prop2, 'V_Wb_dt', zeros( 3, wing.n_panel ) );

    % Force components in body frame
    X_i_b = wing.state.aero.force_loc.R_i_b(1,end/2+1:end);
    Z_i_b = wing.state.aero.force_loc.R_i_b(3,end/2+1:end);
    y = wing.geometry.ctrl_pt.pos(2,end/2+1:end);
    for j = 1:length(y)
        index = 0;
        if y(j) > 1
            index = j-1;
            break
        else
            continue
        end
    end
    Delta_y = diff(wing.geometry.line_25.pos(2,end/2+0.5:end));
    
    [~,~, drags_wing, drags_i_wing] = wingGetDrag(wing);
    drags_wing       = drags_wing(end/2+1:end);
    drags_i_wing     = drags_i_wing(end/2+1:end);    

    X_i_b(1:index) = 0;
    Z_i_b(1:index) = 0;

    X_i_b_clean = wing0.state.aero.force_loc.R_i_b(1,end/2+1:end);
    Z_i_b_clean = wing0.state.aero.force_loc.R_i_b(3,end/2+1:end);
    y_clean = wing0.geometry.ctrl_pt.pos(2,end/2+1:end);
    Delta_y_clean = diff(wing0.geometry.line_25.pos(2,end/2+0.5:end));

    [~,~, drags_wing0, drags_i_wing0] = wingGetDrag(wing0);
    drags_wing0       = drags_wing0(end/2+1:end);
    drags_i_wing0     = drags_i_wing0(end/2+1:end);

    X_i_b_clean(1:index) = 0;
    Z_i_b_clean(1:index) = 0;

    X_i_b_one_prop = wing1.state.aero.force_loc.R_i_b(1,end/2+1:end);
    Z_i_b_one_prop = wing1.state.aero.force_loc.R_i_b(3,end/2+1:end);
    y_one_prop = wing1.geometry.ctrl_pt.pos(2,end/2+1:end);
    Delta_y_one_prop = diff(wing1.geometry.line_25.pos(2,end/2+0.5:end));

    X_i_b_one_prop(1:index) = 0;
    Z_i_b_one_prop(1:index) = 0;

    distr_drag_for_deco = drags_wing./Delta_y;
    distr_induced_drag = drags_i_wing./Delta_y;
    distr_visc_drag = drags_wing./Delta_y-drags_i_wing./Delta_y;

    figure(8)
    plot(y, distr_drag_for_deco, '-', DisplayName='FDM $D_{total}$/y')
    hold on 
    grid on
    plot(y, distr_induced_drag, '-', DisplayName='FDM $D_i$/y')
    plot(y, distr_visc_drag, '-', DisplayName='FDM $D_{visc}$/y')
    title('Drag Decomposition', 'Interpreter','latex')
    %title('Drag Decomposition (Cruise, $\alpha = 3^\circ$)', Interpreter='latex')
    Drag_Deco_table = readtable('Drag_Deco_Pitch52_AOA0.5.txt', 'Delimiter', ' ', ...
                  'MultipleDelimsAsOne', true);
    if AoA(i) == 0.5 
        plot(Drag_Deco_table.y, Drag_Deco_table.TotalDrag, DisplayName='CFD $D_{total}$/y')
        plot(Drag_Deco_table.y, Drag_Deco_table.skinFrictionDrag, DisplayName='CFD $D_f$/y')
        plot(Drag_Deco_table.y, Drag_Deco_table.TotalDrag-Drag_Deco_table.skinFrictionDrag, DisplayName='CFD $D_p$/y')
        %title('Drag Decomposition (Cruise, $\alpha = 0.5^\circ$)', Interpreter='latex')
    end
    xlim([y(index+1) inf])
    xlabel('Spanwise Position (m)', Interpreter='latex')
    ylabel('Drag Distribution $dD/dy$ (N/m)', Interpreter='latex')
    legend('Interpreter','latex')

    writeTikz('../../Exports/DragDeco.tex', do_write)
    
    Total_drag_for_deco_FDM = trapz(y(index:end), distr_drag_for_deco(index:end))
    Total_induced_drag_FDM = trapz(y(index:end), distr_induced_drag(index:end))
    Total_visc_drag_FDM = trapz(y(index:end), distr_visc_drag(index:end))

    Total_drag_for_deco_CFD = trapz(Drag_Deco_table.y, Drag_Deco_table.TotalDrag)
    Total_pressure_drag_CFD = trapz(Drag_Deco_table.y, Drag_Deco_table.TotalDrag-Drag_Deco_table.skinFrictionDrag)
    Total_skinfric_drag_CFD = trapz(Drag_Deco_table.y, Drag_Deco_table.skinFrictionDrag)
    

    
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
        Lift_total_all_props_CFD = cl_total_all_props_CFD*(0.5*atm.rho*V^2*55.76)
        cd_total_all_props_CFD = sum((CFD1.dFx_dy.*diff(CFD1.y(1:2))*cosd(AoA(i))+CFD1.dFz_dy.*diff(CFD1.y(1:2))*sind(AoA(i))))/(0.5*atm.rho*V^2*55.76);
        Drag_total_all_props_CFD = cd_total_all_props_CFD*(0.5*atm.rho*V^2*55.76)
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
        T_z = sum(Thrust)*sin(prop_angle);
        T_x = sum(Thrust)*cos(prop_angle);
    
        cl_total_all_props_LLT = (sum(-X_i_b*-sin(alpha)-Z_i_b*cos(alpha))+T_z)/(0.5*atm.rho*V^2*55.76);
        Lift_total_all_props_LLT = cl_total_all_props_LLT*(0.5*atm.rho*V^2*55.76)
        cd_total_all_props_LLT = sum(-X_i_b*cos(alpha)-Z_i_b*sin(alpha))/(0.5*atm.rho*V^2*55.76);
        Drag_total_all_props_LLT = cd_total_all_props_LLT*(0.5*atm.rho*V^2*55.76)
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
        plot( CFD5.y, CFD5.dFx_dy*-sind(AoA(i))+CFD5.dFz_dy*cosd(AoA(i)), '-', DisplayName=nameCFD5 )
        plot( y, (-X_i_b*-sin(alpha)-Z_i_b*cos(alpha))./Delta_y, '-', DisplayName=nameLLT1 )
        plot( y_clean, (-X_i_b_clean*-sin(alpha)-Z_i_b_clean*cos(alpha))./Delta_y_clean, '-', DisplayName=nameLLT2 )
        %plot( y_one_prop, (-X_i_b_one_prop*-sin(alpha)-Z_i_b_one_prop*cos(alpha))./Delta_y_one_prop, '-', DisplayName=nameLLT3 )
    
        figure(2) % Plot drag distribution
        plot( CFD1.y, CFD1.dFx_dy*cosd(AoA(i))+CFD1.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD1 )
        plot( CFD2.y, CFD2.dFx_dy*cosd(AoA(i))+CFD2.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD2 )
        plot( CFD3.y, CFD3.dFx_dy*cosd(AoA(i))+CFD3.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD3 )
        % plot( CFD4.y, CFD4.dFx_dy*cosd(AoA(i))+CFD4.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD4 )
        % plot( CFD5.y, CFD5.dFx_dy*cosd(AoA(i))+CFD5.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD5 )
        plot( y, (-X_i_b*cos(alpha)-Z_i_b*sin(alpha))./Delta_y, '-', DisplayName=nameLLT1 )
        plot( y_clean, (-X_i_b_clean*cos(alpha)-Z_i_b_clean*sin(alpha))./Delta_y_clean, '-', DisplayName=nameLLT2 )
        %plot( y_one_prop, (-X_i_b_one_prop*cos(alpha)-Z_i_b_one_prop*sin(alpha))./Delta_y_one_prop, '-', DisplayName=nameLLT3 )

        % Plot difference in lift between the all props on vs. off case
        figure(3)
        hold off
        plot( CFD1.y, (CFD1.dFx_dy*-sind(AoA(i))+CFD1.dFz_dy*cosd(AoA(i)))-(CFD2.dFx_dy*-sind(AoA(i))+CFD2.dFz_dy*cosd(AoA(i))), '-', DisplayName='CFD all props' )
        hold on
        plot( CFD3.y, (CFD3.dFx_dy*-sind(AoA(i))+CFD3.dFz_dy*cosd(AoA(i)))-(CFD2.dFx_dy*-sind(AoA(i))+CFD2.dFz_dy*cosd(AoA(i))), '-', DisplayName='CFD 2nd prop' )
        plot( y, ((-X_i_b*-sin(alpha)-Z_i_b*cos(alpha))./Delta_y)-((-X_i_b_clean*-sin(alpha)-Z_i_b_clean*cos(alpha))./Delta_y_clean), '-', DisplayName='FDM all props' )
        plot( y, ((-X_i_b_one_prop*-sin(alpha)-Z_i_b_one_prop*cos(alpha))./Delta_y)-((-X_i_b_clean*-sin(alpha)-Z_i_b_clean*cos(alpha))./Delta_y_clean), '-', DisplayName='FDM 2nd prop' )
        xlabel('Spanwise Position (m)',Interpreter='latex')
        ylabel('Delta Lift Distribution $\Delta dL/dy$ (N/m)',Interpreter='latex')
        legend('Interpreter','latex')
        grid on
        title({'Delta in lift-loading distribution','at AOA = 3deg (propellers on vs. off)'}, 'Interpreter','latex')

        writeTikz('../../Exports/DiffInLift.tex', do_write)

        % Plot difference in drag between the all props on vs. off case
        figure(4)
        hold off
        plot( CFD1.y, (CFD1.dFx_dy*cosd(AoA(i))+CFD1.dFz_dy*sind(AoA(i)))-(CFD2.dFx_dy*cosd(AoA(i))+CFD2.dFz_dy*sind(AoA(i))), '-', DisplayName='CFD all props' )
        hold on
        plot( CFD3.y, (CFD3.dFx_dy*cosd(AoA(i))+CFD3.dFz_dy*sind(AoA(i)))-(CFD2.dFx_dy*cosd(AoA(i))+CFD2.dFz_dy*sind(AoA(i))), '-', DisplayName='CFD 2nd prop' )        
        plot( y, ((-X_i_b*cos(alpha)-Z_i_b*sin(alpha))./Delta_y)-((-X_i_b_clean*cos(alpha)-Z_i_b_clean*sin(alpha))./Delta_y_clean), '-', DisplayName='FDM all props' )
        plot( y, ((-X_i_b_one_prop*cos(alpha)-Z_i_b_one_prop*sin(alpha))./Delta_y)-((-X_i_b_clean*cos(alpha)-Z_i_b_clean*sin(alpha))./Delta_y_clean), '-', DisplayName='FDM 2nd prop' )        
                xlabel('Spanwise Position (m)',Interpreter='latex')
        ylabel('Delta Drag Distribution $\Delta dD/dy$ (N/m)',Interpreter='latex')
        legend('Interpreter','latex')
        grid on
        title({'Delta in drag-loading distribution','at AOA = 3deg (propellers on vs. off)'}, 'Interpreter','latex')

        writeTikz('../../Exports/DiffInDrag.tex', do_write)

        % % Plot difference in Force Z between the all props on vs. off case
        % figure(5)
        % hold off
        % hold on
        % plot( CFD1.y, CFD1.dFz_dy-CFD2.dFz_dy)
        % plot( y,  (-Z_i_b-(-Z_i_b_clean))./Delta_y)
        % grid on
        % title('Difference in Force Z (propellers on minus off)')

    else
        % All other AOA cases than 3° (props-on available for all other AOA;
        % one further props-off case for AOA 0° currently not integrated here)
        CFD_path_1 = ['distributions_AOA',num2str(AoA(i)),'.dat'];
        CFD1 = readTecplotDistribution(CFD_path_1);
        
        prop_angle = aircraft.prop.angle;
        T_z = sum(Thrust)*sin(prop_angle);
        T_x = sum(Thrust)*cos(prop_angle);

        cl_total_all_props_CFD = sum((CFD1.dFx_dy.*diff(CFD1.y(1:2))*-sind(AoA(i))+CFD1.dFz_dy.*diff(CFD1.y(1:2))*cosd(AoA(i))))/(0.5*atm.rho*V^2*55.76);
        Lift_total_all_props_CFD = cl_total_all_props_CFD*(0.5*atm.rho*V^2*55.76)
        cd_total_all_props_CFD = sum((CFD1.dFx_dy.*diff(CFD1.y(1:2))*cosd(AoA(i))+CFD1.dFz_dy.*diff(CFD1.y(1:2))*sind(AoA(i))))/(0.5*atm.rho*V^2*55.76);
        Drag_total_all_props_CFD = cd_total_all_props_CFD*(0.5*atm.rho*V^2*55.76)

        cl_total_all_props_LLT = (sum(-X_i_b*-sin(alpha)-Z_i_b*cos(alpha))+T_z)/(0.5*atm.rho*V^2*55.76);
        Lift_total_all_props_LLT = cl_total_all_props_LLT*(0.5*atm.rho*V^2*55.76)
        cd_total_all_props_LLT = sum(-X_i_b*cos(alpha)-Z_i_b*sin(alpha))/(0.5*atm.rho*V^2*55.76);
        Drag_total_all_props_LLT = cd_total_all_props_LLT*(0.5*atm.rho*V^2*55.76)

        nameCFD = 'CFD'; %['CFD: ','$\alpha= $',num2str(AoA(i)),' deg'];
        nameLLT = 'FDM'; %['FDM: ','$\alpha= $',num2str(rad2deg(alpha)),' deg'];
        
        figure(1) % Plot lift distribution
        plot( CFD1.y, CFD1.dFx_dy*-sind(AoA(i))+CFD1.dFz_dy*cosd(AoA(i)), '-' , DisplayName=nameCFD)
        plot( y, (-X_i_b*-sin(alpha)-Z_i_b*cos(alpha))./Delta_y, '-', DisplayName=nameLLT )

        figure(2) % Plot drag distribution
        plot( CFD1.y, CFD1.dFx_dy*cosd(AoA(i))+CFD1.dFz_dy*sind(AoA(i)), '-', DisplayName=nameCFD )
        plot( y, (-X_i_b*cos(alpha)-Z_i_b*sin(alpha))./Delta_y, '-', DisplayName=nameLLT )

        % figure(3) % Plot Force X Distribution
        % plot( CFD1.y, CFD1.dFx_dy)
        % hold on
        % plot( y, -X_i_b./Delta_y)
        % 
        % figure(4) % Plot Force Z Distribution 
        % plot(CFD1.y, CFD1.dFz_dy)
        % hold on 
        % plot( y, -Z_i_b./Delta_y)


        
        Total_lift_wing = sum(-X_i_b(10:end)*-sin(alpha)-Z_i_b(10:end)*cos(alpha)); %+T_z;
        Total_drag_wing = sum(-X_i_b(10:end)*cos(alpha)-Z_i_b(10:end)*sin(alpha));

        Total_thrust = sum(Thrust);

    end

end

%finalize figure(1) and figure(2) and add pylon positions

figure(1)
plot(prop_y_pos(end/2+1:end),zeros(size(prop_y_pos(end/2+1:end))),'ko', DisplayName='Pylon positions')
title('Lift Distribution', 'Interpreter','latex')
xlabel('Spanwise Position $y$ (m)','interpreter','latex')
ylabel('Lift Distribution $dL/dy$ (N/m)','interpreter','latex')
xlim([1.07,inf])
legend(Legend{:}, ...
       'Interpreter', 'latex', ...
       'Location', 'south')
legend show
writeTikz('../../Exports/LiftLoadDistribution.tex', do_write)

figure(2)
plot(prop_y_pos(end/2+1:end),zeros(size(prop_y_pos(end/2+1:end))),'ko', DisplayName='Pylon positions')
title('Drag Distribution', 'Interpreter','latex')
xlabel('Spanwise Position $y$ (m)','interpreter','latex')
ylabel('Drag Distribution $dD/dy$ (N/m)','interpreter','latex')
xlim([1.07,inf])
% legend(Legend{:}, ...
%        'Interpreter', 'latex', ...
%        'Location', 'south')
%legend show
writeTikz('../../Exports/DragLoadDistribution.tex', do_write)
