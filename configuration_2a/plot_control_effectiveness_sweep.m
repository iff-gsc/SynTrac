% Script to plot control effectiveness sweep for velocity, altitude, or
% load factor
% First time running may take some time because the Simulink model has to
% be re-compiled when the Mach number changes (but only if that Mach number
% has not been used before).

clear;
addPathConfig2a();

%% Define sweep

% Chose variable to change (V: Velocity, h: Altitude, n: Load factor)
x_name = 'V'; % 'V', 'h', 'n'

% Define velocity sweep (m/s)
V_vec = [92,110:30:170];

% Define load factor sweep (-)
n_vec = [0,1,2];

% Define altiutde sweep (ft)
h_vec = ft2m([22000,18000,12000]);

% Define default values
V = 136.33;       % Velocity (m/s)
h = ft2m(22000);  % Altitude (m)
n = 1;            % Load factor

% Zero-lift angle of attack (deg)
alpha_0 = -6.795;

% Set flag to export figures to Tikz
write_tikz = true;                             


%% Run sweep computation

clear lin_r;
clear alpha;
clear rho;
switch x_name
    case 'V'
        x_unit = 'm/s';
        x_var = V_vec;
        atm = isAtmosphere(h);
        for i = 1:length(V_vec)
            Ma = V_vec(i)/atm.a;
            aircraft = aircraftSyntracCreate('Mach',Ma);
            [ic,u0,~,lin,lin_r(i)] = trimConfig2a(aircraft, V_vec(i), h);
            alpha(i) = rad2deg(aeroAngles(ic.V_Kb));
            rho(i) = atm.rho;
        end
    case 'h'
        x_unit = 'ft';
        x_var = round(m2ft(h_vec));
        for i = 1:length(h_vec)
            atm = isAtmosphere(h_vec(i));
            Ma = V_vec(i)/atm.a;
            aircraft = aircraftSyntracCreate('Mach',Ma);
            [ic,u0,~,lin,lin_r(i)] = trimConfig2a(aircraft, V, h_vec(i));
            alpha(i) = rad2deg(aeroAngles(ic.V_Kb));
            rho(i) = atm.rho;
        end
    case 'n'
        x_unit = '';
        x_var = n_vec;
        aircraft = aircraftSyntracCreate();
        atm = isAtmosphere(h);
        for i = 1:length(n_vec)
            [ic,u0,~,lin,lin_r(i)] = trimConfig2a(aircraft, V, h, n_vec(i));
            alpha(i) = rad2deg(aeroAngles(ic.V_Kb));
            rho(i) = atm.rho;
        end
end

Dalpha = alpha - alpha_0;

%% Plot results

du_dtheta = 1/30;

Lgd = {};
figure
hold on
for i = 1:length(lin_r)
    Lgd_var = ['$',x_name,'=\,',num2str(x_var(i)),'\,$',x_unit];
    Lgd_alpha = ['$\alpha - \alpha_0=\,$',num2str(round(Dalpha(i),1)),'$^\circ$'];
    Lgd_rho = ['$\rho=\,$',num2str(round(rho(i),2)),'\,kg/m$^3$'];
    switch x_name
        case 'V'
            Lgd{i} = [Lgd_var, '\,\,(',Lgd_alpha,')'];
        case 'h'
            Lgd{i} = [Lgd_var, '\,\,(',Lgd_rho,',\,\,',Lgd_alpha,')'];
        case 'n'
            Lgd{i} = [Lgd_var, '\,\,(',Lgd_alpha,')'];
    end
    plot(du_dtheta*lin_r(i).D(4,5:end),'D-','DisplayName',Lgd{i});
end
h = gca;
ylim([0,h.YLim(2)])
grid on
box on
xlabel('Propellers')
ylabel('Forward effectiveness ((m/s^2)/°)')
title('Forward Effectiveness')
legend('location','south','interpreter','latex')
writeTikz(['exports/ForwardEff_',x_name,'.tex'], write_tikz)

figure
hold on
for i = 1:length(lin_r)
    plot(du_dtheta*lin_r(i).D(3,5:end),'D-','DisplayName',Lgd{i});
end
grid on
yline(0, '-', 'Color', 0.5*[1,1,1], 'HandleVisibility', 'off');
box on
xlabel('Propellers')
ylabel('Yaw effectiveness ((rad/s^2)/°)')
title('Yaw Effectiveness')
writeTikz(['exports/YawEff_',x_name,'.tex'], write_tikz)

figure
hold on
for i = 1:length(lin_r)
    plot(du_dtheta*-lin_r(i).D(6,5:end),'D-','DisplayName',Lgd{i});
end
h = gca;
ylim([0,h.YLim(2)])
grid on
box on
xlabel('Propellers')
ylabel('Upward effectiveness ((m/s^2)/°)')
title('Upward Effectiveness')
writeTikz(['exports/UpwardEff_',x_name,'.tex'], write_tikz)

figure
hold on
for i = 1:length(lin_r)
    plot(du_dtheta*lin_r(i).D(1,5:end),'D-','DisplayName',Lgd{i});
end
grid on
yline(0, '-', 'Color', 0.5*[1,1,1], 'HandleVisibility', 'off');
box on
xlabel('Propellers')
ylabel('Roll effectiveness ((m/s^2)/°)')
title('Roll Effectiveness')
writeTikz(['exports/RollEff_',x_name,'.tex'], write_tikz)

