%% Init propeller
clear;
addPathConfig2a();

% Set to true if Tikz export is desired
tikz_write = false;

mat = 'data/xprop_data_new_unfiltered_lin.mat';
xprop_build_matfile('data/xprop_data_new_unfiltered.txt', mat,'linear');
[tbl, tbl_bus, tbl_meta] = xprop_load_table(mat);


rho     = 0.609542;
RPM     = 1225;
n       = RPM / 60;
D       = 2.35;
pitches = -15:1:15;
V_range = 80:30:170;


%% Plot propeller map and control effectiveness

f1 = figure;
hold on; grid on; box on;
xlabel('Pitch $\theta$ (deg)','interpreter','latex');
ylabel('Thrust $T$ (N)','interpreter','latex');
hf1 = {};

f2 = figure(2);
hold on; grid on; box on;
xlabel('Pitch $\theta$ (deg)','interpreter','latex')
ylabel('$\partial T/\partial \theta$ (N/deg)','interpreter','latex')

f3 = figure(3);
hold on; grid on; box on;
xlabel('Pitch $\theta$ (deg)','interpreter','latex')
ylabel('$\partial v_i/\partial \theta$ ((m/s)/deg)','interpreter','latex')

f4 = figure(4);
hold on; grid on; box on;
xlabel('Pitch $\theta$ (deg)','interpreter','latex')
ylabel('$\partial v_i/\partial T$ ((m/s)/N)','interpreter','latex')

f5 = figure(5);
hold on; grid on; box on;
xlabel('Pitch $\theta$ (deg)','interpreter','latex')
ylabel('Torque $Q$ (Nm)','interpreter','latex');
hf5 = {};

f6 = figure(6);
hold on; grid on; box on;
xlabel('Pitch $\theta$ (deg)','interpreter','latex')
ylabel('$\partial Q/\partial \theta$ (Nm/deg)','interpreter','latex')

f7 = figure(7);
hold on; grid on; box on;
xlabel('Pitch $\theta$ (deg)','interpreter','latex')
ylabel('Induced velocity $v_i$ (m/s)','interpreter','latex')
hf7 = {};

f8 = figure(8);
hold on; grid on; box on;
xlabel('Pitch $\theta$ (deg)','interpreter','latex')
ylabel('Tangential velocity $v_t$ (m/s)','interpreter','latex')
hf8 = {};

f9 = figure(9);
hold on; grid on; box on;
xlabel('Pitch $\theta$ (deg)','interpreter','latex')
ylabel('$\partial v_t/\partial \theta$ ((m/s)/deg)','interpreter','latex')

pitches_abs = pitches + 53.5;
omega = n * 2 * pi;
R = D/2;
colors  = lines(length(V_range));
for k = 1:length(V_range)
    Tv = zeros(length(pitches),1);
    Qv = Tv;
    T_dJv = zeros(length(pitches),1);
    Q_dJv = zeros(length(pitches),1);
    nextrapv = false(length(pitches),1);
    for i = 1:length(pitches)
        pd = pitches(i);  
        [T,Q,~,vi,~,~,~,~,~,extrap] = propeller_aero(tbl, RPM, pd, V_range(k), rho);
        [dT_dtheta, dP_dtheta, dCt_dbeta, dCp_dbeta] = ...
            propeller_aero_deriv(tbl, RPM, pd, V_range(k), rho);
        Tv(i) = T;
        Qv(i) = Q;
        T_dJv(i) = dT_dtheta;
        Q_dJv(i) = dP_dtheta / omega;
        nextrapv(i) = extrap~=1;
    end

    figure(f1)
    plot(pitches_abs, Tv, '--', 'Color', colors(k,:))
    hf1{k} = plot(pitches_abs(nextrapv), Tv(nextrapv), '-', 'Color', colors(k,:), ...
        'DisplayName', sprintf('$V=%.0f$ m/s',V_range(k)));
    xlim([min(pitches_abs),max(pitches_abs)])
    ylim([-3000,inf])

    figure(f2)
    plot(pitches_abs, T_dJv, '--', 'Color', colors(k,:))
    plot(pitches_abs(nextrapv), T_dJv(nextrapv), '-', 'Color', colors(k,:));
    xlim([min(pitches_abs),max(pitches_abs)])
    A = pi/4*D*D;
    dV_prop_dT = 1./(4*rho*A*sqrt((V_range(k)/2).^2+Tv/(2*rho*A)));
    V_prop = -V_range(k)/2 + sqrt((V_range(k)/2).^2+Tv/(2*rho*A));

    figure(f3)
    dV_prop_dtheta = dV_prop_dT.*T_dJv;
    plot(pitches_abs, dV_prop_dtheta, '--', 'Color', colors(k,:))
    plot(pitches_abs(nextrapv), dV_prop_dtheta(nextrapv), '-', 'Color', colors(k,:))
    xlim([min(pitches_abs),max(pitches_abs)])

    figure(f4)
    Thrust = linspace(-1500,2500,100);
    dV_prop_dT = 1./(4*rho*A*sqrt((V_range(k)/2).^2+Tv/(2*rho*A)));
    plot(pitches_abs, dV_prop_dT, '--', 'Color', colors(k,:))
    plot(pitches_abs(nextrapv), dV_prop_dT(nextrapv), '-', 'Color', colors(k,:))
    xlim([min(pitches_abs),max(pitches_abs)])

    figure(f5)
    plot(pitches_abs, Qv, '--', 'Color', colors(k,:))
    hf5{k} = plot(pitches_abs(nextrapv), Qv(nextrapv), '-', 'Color', colors(k,:), ...
        'DisplayName', sprintf('$V=%.0f$ m/s',V_range(k)));
    xlim([min(pitches_abs),max(pitches_abs)])
    ylim([-1000,inf])

    figure(f6)
    plot(pitches_abs, Q_dJv, '--', 'Color', colors(k,:))
    plot(pitches_abs(nextrapv), Q_dJv(nextrapv), '-', 'Color', colors(k,:), ...
        'DisplayName', sprintf('V=%.0fm/s',V_range(k)));
    xlim([min(pitches_abs),max(pitches_abs)])

    figure(f7)
    plot(pitches_abs, V_prop ,'--', 'Color', colors(k,:))
    hf7{k} = plot(pitches_abs(nextrapv), V_prop(nextrapv) ,'-', 'Color', colors(k,:), ...
        'DisplayName', sprintf('$V=%.0f$ m/s',V_range(k)));
    xlim([min(pitches_abs),max(pitches_abs)])
    vt = 3/2 * Qv ./ (pi * rho * R.^3 .* (V_range(k)+vi));
    dvt_dtheta = 3/2 * Q_dJv ./ (pi * rho * R.^3 .* (V_range(k)+vi));

    figure(f8)
    plot(pitches_abs, vt, '--', 'Color', colors(k,:))
    hf8{k} = plot(pitches_abs(nextrapv), vt(nextrapv), '-', 'Color', colors(k,:), ...
        'DisplayName', sprintf('$V=%.0f$ m/s',V_range(k)));
    xlim([min(pitches_abs),max(pitches_abs)])

    figure(f9)
    plot(pitches_abs, dvt_dtheta, '--', 'Color', colors(k,:))
    plot(pitches_abs(nextrapv), dvt_dtheta(nextrapv), '-', 'Color', colors(k,:))
    xlim([min(pitches_abs),max(pitches_abs)])
end


%% Export Tikz files

figure(f1)
legend([hf1{:}],'Location','southeast','interpreter','latex');
writeTikz('prop_thrust_map.tex',tikz_write,{'scaled y ticks=base 10:-3'})
figure(f2)
writeTikz('prop_dthrust_dtheta.tex',tikz_write,{'scaled y ticks=base 10:-2'})
figure(f3)
writeTikz('prop_dvi_dtheta.tex',tikz_write,{'scaled y ticks=base 10:1'})
figure(f4)
writeTikz('prop_dvi_dthrust.tex',tikz_write)
figure(f5)
legend([hf5{:}],'Location','northwest','interpreter','latex');
writeTikz('prop_torque_map.tex',tikz_write,{'scaled y ticks=base 10:-3'})
figure(f6)
writeTikz('prop_dtorque_dtheta.tex',tikz_write,{'scaled y ticks=base 10:-2'})
figure(f7)
writeTikz('prop_vi.tex',tikz_write)
legend([hf7{:}],'Location','northwest','interpreter','latex');
figure(f8)
writeTikz('prop_vt.tex',tikz_write)
legend([hf8{:}],'Location','northwest','interpreter','latex');
figure(f9)
writeTikz('prop_dvt_dtheta.tex',tikz_write)

