% validate_propeller_model.m

clear; close all; clc;

%% ---- Build .mat if needed, load once ------------------------------------
txt = 'data/xprop_data.txt';
mat = 'data/xprop_data.mat';
if ~isfile(mat)
    xprop_build_matfile(txt, mat);
end
[tbl, tbl_bus, tbl_meta] = xprop_load_table(mat);
return
%% ---- Reference conditions -----------------------------------------------
rho     = 0.609542;
RPM     = 1225;
n       = RPM / 60;
D       = 2.35;
pitches = -15:5:15;
colors  = lines(length(pitches));

% Per-pitch V range using valid_mat to find J extents per row
V_ranges = cell(length(pitches), 1);
for k = 1:length(pitches)
    row_idx  = find(tbl.pitch_vec == pitches(k));
    v        = logical(tbl.valid_mat(row_idx, :));
    J_row    = tbl.J_mat(row_idx, v);
    V_ranges{k} = linspace(J_row(1)*n*D, J_row(end)*n*D, 80);
end

%% ===== FIGURE 1: Ct and Cp vs J ==========================================
figure('Name','Ct Cp vs J','Position',[100 100 1100 500]);
subplot(1,2,1); hold on; grid on; title('C_t vs J'); xlabel('J'); ylabel('C_t');
subplot(1,2,2); hold on; grid on; title('C_p vs J'); xlabel('J'); ylabel('C_p');

for k = 1:length(pitches)
    pd = pitches(k);  V_range = V_ranges{k};
    Jv = zeros(length(V_range),1);  Ctv = Jv;  Cpv = Jv;
    for i = 1:length(V_range)
        [~,~,~,~,~,Ct,Cp,J,~] = propeller_aero(tbl, RPM, pd, V_range(i), rho);
        Jv(i) = J;  Ctv(i) = Ct;  Cpv(i) = Cp;
    end
    subplot(1,2,1); plot(Jv, Ctv, '-', 'Color', colors(k,:), ...
        'DisplayName', sprintf('\\Delta\\beta=%+d°',pd));
    subplot(1,2,2); plot(Jv, Cpv, '-', 'Color', colors(k,:), ...
        'DisplayName', sprintf('\\Delta\\beta=%+d°',pd));
end
subplot(1,2,1); legend('Location','northeast','FontSize',8);
subplot(1,2,2); legend('Location','northeast','FontSize',8);

%% ===== FIGURE 2: Thrust and Torque vs V ==================================
figure('Name','T Q vs V','Position',[100 650 1100 500]);
subplot(1,2,1); hold on; grid on; title('Thrust vs V'); xlabel('V [m/s]'); ylabel('T [N]');
subplot(1,2,2); hold on; grid on; title('Torque vs V'); xlabel('V [m/s]'); ylabel('Q [N·m]');

for k = 1:length(pitches)
    pd = pitches(k);  V_range = V_ranges{k};
    Tv = zeros(length(V_range),1);  Qv = Tv;
    for i = 1:length(V_range)
        [T,Q] = propeller_aero(tbl, RPM, pd, V_range(i), rho);
        Tv(i) = T;  Qv(i) = Q;
    end
    subplot(1,2,1); plot(V_range, Tv, '-', 'Color', colors(k,:), ...
        'DisplayName', sprintf('\\Delta\\beta=%+d°',pd));
    subplot(1,2,2); plot(V_range, Qv, '-', 'Color', colors(k,:), ...
        'DisplayName', sprintf('\\Delta\\beta=%+d°',pd));
end
subplot(1,2,1); legend('Location','southwest','FontSize',8);
subplot(1,2,2); legend('Location','southwest','FontSize',8);

%% ===== FIGURE 3: Induced velocity ========================================
figure('Name','vi vs V','Position',[650 100 700 450]);
hold on; grid on;
title('Induced Velocity at Disk'); xlabel('V [m/s]'); ylabel('v_i [m/s]');

for k = 1:length(pitches)
    pd = pitches(k);  V_range = V_ranges{k};
    viv = zeros(length(V_range),1);
    for i = 1:length(V_range)
        [~,~,~,vi] = propeller_aero(tbl, RPM, pd, V_range(i), rho);
        viv(i) = vi;
    end
    plot(V_range, viv, '-', 'Color', colors(k,:), ...
        'DisplayName', sprintf('\\Delta\\beta=%+d°',pd));
end
legend('Location','northeast','FontSize',8);

%% ===== FIGURE 4: Pitch derivatives =======================================
figure('Name','Pitch Derivatives','Position',[650 600 1000 420]);
subplot(1,3,1); hold on; grid on; title('dT/d\beta [N/deg]');       xlabel('V [m/s]');
subplot(1,3,2); hold on; grid on; title('dQ/d\beta [N·m/deg]');     xlabel('V [m/s]');
subplot(1,3,3); hold on; grid on; title('dv_i/d\beta [(m/s)/deg]'); xlabel('V [m/s]');

for k = 1:length(pitches)
    pd = pitches(k);  V_range = V_ranges{k};
    dTb = zeros(length(V_range),1);  dQb = dTb;  dvib = dTb;
    for i = 1:length(V_range)
        [dT_dbeta, dQ_dbeta, dvi_dbeta] = propeller_derivatives(tbl, RPM, pd, V_range(i), rho);
        dTb(i) = dT_dbeta;  dQb(i) = dQ_dbeta;  dvib(i) = dvi_dbeta;
    end
    subplot(1,3,1); plot(V_range, dTb,  '-','Color',colors(k,:),'DisplayName',sprintf('%+d°',pd));
    subplot(1,3,2); plot(V_range, dQb,  '-','Color',colors(k,:),'DisplayName',sprintf('%+d°',pd));
    subplot(1,3,3); plot(V_range, dvib, '-','Color',colors(k,:),'DisplayName',sprintf('%+d°',pd));
end
for sp = 1:3; subplot(1,3,sp); legend('Location','best','FontSize',7); end

%% ===== FIGURE 5: Velocity derivatives ====================================
figure('Name','Velocity Derivatives','Position',[100 1200 1000 420]);
subplot(1,3,1); hold on; grid on; title('dT/dV [N/(m/s)]');       xlabel('V [m/s]');
subplot(1,3,2); hold on; grid on; title('dQ/dV [N·m/(m/s)]');     xlabel('V [m/s]');
subplot(1,3,3); hold on; grid on; title('dv_i/dV [(m/s)/(m/s)]'); xlabel('V [m/s]');

for k = 1:length(pitches)
    pd = pitches(k);  V_range = V_ranges{k};
    dTV = zeros(length(V_range),1);  dQV = dTV;  dviV = dTV;
    for i = 1:length(V_range)
        [~,~,~,dT_dV,dQ_dV,dvi_dV] = propeller_derivatives(tbl, RPM, pd, V_range(i), rho);
        dTV(i) = dT_dV;  dQV(i) = dQ_dV;  dviV(i) = dvi_dV;
    end
    subplot(1,3,1); plot(V_range, dTV,  '-','Color',colors(k,:),'DisplayName',sprintf('%+d°',pd));
    subplot(1,3,2); plot(V_range, dQV,  '-','Color',colors(k,:),'DisplayName',sprintf('%+d°',pd));
    subplot(1,3,3); plot(V_range, dviV, '-','Color',colors(k,:),'DisplayName',sprintf('%+d°',pd));
end
for sp = 1:3; subplot(1,3,sp); legend('Location','best','FontSize',7); end

%% ===== FIGURE 6: Efficiency — model vs XROTOR table =====================
s   = load(mat, 'raw');
raw = s.raw;

figure('Name','Efficiency model vs table','Position',[650 1200 1100 480]);
hold on; grid on;
title('Propulsive Efficiency \eta  —  model (T\cdotV/P) vs XROTOR table');
xlabel('J [-]'); ylabel('\eta [-]');

for k = 1:length(pitches)
    pd = pitches(k);  V_range = V_ranges{k};
    Jv   = zeros(length(V_range),1);
    effv = Jv;
    for i = 1:length(V_range)
        [~,~,~,~,~,~,~,J,eff] = propeller_aero(tbl, RPM, pd, V_range(i), rho);
        Jv(i) = J;  effv(i) = eff;
    end
    plot(Jv, effv, '-', 'Color', colors(k,:), 'LineWidth', 1.4, ...
         'DisplayName', sprintf('model \\Delta\\beta=%+d°', pd));
    rows = raw(:,1) == pd;
    plot(raw(rows,6), raw(rows,5), 'o', 'Color', colors(k,:), ...
         'MarkerFaceColor', 'w', 'MarkerSize', 5, 'HandleVisibility', 'off');
end
plot(nan, nan, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 5, ...
     'DisplayName', 'XROTOR table points');
legend('Location','southeast','FontSize',8);
ylim([0.5 1.0]);

%% ===== FIGURE 7: Ct_mat raw data — columns vs pitch =====================
% Each line is one J column plotted against pitch_vec (x-axis).
% Solid line = full row (original + extrapolated). Open circles = original data only.
s_mat      = load(mat, 'Ct_mat', 'Cp_mat', 'J_vec', 'valid_mat');
Ct_mat_raw = s_mat.Ct_mat;
Cp_mat_raw = s_mat.Cp_mat;
J_vec_raw  = s_mat.J_vec;
valid_raw  = logical(s_mat.valid_mat);
n_J_raw    = length(J_vec_raw);
figure('Name','Ct_mat raw columns vs pitch','Position',[100 100 1100 500]);
hold on; grid on;
title('C_t');
xlabel('\Delta\beta [deg]'); ylabel('C_t [-]');
for j = 1:n_J_raw
    valid_rows = valid_raw(:, j);
    if ~any(valid_rows); continue; end
%     plot(tbl.pitch_vec, Ct_mat_raw(:, j), '-', 'LineWidth', 1.2, ...
%          'DisplayName', sprintf('V=%.1f m/s', J_vec_raw(j)*n*D));
    plot(tbl.pitch_vec(valid_rows), Ct_mat_raw(valid_rows, j), '-', ...
         'MarkerFaceColor', 'w', 'MarkerSize', 4, 'DisplayName', sprintf('V=%.1f m/s', J_vec_raw(j)*n*D));
end
legend('Location','bestoutside','FontSize',6,'NumColumns',2);

%% ===== FIGURE 8: Cp_mat raw data — columns vs pitch =====================
figure('Name','Cp_mat raw columns vs pitch','Position',[100 650 1100 500]);
hold on; grid on;
title('C_p  —  raw grid columns (each line = one J value)');
xlabel('\Delta\beta [deg]'); ylabel('C_p [-]');
for j = 1:n_J_raw
    valid_rows = valid_raw(:, j);
    if ~any(valid_rows); continue; end
   % plot(tbl.pitch_vec, Cp_mat_raw(:, j), '-', 'LineWidth', 1.2, ...
   %      'DisplayName', sprintf('V=%.1f m/s', J_vec_raw(j)*n*D));
    plot(tbl.pitch_vec(valid_rows), Cp_mat_raw(valid_rows, j), '-', ...
         'MarkerFaceColor', 'w', 'MarkerSize', 4, 'DisplayName', sprintf('V=%.1f m/s', J_vec_raw(j)*n*D));
end
legend('Location','bestoutside','FontSize',6,'NumColumns',2);


%% ===== Example operating point ===========================================
fprintf('\n========== EXAMPLE OPERATING POINT ==========\n');
[T,Q,P,vi,v_sw,Ct,Cp,J,eff] = propeller_aero(tbl, 1225, 5, 60, 0.609542);
[dT_dbeta,dQ_dbeta,dvi_dbeta,dT_dV,dQ_dV,dvi_dV] = ...
    propeller_derivatives(tbl, 1225, 5, 60, 0.609542);
fprintf('RPM=1225  pitch_delta=+5 deg  V=60 m/s  rho=0.609542 kg/m3\n');
fprintf('  J    = %.4f\n',  J);
fprintf('  Ct   = %.5f\n',  Ct);
fprintf('  Cp   = %.5f\n',  Cp);
fprintf('  T    = %.1f N\n',   T);
fprintf('  Q    = %.2f N·m\n', Q);
fprintf('  P    = %.1f W\n',   P);
fprintf('  eff  = %.2f%%\n',   eff*100);
fprintf('  vi   = %.3f m/s\n', vi);
fprintf('  v_sw = %.3f m/s\n', v_sw);
fprintf('--- Pitch derivatives ---\n');
fprintf('  dT/dbeta  = %.2f N/deg\n',     dT_dbeta);
fprintf('  dQ/dbeta  = %.3f N·m/deg\n',   dQ_dbeta);
fprintf('  dvi/dbeta = %.4f (m/s)/deg\n', dvi_dbeta);
fprintf('--- Velocity derivatives ---\n');
fprintf('  dT/dV  = %.2f N/(m/s)\n',       dT_dV);
fprintf('  dQ/dV  = %.3f N·m/(m/s)\n',     dQ_dV);
fprintf('  dvi/dV = %.4f (m/s)/(m/s)\n',   dvi_dV);
fprintf('==============================================\n');