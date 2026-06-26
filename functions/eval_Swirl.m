% eval_inducedSwirl  Quick verification / documentation figures for inducedSwirl.
%
%   Sweeps the swirl model across the rotor span for a single operating
%   point and plots
%       (1) the additional (swirl) flow velocity w_theta over the span,
%       (2) the corresponding change in angle of attack in degrees,
%   and adds a torque sweep that shows the power-dependence of the effect.
%   The signs are easy to verify by eye: w_theta is antisymmetric (downward
%   on the right wing, upward on the left), and a downwash reduces alpha, so
%   delta_alpha < 0 on the right and > 0 on the left.
%
% See also: inducedSwirl, propeller_aero

clear; clc; close all;

%% Operating point (SI; from propeller_aero) ------------------------------
V   = 150;      % flight speed                  [m/s]
vi  = 3.0;      % induced velocity (momentum)   [m/s]
Q   = 2000;     % rotor torque = P/omega        [N*m]
R   = 1.175;    % rotor radius (D = 2.35 m)     [m]
rho = 0.65;     % air density                   [kg/m^3]

s      = 0.5;               % wake development: 0 = at disk, 1 = far wake
v_disk = V + vi;            % throughflow at the disk (sets swirl magnitude)
v_wing = V + (1 + s)*vi;    % velocity the wing sees (AoA conversion)

%% Span sweep -------------------------------------------------------------
y = linspace(-R, R, 201);   % spanwise stations, +y = right wing  [m]
[omega_s, w_theta, delta_alpha] = inducedSwirl(Q, v_disk, v_wing, R, rho, y);
da_deg = delta_alpha * 180/pi;

fprintf('omega_s   = %.3f rad/s\n', omega_s);
fprintf('tip swirl = %.2f m/s   tip dAoA = %.2f deg\n', max(w_theta), min(da_deg));

%% Figure 1: swirl velocity and delta-AoA across the span -----------------
figure('Color','w','Position',[100 100 980 390]);

subplot(1,2,1);
plot(y, w_theta, 'LineWidth', 2.0, 'Color', [0.12 0.37 0.66]); hold on; grid on;
yline(0,'k:'); xline(0,'k:');
xlabel('spanwise position  y  [m]   (+y = right wing)');
ylabel('swirl velocity  w_\theta  [m/s]   (+ = downward)');
title('Additional (swirl) flow velocity');

subplot(1,2,2);
plot(y, da_deg, 'LineWidth', 2.0, 'Color', [0.75 0.22 0.17]); hold on; grid on;
yline(0,'k:'); xline(0,'k:');
xlabel('spanwise position  y  [m]   (+y = right wing)');
ylabel('\Delta\alpha  [deg]');
title('Induced change in angle of attack');
text(-0.55*R, max(da_deg)*0.55, {'left wing:','\alpha increased'}, ...
     'HorizontalAlignment','center','FontSize',8,'Color',[0.12 0.37 0.66]);
text( 0.55*R, min(da_deg)*0.55, {'right wing:','\alpha reduced'}, ...
     'HorizontalAlignment','center','FontSize',8,'Color',[0.75 0.22 0.17]);

sgtitle(sprintf(['Propeller swirl on the wing   ' ...
                 '(Q = %.0f N\\cdotm, V = %.0f m/s, v_i = %.0f m/s, ' ...
                 'R = %.3f m, \\rho = %.2f)'], Q, V, vi, R, rho));

%% Figure 2: power (torque) dependence ------------------------------------
figure('Color','w','Position',[100 560 600 390]);
Q_list = [1000 2000 4000];
hold on; grid on;
for Qi = Q_list
    [~, ~, da_i] = inducedSwirl(Qi, v_disk, v_wing, R, rho, y);
    plot(y, da_i*180/pi, 'LineWidth', 1.8, ...
         'DisplayName', sprintf('Q = %d N\\cdotm', Qi));
end
yline(0,'k:'); xline(0,'k:');
xlabel('spanwise position  y  [m]   (+y = right wing)');
ylabel('\Delta\alpha  [deg]');
title('Swirl \Delta\alpha scales with torque (power)');
legend('Location','northeast');