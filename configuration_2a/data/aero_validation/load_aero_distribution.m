%% Plot spanwise lift distribution (symmetric) from CFD force/moment data
%
% Reads the spanwise force/moment derivative distribution exported from
% the high-fidelity CFD solution (Tecplot POINT format, see
% readTecplotPointFile.m) for one wing half, and plots the lift
% distribution (dFz/dy) mirrored onto the full span.
%
% Assumes:
%   - Input file contains data for one wing side only (e.g. right wing).
%   - The configuration/flow condition is symmetric, so the left wing
%     can be obtained by mirroring y -> -y without flipping the sign of
%     dFz/dy (lift is a scalar load per unit span, unlike e.g. rolling
%     moment dMx/dy, which would need a sign flip under mirroring).
%
% Input file: distributions_AOA0.dat  (AoA = 0 deg case)

data = readTecplotPointFile('distributions_AOA3.dat');
data.varNames    % cleaned variable names, e.g. {'y','6Fx/6y','6Fy/6y',...}

% Extract spanwise coordinate and force/moment derivatives
y      = data.values(:,1);   % spanwise position
dFx_dy = data.values(:,2);
dFy_dy = data.values(:,3);
dFz_dy = data.values(:,4);   % lift distribution
dMx_dy = data.values(:,5);
dMy_dy = data.values(:,6);
dMz_dy = data.values(:,7);

% Mirror right wing data to get the left wing (lift distribution is symmetric)
y_full      = [-flipud(y); y];
dFz_dy_full = [flipud(dFz_dy); dFz_dy];

% Plot
figure;
plot(y_full, dFz_dy_full, '-o', 'LineWidth', 1.5);
xlabel('y [m]');
ylabel('dF_z/dy [N/m]');
title('Spanwise Lift Distribution');
grid on;