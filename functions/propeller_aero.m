function [T, Q, P, vi, v_slipstream, Ct, Cp, J, eff, extrap] = ...
    propeller_aero(tbl, RPM, pitch_delta, V, rho, D)

% D: propeller diameter in m (optional, default 2.35 m, the diameter of the
% XROTOR propeller the table was computed for)
if nargin < 6
    D = 2.35;
end
R = D/2;   A = pi*R^2;
n     = RPM / 60;
omega = n * 2 * pi;
J     = V / (n * D);

[Ct, Cp, extrap] = interp_bilinear(tbl.pitch_vec, tbl.J_vec, pitch_delta, J, tbl.Ct_mat, tbl.Cp_mat, tbl.extrap_mat);

T = Ct * rho * n^2 * D^4;
P = Cp * rho * n^3 * D^5;
Q = P / omega;

if V > 0.1; eff = T * V / P; else; eff = 0; end

disc = (V/2)^2 + T / (2*rho*A);
if disc >= 0; vi = -V/2 + sqrt(disc); else; vi = 0; end
v_slipstream = 2 * vi;

end
