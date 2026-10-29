function [dT_dtheta, dP_dtheta, dCt_dbeta, dCp_dbeta, extrap] = ...
    propeller_aero_deriv(tbl, RPM, pitch_delta, V, rho, D)

% D: propeller diameter in m (optional, default 2.35 m, the diameter of the
% XROTOR propeller the table was computed for)
if nargin < 6
    D = 2.35;
end
n     = RPM / 60;
J     = V / (n * D);

[dCt_dbeta, dCp_dbeta, extrap] = interp_bilinear(tbl.pitch_vec, tbl.J_vec, ...
    pitch_delta, J, tbl.dCt_dbeta_mat, tbl.dCp_dbeta_mat, tbl.extrap_mat);

dT_dtheta = dCt_dbeta * rho * n^2 * D^4;
dP_dtheta = dCp_dbeta * rho * n^3 * D^5;

end
