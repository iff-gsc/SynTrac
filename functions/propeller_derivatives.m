function [dT_dbeta, dQ_dbeta, dvi_dbeta, dT_dV, dQ_dV, dvi_dV] = ...
    propeller_derivatives(tbl, RPM, pitch_delta, V, rho)
% PROPELLER_DERIVATIVES  Linearised pitch and velocity derivatives.

D = 2.35;   R = D/2;   A = pi*R^2;
n     = RPM / 60;
omega = n * 2 * pi;
J     = V / (n * D);

[Ct, ~] = interp_bilinear(tbl.pitch_vec, tbl.J_vec, tbl.Ct_mat, tbl.Cp_mat, pitch_delta, J);

[dCt_dbeta, dCp_dbeta] = interp_bilinear(tbl.pitch_vec, tbl.J_vec, ...
                           tbl.dCt_dbeta_mat, tbl.dCp_dbeta_mat, pitch_delta, J);

[dCt_dJ, dCp_dJ] = interp_bilinear(tbl.pitch_vec, tbl.J_vec, ...
                     tbl.dCt_dJ_mat, tbl.dCp_dJ_mat, pitch_delta, J);

dT_dbeta = dCt_dbeta * rho * n^2 * D^4;
dQ_dbeta = (dCp_dbeta * rho * n^3 * D^5) / omega;

dJ_dV = 1 / (n * D);
dT_dV = dCt_dJ * dJ_dV * rho * n^2 * D^4;
dQ_dV = (dCp_dJ * dJ_dV * rho * n^3 * D^5) / omega;

T    = Ct * rho * n^2 * D^4;
disc = (V/2)^2 + T / (2*rho*A);
if disc > 0; vi = -V/2 + sqrt(disc); else; vi = 0; end

denom = 2 * rho * A * (V + 2*vi);
if denom > 1e-6; dvi_dbeta = dT_dbeta / denom; else; dvi_dbeta = 0; end

if disc > 0
    dvi_dV = -0.5 + (1/(2*sqrt(disc))) * (V/2 + dT_dV/(2*rho*A));
else
    dvi_dV = 0;
end

end


%% =========================================================================
function [val1, val2] = interp_bilinear(pitch_vec, J_vec, mat1, mat2, pitch_delta, J)

n_p = length(pitch_vec);
n_J = length(J_vec);

pitch_c = max(pitch_vec(1), min(pitch_vec(end), pitch_delta));
J_c     = max(J_vec(1),     min(J_vec(end),     J));

ip = n_p - 1;
for i = 1:n_p-1
    if pitch_vec(i+1) >= pitch_c; ip = i; break; end
end

ij = n_J - 1;
for j = 1:n_J-1
    if J_vec(j+1) >= J_c; ij = j; break; end
end

wp2 = (pitch_c - pitch_vec(ip)) / (pitch_vec(ip+1) - pitch_vec(ip));
wp1 = 1.0 - wp2;
wj2 = (J_c - J_vec(ij)) / (J_vec(ij+1) - J_vec(ij));
wj1 = 1.0 - wj2;

val1 = wp1*(wj1*mat1(ip,ij) + wj2*mat1(ip,ij+1)) + ...
       wp2*(wj1*mat1(ip+1,ij) + wj2*mat1(ip+1,ij+1));

val2 = wp1*(wj1*mat2(ip,ij) + wj2*mat2(ip,ij+1)) + ...
       wp2*(wj1*mat2(ip+1,ij) + wj2*mat2(ip+1,ij+1));

end