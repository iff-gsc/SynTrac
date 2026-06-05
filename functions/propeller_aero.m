function [T, Q, P, vi, v_slipstream, Ct, Cp, J, eff] = ...
    propeller_aero(tbl, RPM, pitch_delta, V, rho)

D = 2.35;   R = D/2;   A = pi*R^2;
n     = RPM / 60;
omega = n * 2 * pi;
J     = V / (n * D);

[Ct, Cp] = interp_bilinear(tbl.pitch_vec, tbl.J_vec, tbl.Ct_mat, tbl.Cp_mat, pitch_delta, J);

T = Ct * rho * n^2 * D^4;
P = Cp * rho * n^3 * D^5;
Q = P / omega;

if V > 0.1; eff = T * V / P; else; eff = 0; end

disc = (V/2)^2 + T / (2*rho*A);
if disc >= 0; vi = -V/2 + sqrt(disc); else; vi = 0; end
v_slipstream = 2 * vi;

end


%% =========================================================================
function [val1, val2] = interp_bilinear(pitch_vec, J_vec, mat1, mat2, pitch_delta, J)
% INTERP_BILINEAR  Fast bilinear interpolation on a complete regular grid.
% Grid is guaranteed complete (no ragged edges) after xprop_build_matfile.
% Single binary search per axis, 4 corner reads

n_p = length(pitch_vec);
n_J = length(J_vec);

% ---- Clamp inputs to grid bounds ----------------------------------------
pitch_c = max(pitch_vec(1), min(pitch_vec(end), pitch_delta));
J_c     = max(J_vec(1),     min(J_vec(end),     J));

% ---- Pitch bracket -------------------------------------------------------
ip = n_p - 1;
for i = 1:n_p-1
    if pitch_vec(i+1) >= pitch_c; ip = i; break; end
end

% ---- J bracket -----------------------------------------------------------
ij = n_J - 1;
for j = 1:n_J-1
    if J_vec(j+1) >= J_c; ij = j; break; end
end

% ---- Interpolation weights -----------------------------------------------
wp2 = (pitch_c - pitch_vec(ip)) / (pitch_vec(ip+1) - pitch_vec(ip));
wp1 = 1.0 - wp2;

wj2 = (J_c - J_vec(ij)) / (J_vec(ij+1) - J_vec(ij));
wj1 = 1.0 - wj2;

% ---- Bilinear blend from four corners ------------------------------------
val1 = wp1*(wj1*mat1(ip,ij) + wj2*mat1(ip,ij+1)) + ...
       wp2*(wj1*mat1(ip+1,ij) + wj2*mat1(ip+1,ij+1));

val2 = wp1*(wj1*mat2(ip,ij) + wj2*mat2(ip,ij+1)) + ...
       wp2*(wj1*mat2(ip+1,ij) + wj2*mat2(ip+1,ij+1));

end