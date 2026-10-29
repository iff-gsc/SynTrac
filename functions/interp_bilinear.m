function [varargout] = interp_bilinear(pitch_vec, J_vec, pitch_delta, J, varargin)
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
for i = 1:length(varargin)
    varargout{i} = wp1*(wj1*varargin{i}(ip,ij) + wj2*varargin{i}(ip,ij+1)) + ...
           wp2*(wj1*varargin{i}(ip+1,ij) + wj2*varargin{i}(ip+1,ij+1));
end

end