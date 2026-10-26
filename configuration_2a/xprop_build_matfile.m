function xprop_build_matfile(txt_file, mat_file)
% XPROP_BUILD_MATFILE  Parse XROTOR plain-text data and save a .mat file.
%
% Offline preprocessing step — run once after any data change.
% Computes Ct/Cp grids AND all derivative grids (dCt/dbeta, dCp/dbeta,
% dCt/dJ, dCp/dJ) at every table node via central finite differences.
% All grids are complete and smooth over the full table: original XROTOR
% points are kept exactly, isolated holes are spline-filled, the region
% beyond zero thrust is filled with a windmill/brake shape in four-quadrant
% scaling, and all non-trusted nodes are smoothed in 2-D (minimum
% curvature). valid_mat marks the trustworthy nodes, stall_mat the stalled
% side of each pitch row. See data/XPROP_TABLE.md.
% All runtime functions only interpolate — no derivatives computed online.
%
% USAGE:
%   xprop_build_matfile('xprop_data.txt')
%   xprop_build_matfile('xprop_data.txt', 'xprop_data.mat')
%
% INPUT FILE FORMAT (plain text, whitespace-delimited):
%   Header line:  pitch_delta  V  Ct  Cp  eff  J  [Tc optional, ignored]
%   Comment lines starting with % or # are skipped.
%
% OUTPUT .mat variables:
%   raw           [N x 6]          raw data: [pitch_delta, V, Ct, Cp, eff, J]
%   pitch_vec     [n_p x 1]        unique pitch_delta values [deg]
%   J_vec         [n_J x 1]        unique J values [-]
%   J_mat         [n_p x n_J]      advance ratio grid (J_vec in every row)
%   Ct_mat        [n_p x n_J]      thrust coefficient
%   Cp_mat        [n_p x n_J]      power coefficient
%   dCt_dbeta_mat [n_p x n_J]      dCt/d(pitch_delta)  [1/deg]
%   dCp_dbeta_mat [n_p x n_J]      dCp/d(pitch_delta)  [1/deg]
%   dCt_dJ_mat    [n_p x n_J]      dCt/dJ              [1/-]
%   dCp_dJ_mat    [n_p x n_J]      dCp/dJ              [1/-]
%   valid_mat     [n_p x n_J]      true = node usable (see Valid mask)
%   data_mat      [n_p x n_J]      true = original XROTOR point
%   filled_mat    [n_p x n_J]      true = interior hole, spline-filled
%   trusted_mat   [n_p x n_J]      true = data or trusted filled hole
%   stall_mat     [n_p x n_J]      true = stalled side of the row (see Stall mask)
%   J_peak        [n_p x 1]        J of the Ct maximum per pitch row
%   meta          struct           source file, build time, table extents

%% ---- Default arguments --------------------------------------------------
if nargin < 1 || isempty(txt_file)
    txt_file = 'xprop_data.txt';
end
if nargin < 2 || isempty(mat_file)
    [folder, name, ~] = fileparts(txt_file);
    mat_file = fullfile(folder, [name '.mat']);
end

fprintf('xprop_build_matfile\n');
fprintf('  Source : %s\n', txt_file);
fprintf('  Output : %s\n', mat_file);

%% ---- Parse text file ----------------------------------------------------
if ~isfile(txt_file)
    error('xprop_build_matfile:fileNotFound', 'Data file not found: %s', txt_file);
end
fid = fopen(txt_file, 'r');
if fid == -1
    error('xprop_build_matfile:cannotOpen', 'Cannot open file: %s', txt_file);
end

header_found = false;
col_pitch = []; col_V = []; col_Ct = []; col_Cp = []; col_eff = []; col_J = [];
data_rows = {};

while ~feof(fid)
    line = strtrim(fgetl(fid));
    if isempty(line) || line(1) == '%' || line(1) == '#'; continue; end

    if ~header_found
        tokens    = strsplit(lower(line));
        col_pitch = find(strcmp(tokens, 'pitch_delta'));
        col_V     = find(strcmp(tokens, 'v'));
        col_Ct    = find(strcmp(tokens, 'ct'));
        col_Cp    = find(strcmp(tokens, 'cp'));
        col_eff   = find(strcmp(tokens, 'eff'));
        col_J     = find(strcmp(tokens, 'j'));
        missing = {};
        if isempty(col_pitch), missing{end+1} = 'pitch_delta'; end
        if isempty(col_V),     missing{end+1} = 'V';           end
        if isempty(col_Ct),    missing{end+1} = 'Ct';          end
        if isempty(col_Cp),    missing{end+1} = 'Cp';          end
        if isempty(col_J),     missing{end+1} = 'J';           end
        if ~isempty(missing)
            fclose(fid);
            error('xprop_build_matfile:missingColumns', ...
                  'Missing columns: %s\nHeader: %s', strjoin(missing,', '), line);
        end
        header_found = true;
        fprintf('  Header : found (%d columns)\n', length(tokens));
        continue
    end

    nums = str2double(strsplit(line));
    if any(isnan(nums))
        warning('xprop_build_matfile:badLine', 'Skipping line: %s', line);
        continue
    end
    data_rows{end+1} = nums; %#ok<AGROW>
end
fclose(fid);

if ~header_found; error('xprop_build_matfile:noHeader', 'No header found.'); end
if isempty(data_rows); error('xprop_build_matfile:noData', 'No data rows found.'); end

%% ---- Assemble raw matrix ------------------------------------------------
n_rows     = length(data_rows);
n_cols_max = max(cellfun(@length, data_rows));
raw_full   = NaN(n_rows, n_cols_max);
for k = 1:n_rows
    r = data_rows{k};
    raw_full(k, 1:length(r)) = r;
end

raw = zeros(n_rows, 6);
raw(:,1) = raw_full(:, col_pitch);
raw(:,2) = raw_full(:, col_V);
raw(:,3) = raw_full(:, col_Ct);
raw(:,4) = raw_full(:, col_Cp);
if ~isempty(col_eff); raw(:,5) = raw_full(:, col_eff); end
raw(:,6) = raw_full(:, col_J);

fprintf('  Data   : %d rows parsed\n', n_rows);

assert(all(raw(:,3) >= 0), 'Ct values must be non-negative');
assert(all(raw(:,4) >= 0), 'Cp values must be non-negative');
assert(all(raw(:,6) >  0), 'J values must be positive');

%% ---- Build Ct/Cp grids aligned to common pitch and J axes ---------------
% Column j always corresponds to J_vec(j) in every row, so all finite
% differences below compare values at the same J (or the same pitch).
pitch_vec = unique(raw(:,1));
J_vec     = unique(round(raw(:,6), 6));   % sorted ascending
n_pitch   = length(pitch_vec);
n_J       = length(J_vec);
fprintf('  Pitch  : %d levels  (%g to %g deg)\n', n_pitch, pitch_vec(1), pitch_vec(end));

Ct_mat = NaN(n_pitch, n_J);
Cp_mat = NaN(n_pitch, n_J);
[~, row_idx] = ismember(raw(:,1), pitch_vec);
[~, col_idx] = ismember(round(raw(:,6), 6), J_vec);
lin_idx = sub2ind([n_pitch, n_J], row_idx, col_idx);
if length(unique(lin_idx)) < n_rows
    error('xprop_build_matfile:duplicatePoints', ...
          'Duplicate (pitch_delta, J) points in %s.', txt_file);
end
Ct_mat(lin_idx) = raw(:,3);
Cp_mat(lin_idx) = raw(:,4);

% data_mat(i,j) = true → real XROTOR point at (pitch_vec(i), J_vec(j))
data_mat = ~isnan(Ct_mat);

%% ---- Fill interior holes ------------------------------------------------
% Missing points inside the data envelope (typically non-converged XROTOR
% runs) would otherwise end up as Ct = Cp = 0 in the runtime table.
% 1) Holes with data on both sides in J: cubic spline along the pitch row.
% 2) Remaining holes with data on both sides in pitch: cubic spline along
%    the J column (e.g. a missing first point of a row).
% Leave-out tests on the complete old table gave errors < 2e-4 (Ct) and
% < 1e-3 (Cp) for the spline along J.
filled_mat = false(n_pitch, n_J);
for i = 1:n_pitch
    d = find(data_mat(i,:));
    if length(d) < 2; continue; end
    holes = d(1) + find(~data_mat(i, d(1):d(end))) - 1;
    if isempty(holes); continue; end
    Ct_mat(i, holes) = interp1(J_vec(d), Ct_mat(i, d), J_vec(holes), 'spline');
    Cp_mat(i, holes) = interp1(J_vec(d), Cp_mat(i, d), J_vec(holes), 'spline');
    filled_mat(i, holes) = true;
end
for j = 1:n_J
    d = find(~isnan(Ct_mat(:,j)));
    if length(d) < 2; continue; end
    holes = d(1) + find(isnan(Ct_mat(d(1):d(end), j))) - 1;
    if isempty(holes); continue; end
    Ct_mat(holes, j) = interp1(pitch_vec(d), Ct_mat(d, j), pitch_vec(holes), 'spline');
    Cp_mat(holes, j) = interp1(pitch_vec(d), Cp_mat(d, j), pitch_vec(holes), 'spline');
    filled_mat(holes, j) = true;
end
fprintf('  Holes  : %d interior points filled by spline interpolation\n', sum(filled_mat(:)));

%% ---- Trusted nodes ------------------------------------------------------
% Trusted nodes are original XROTOR points plus filled holes that are
% (a) isolated: original data at all four direct neighbours, and
% (b) consistent: the spline estimate along J (used in the grid) and an
%     independent spline estimate along pitch agree within tol_Ct/tol_Cp.
% Leave-one-out tests on both data sets: isolated holes are recovered with
% a 95th percentile error of 1.5e-3 (Ct) / 3e-3 (Cp); runs of two or more
% missing points are clearly worse and are never trusted.
% Trusted nodes are never changed by the smoothing below.
tol_Ct = 2e-3;
tol_Cp = 5e-3;
trusted_mat = data_mat;
[fi, fj] = find(filled_mat);
for k = 1:length(fi)
    i = fi(k); j = fj(k);
    if i == 1 || i == n_pitch || j == 1 || j == n_J; continue; end
    if ~(data_mat(i-1,j) && data_mat(i+1,j) && data_mat(i,j-1) && data_mat(i,j+1))
        continue
    end
    d = find(data_mat(:,j));
    Ct_col = interp1(pitch_vec(d), Ct_mat(d,j), pitch_vec(i), 'spline');
    Cp_col = interp1(pitch_vec(d), Cp_mat(d,j), pitch_vec(i), 'spline');
    if abs(Ct_col - Ct_mat(i,j)) <= tol_Ct && abs(Cp_col - Cp_mat(i,j)) <= tol_Cp
        trusted_mat(i,j) = true;
    end
end

%% ---- Extrapolate beyond zero thrust (windmill / brake prior) ------------
% Beyond the last point of a row (Ct -> 0) the propeller enters the
% windmill and brake states. No delivered data exists there. Instead of a
% plain linear continuation, each row is continued with a normalised
% windmill shape in four-quadrant scaling (as used for propeller
% four-quadrant data):
%   CT* = 8 Ct / (pi W2),  CP* = 8 Cp / (pi^2 0.7 W2),  W2 = J^2 + (0.7 pi)^2
% as a function of the geometric angle of attack at 0.7R,
%   aoa = 54.7 deg + pitch_delta - atan(J / (0.7 pi)).
% In this scaling all pitch rows of the XROTOR output collapse onto almost
% one curve. The shape functions windmill_hT/hP were fitted to the 118
% Ct < 0 points of the unfiltered XROTOR run of the new data set
% (xprop_data_new_unfiltered.txt, removed from the delivered data) and are
% scaled per row to the CT*/CP* slope of the row's last three points, so
% value and slope are continuous at the data edge. The same shape is
% assumed for the old data set. These values are a plausible placeholder
% for solver robustness, NOT data: none of these nodes is valid.
inside_mat = data_mat | filled_mat;
J_grid  = repmat(J_vec', n_pitch, 1);
W2_mat  = J_grid.^2 + (0.7*pi)^2;
aoa_mat = 54.7 + repmat(pitch_vec, 1, n_J) - atand(J_grid / (0.7*pi));
CTs = 8 * Ct_mat ./ (pi * W2_mat);
CPs = 8 * Cp_mat ./ (pi^2 * 0.7 * W2_mat);
for i = 1:n_pitch
    d = find(inside_mat(i,:));
    if length(d) < 3
        error('xprop_build_matfile:shortRow', ...
              'Pitch row %g has fewer than 3 points.', pitch_vec(i));
    end
    % Low-J side (only if a row starts late): linear in J
    lo = 1:d(1)-1;
    if ~isempty(lo)
        for m = 1:2
            if m == 1; row = CTs(i,:); else; row = CPs(i,:); end
            s_lo = (row(d(2)) - row(d(1))) / (J_vec(d(2)) - J_vec(d(1)));
            row(lo) = row(d(1)) + s_lo * (J_vec(lo)' - J_vec(d(1)));
            if m == 1; CTs(i,:) = row; else; CPs(i,:) = row; end
        end
    end
    % High-J side: windmill prior
    hi = d(end)+1:n_J;
    if isempty(hi); continue; end
    L  = d(end);
    k3 = d(end-2:end);
    cT = polyfit(aoa_mat(i,k3), CTs(i,k3), 1);
    cP = polyfit(aoa_mat(i,k3), CPs(i,k3), 1);
    if cT(1) > 0
        a_zt = aoa_mat(i,L) - CTs(i,L) / cT(1);    % zero-thrust angle
        x  = aoa_mat(i,hi) - a_zt;
        xL = aoa_mat(i,L)  - a_zt;
        CTs(i,hi) = CTs(i,L) + cT(1) * (windmill_hT(x) - windmill_hT(xL));
        CPs(i,hi) = CPs(i,L) + cP(1) * (windmill_hP(x) - windmill_hP(xL));
    else
        % no positive thrust slope: plain linear continuation in aoa
        CTs(i,hi) = CTs(i,L) + cT(1) * (aoa_mat(i,hi) - aoa_mat(i,L));
        CPs(i,hi) = CPs(i,L) + cP(1) * (aoa_mat(i,hi) - aoa_mat(i,L));
    end
end

%% ---- Smooth all non-trusted nodes in 2-D --------------------------------
% Trusted nodes stay exactly as they are. All other nodes (untrusted holes
% and the extrapolated region) minimise the squared second differences
% along J and pitch, so the surfaces and their derivatives join the data
% smoothly in both directions. Extrapolated nodes are additionally pulled
% towards the windmill prior (weight w_prior), so the far field follows
% its shape instead of a plain linear continuation. Done in CT*/CP*.
w_prior = 0.1;
free_mat = ~trusted_mat;
pull_mat = w_prior * double(~inside_mat);
x_J = J_vec / (J_vec(2) - J_vec(1));               % J in units of one step
CTs = smooth_fill(CTs, x_J, pitch_vec, free_mat, pull_mat);
CPs = smooth_fill(CPs, x_J, pitch_vec, free_mat, pull_mat);
Ct_mat(free_mat) = CTs(free_mat) .* pi .* W2_mat(free_mat) / 8;
Cp_mat(free_mat) = CPs(free_mat) .* pi^2 * 0.7 .* W2_mat(free_mat) / 8;

if any(isnan(Ct_mat(:))) || any(isnan(Cp_mat(:)))
    error('xprop_build_matfile:incompleteGrid', 'Grid still contains NaN after filling.');
end
fprintf('  Filled : %d extrapolated cells (windmill prior), %d holes smoothed\n', ...
        sum(~inside_mat(:)), sum(filled_mat(:) & ~trusted_mat(:)));

J_mat = J_grid;

%% ---- Precompute derivative grids ----------------------------------------
% Second-order finite differences on the complete, aligned grid (three-point
% formula for non-uniform spacing, one-sided at the table borders).
% Units: dCt/dbeta [1/deg], dCp/dbeta [1/deg], dCt/dJ [1/-], dCp/dJ [1/-]
fprintf('  Computing derivative grids...\n');
dCt_dbeta_mat = fd_deriv(Ct_mat,  pitch_vec);
dCp_dbeta_mat = fd_deriv(Cp_mat,  pitch_vec);
dCt_dJ_mat    = fd_deriv(Ct_mat', J_vec)';
dCp_dJ_mat    = fd_deriv(Cp_mat', J_vec)';

%% ---- Valid mask ---------------------------------------------------------
% valid_mat(i,j) = true  → the value AND all four derivatives at this node
%                          are computed exclusively from trusted nodes
% valid_mat(i,j) = false → node is extrapolated, an untrusted filled hole,
%                          or its finite-difference stencil touches one
% A node is valid only if every node of its derivative stencils (same
% stencils as fd_deriv) is trusted. This trims one cell at the data
% envelope edge and around untrusted holes.
valid_mat = trusted_mat & stencil_ok(trusted_mat) & stencil_ok(trusted_mat')';
fprintf('  Valid  : %d nodes (%d data, %d of %d filled holes trusted, %d trusted nodes excluded by stencil)\n', ...
        sum(valid_mat(:)), sum(data_mat(:)), sum(trusted_mat(:) & filled_mat(:)), ...
        sum(filled_mat(:)), sum(trusted_mat(:)) - sum(valid_mat(:)));

%% ---- Stall mask ---------------------------------------------------------
% stall_mat(i,j) = true → node on the stalled side of its pitch row, i.e. at
% or below the J of the Ct maximum (there Ct rises with J because the blade
% sections run at angles of attack beyond CLmax). Conservative:
% - the peak is searched on trusted nodes only,
% - the peak node and stall_margin nodes beyond it are included,
% - a row whose Ct still rises at its last trusted node is flagged entirely
%   (peak not reached inside the data),
% - a row whose maximum is its first trusted node has no stalled side.
% Independent of valid_mat: stalled nodes can be valid (they are XROTOR
% data), but XROTOR's 2-D post-stall model makes them low-confidence.
stall_margin = 1;
stall_mat = false(n_pitch, n_J);
J_peak    = NaN(n_pitch, 1);
for i = 1:n_pitch
    d = find(trusted_mat(i,:));
    if length(d) < 2; continue; end
    [~, k] = max(Ct_mat(i, d));
    J_peak(i) = J_vec(d(k));
    if k == 1; continue; end
    if k == length(d)
        stall_mat(i, :) = true;
    else
        stall_mat(i, 1:min(d(k) + stall_margin, n_J)) = true;
    end
end
fprintf('  Stall  : %d nodes flagged (%d of them valid)\n', ...
        sum(stall_mat(:)), sum(stall_mat(:) & valid_mat(:)));

%% ---- Metadata -----------------------------------------------------------
meta.source_file = txt_file;
meta.mat_file    = mat_file;
meta.build_time  = datestr(now, 'yyyy-mm-dd HH:MM:SS');
meta.n_rows      = n_rows;

% --- Axis descriptions ----------------------------------------------------
meta.dim1.name   = 'pitch_delta';
meta.dim1.unit   = 'deg';
meta.dim1.desc   = 'Pitch increment relative to zero-pitch (54.7 deg at r/R=0.7). Row index of all _mat grids.';
meta.dim1.values = pitch_vec;          % [n_pitch x 1]
meta.dim1.n      = n_pitch;

meta.dim2.name   = 'J';
meta.dim2.unit   = '-';
meta.dim2.desc   = 'Advance ratio J = V/(n*D). Column index of all _mat grids.';
meta.dim2.values = J_vec;              % [n_J x 1]
meta.dim2.n      = n_J;

% --- Grid descriptions ----------------------------------------------------
meta.grids.J_mat.desc      = 'Advance ratio at each grid node [−].';
meta.grids.J_mat.dim1      = 'pitch_delta [deg] — see meta.dim1.values';
meta.grids.J_mat.dim2      = 'J [−] — see meta.dim2.values';
meta.grids.J_mat.size      = sprintf('%d x %d', n_pitch, n_J);

meta.grids.Ct_mat.desc     = 'Thrust coefficient Ct = T/(rho*n^2*D^4) [−]';
meta.grids.Cp_mat.desc     = 'Power coefficient  Cp = P/(rho*n^3*D^5) [−]';

meta.grids.dCt_dbeta.desc  = 'dCt/d(pitch_delta) [1/deg] — precomputed FD on the complete grid, offline';
meta.grids.dCp_dbeta.desc  = 'dCp/d(pitch_delta) [1/deg] — precomputed FD on the complete grid, offline';
meta.grids.dCt_dJ.desc     = 'dCt/dJ [−] — precomputed FD on the complete grid, offline';
meta.grids.dCp_dJ.desc     = 'dCp/dJ [−] — precomputed FD on the complete grid, offline';

meta.grids.valid_mat.desc  = 'logical [n_pitch x n_J] — true where value and all derivatives are computed from trusted nodes only (see trusted_mat). false = extrapolated, untrusted hole, or stencil touches one.';
meta.grids.stall_mat.desc  = 'logical [n_pitch x n_J] — conservative stall mask: J at or below the Ct maximum of the pitch row (+1 node margin). J_peak holds the J of the maximum per row (NaN = none).';
meta.grids.trusted_mat.desc = 'logical [n_pitch x n_J] — original XROTOR points plus isolated filled holes whose J- and pitch-spline estimates agree.';
meta.grids.data_mat.desc   = 'logical [n_pitch x n_J] — true where an original XROTOR point exists.';
meta.grids.filled_mat.desc = 'logical [n_pitch x n_J] — true where a missing interior point was filled by spline interpolation.';

meta.grids.note = ['All grids are [n_pitch x n_J]. ' ...
    'Row i = pitch_vec(i), column j = J_vec(j). ' ...
    'All grids are complete (no NaN). Trusted nodes are original XROTOR ' ...
    'points or isolated spline-filled holes and are never modified. All ' ...
    'other nodes are minimum-curvature smoothed; beyond zero thrust they ' ...
    'follow a windmill/brake prior (four-quadrant scaling, shape fitted to ' ...
    'the Ct<0 points of the unfiltered new XROTOR run). These nodes keep ' ...
    'the solvers well-behaved and are NOT data. valid_mat is the ' ...
    'authoritative record of which cells are meaningful.'];
meta.extrapolation.method    = 'windmill prior in CT*/CP* vs geometric AoA at 0.7R + 2-D minimum-curvature smoothing';
meta.extrapolation.scaling   = 'CT* = 8 Ct/(pi W2), CP* = 8 Cp/(pi^2 0.7 W2), W2 = J^2 + (0.7 pi)^2';
meta.extrapolation.shape_src = 'xprop_data_new_unfiltered.txt, 118 points with Ct < 0';
meta.extrapolation.w_prior   = w_prior;

% --- Propeller reference --------------------------------------------------
meta.prop.D_m          = 2.35;
meta.prop.R_m          = 2.35/2;
meta.prop.A_m2         = pi*(2.35/2)^2;
meta.prop.n_blades     = 4;
meta.prop.zero_pitch_deg      = 54.7;
meta.prop.zero_pitch_location = 'r/R = 0.7';
meta.prop.RPM_table    = 1225;
meta.prop.rho_table    = 0.609542;
meta.prop.rho_unit     = 'kg/m^3';

%% ---- Save ---------------------------------------------------------------
save(mat_file, ...
     'raw', 'pitch_vec', 'J_vec', 'J_mat', 'Ct_mat', 'Cp_mat', ...
     'dCt_dbeta_mat', 'dCp_dbeta_mat', 'dCt_dJ_mat', 'dCp_dJ_mat', ...
     'valid_mat', 'data_mat', 'filled_mat', 'trusted_mat', 'stall_mat', 'J_peak', 'meta', '-v7.3');

fprintf('  Saved  : %s\n', mat_file);
fprintf('  Pitch  : [%g .. %g] deg  (%d levels): %s\n', ...
        pitch_vec(1), pitch_vec(end), n_pitch, mat2str(pitch_vec'));
fprintf('  J      : [%.4f .. %.4f]  (%d values)\n', J_vec(1), J_vec(end), n_J);
fprintf('  Grid   : %d x %d  (pitch x J),  %d valid / %d invalid cells\n', ...
        n_pitch, n_J, sum(valid_mat(:)), sum(~valid_mat(:)));
fprintf('  Grids  : Ct, Cp, dCt/dbeta, dCp/dbeta, dCt/dJ, dCp/dJ, valid_mat\n');
fprintf('Done.\n');

end


function dF = fd_deriv(F, x)
% FD_DERIV  dF/dx along dim 1 of F on a (possibly non-uniform) grid x.
% Central three-point formula inside, one-sided three-point at the borders.
x  = x(:);
n  = length(x);
dF = zeros(size(F));
if n < 3
    dF(:,:) = repmat((F(end,:) - F(1,:)) / (x(end) - x(1)), n, 1);
    return
end
for k = 1:n
    if k == 1;     c = [1 2 3];
    elseif k == n; c = [n-2 n-1 n];
    else;          c = [k-1 k k+1];
    end
    % Lagrange weights of the derivative of the quadratic through x(c) at x(k)
    xc = x(c);
    w  = zeros(1,3);
    for a = 1:3
        b = setdiff(1:3, a);
        w(a) = ((x(k) - xc(b(1))) + (x(k) - xc(b(2)))) / ...
               ((xc(a) - xc(b(1))) * (xc(a) - xc(b(2))));
    end
    dF(k,:) = w * F(c,:);
end
end

function ok = stencil_ok(T)
% STENCIL_OK  true where all nodes of the fd_deriv stencil along dim 1 are true.
n  = size(T, 1);
ok = false(size(T));
if n < 3
    ok(:,:) = repmat(all(T, 1), n, 1);
    return
end
for k = 1:n
    if k == 1;     c = [1 2 3];
    elseif k == n; c = [n-2 n-1 n];
    else;          c = [k-1 k k+1];
    end
    ok(k,:) = all(T(c,:), 1);
end
end

function h = windmill_hT(x)
% WINDMILL_HT  Normalised CT* shape beyond zero thrust, x = aoa - aoa_zt [deg].
% h = x for x >= 0 (slope 1 at zero thrust). Fitted to the Ct < 0 points of
% the unfiltered XROTOR run (new data set, x = -10.9 .. 0 deg): flattening
% turbine state, then steeper decline from negative stall (xs). Continued
% linearly with matching slope below x_end (outside the fitted range).
c = 0.1587; b = 0.6293; dq = 0.0845; xs = -6.358; x_end = -11;
core  = @(z) c*z + (1-c)*b*tanh(z/b) - dq*min(z - xs, 0).^2;
dcore = @(z) c + (1-c)*sech(z/b).^2 - 2*dq*min(z - xs, 0);
h = x;
n = x < 0;
h(n) = core(x(n));
t = x < x_end;
h(t) = core(x_end) + dcore(x_end) * (x(t) - x_end);
end

function h = windmill_hP(x)
% WINDMILL_HP  Normalised CP* shape beyond zero thrust, x = aoa - aoa_zt [deg].
% h = x for x >= 0. Same fit basis as windmill_hT: CP* first drops slightly
% below zero (windmilling), then rises again (brake state, negative stall).
q = 0.3322; xm = 5.4077;
h = x;
n = x < 0;
h(n) = x(n) + q * x(n).^2 ./ (1 + abs(x(n))/xm);
end

function F = smooth_fill(F, x, p, free, pull)
% SMOOTH_FILL  Minimum-curvature fill of the free nodes of grid F.
% Minimises the squared three-point second differences along both grid
% directions (x along dim 2, p along dim 1, non-uniform spacing) plus
% pull .* (F - F0).^2, where F0 is the current value of F. Nodes with
% free == false stay unchanged.
[np, nx] = size(F);
idx = reshape(1:np*nx, np, nx);
I = []; Jc = []; V = []; r = 0;
for i = 1:np
    for j = 2:nx-1
        h1 = x(j) - x(j-1); h2 = x(j+1) - x(j);
        r = r + 1;
        I  = [I r r r];                                            %#ok<AGROW>
        Jc = [Jc idx(i, j-1:j+1)];                                 %#ok<AGROW>
        V  = [V 2*[1/(h1*(h1+h2)), -1/(h1*h2), 1/(h2*(h1+h2))]];   %#ok<AGROW>
    end
end
for j = 1:nx
    for i = 2:np-1
        h1 = p(i) - p(i-1); h2 = p(i+1) - p(i);
        r = r + 1;
        I  = [I r r r];                                            %#ok<AGROW>
        Jc = [Jc idx(i-1:i+1, j)'];                                %#ok<AGROW>
        V  = [V 2*[1/(h1*(h1+h2)), -1/(h1*h2), 1/(h2*(h1+h2))]];   %#ok<AGROW>
    end
end
A  = sparse(I, Jc, V, r, np*nx);
f  = free(:);
F0 = F(:);
Af = A(:, f);
w  = pull(:); w = w(f);
u  = (Af'*Af + spdiags(w, 0, sum(f), sum(f))) \ (w .* F0(f) - Af'*(A(:, ~f)*F0(~f)));
F(f) = u;
end
