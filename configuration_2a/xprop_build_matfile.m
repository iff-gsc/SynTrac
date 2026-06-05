function xprop_build_matfile(txt_file, mat_file)
% XPROP_BUILD_MATFILE  Parse XROTOR plain-text data and save a .mat file.
%
% Offline preprocessing step — run once after any data change.
% Computes Ct/Cp grids AND all derivative grids (dCt/dbeta, dCp/dbeta,
% dCt/dJ, dCp/dJ) at every table node via central finite differences.
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
%   J_mat         [n_p x n_J]      advance ratio grid, NaN-padded
%   Ct_mat        [n_p x n_J]      thrust coefficient
%   Cp_mat        [n_p x n_J]      power coefficient
%   dCt_dbeta_mat [n_p x n_J]      dCt/d(pitch_delta)  [1/deg]
%   dCp_dbeta_mat [n_p x n_J]      dCp/d(pitch_delta)  [1/deg]
%   dCt_dJ_mat    [n_p x n_J]      dCt/dJ              [1/-]
%   dCp_dJ_mat    [n_p x n_J]      dCp/dJ              [1/-]
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

%% ---- Build NaN-padded Ct/Cp grids ---------------------------------------
pitch_vec = unique(raw(:,1));
n_pitch   = length(pitch_vec);
fprintf('  Pitch  : %d levels  (%g to %g deg)\n', n_pitch, pitch_vec(1), pitch_vec(end));

n_J_max = 0;
for i = 1:n_pitch
    n_J_max = max(n_J_max, sum(raw(:,1) == pitch_vec(i)));
end

J_mat  = NaN(n_pitch, n_J_max);
Ct_mat = NaN(n_pitch, n_J_max);
Cp_mat = NaN(n_pitch, n_J_max);

for i = 1:n_pitch
    idx  = raw(:,1) == pitch_vec(i);
    rows = raw(idx, :);
    [~, ord] = sort(rows(:,6));
    rows = rows(ord, :);
    n_i  = size(rows, 1);
    J_mat(i,  1:n_i) = rows(:,6);
    Ct_mat(i, 1:n_i) = rows(:,3);
    Cp_mat(i, 1:n_i) = rows(:,4);
end

%% ---- Precompute derivative grids ----------------------------------------
% All derivatives at every table node via central finite differences.
% Boundary nodes use one-sided differences.
% Units: dCt/dbeta [1/deg], dCp/dbeta [1/deg], dCt/dJ [1/-], dCp/dJ [1/-]

fprintf('  Computing derivative grids...\n');

dCt_dbeta_mat = NaN(size(Ct_mat));
dCp_dbeta_mat = NaN(size(Cp_mat));
dCt_dJ_mat    = NaN(size(Ct_mat));
dCp_dJ_mat    = NaN(size(Cp_mat));

% ---- d/d(beta): differentiate across pitch rows at each J column ---------
% For each J column, interpolate Ct/Cp as function of pitch, then differentiate.
% We use the actual pitch_vec spacing (uniform 5 deg in current table).
for j = 1:n_J_max
    for i = 1:n_pitch
        if isnan(Ct_mat(i,j)); continue; end

        % pitch derivative — central where possible, one-sided at boundaries
        if i > 1 && i < n_pitch && ~isnan(Ct_mat(i-1,j)) && ~isnan(Ct_mat(i+1,j))
            dp = pitch_vec(i+1) - pitch_vec(i-1);
            dCt_dbeta_mat(i,j) = (Ct_mat(i+1,j) - Ct_mat(i-1,j)) / dp;
            dCp_dbeta_mat(i,j) = (Cp_mat(i+1,j) - Cp_mat(i-1,j)) / dp;
        elseif i == 1 && ~isnan(Ct_mat(i+1,j))
            dp = pitch_vec(i+1) - pitch_vec(i);
            dCt_dbeta_mat(i,j) = (Ct_mat(i+1,j) - Ct_mat(i,j)) / dp;
            dCp_dbeta_mat(i,j) = (Cp_mat(i+1,j) - Cp_mat(i,j)) / dp;
        elseif i == n_pitch && ~isnan(Ct_mat(i-1,j))
            dp = pitch_vec(i) - pitch_vec(i-1);
            dCt_dbeta_mat(i,j) = (Ct_mat(i,j) - Ct_mat(i-1,j)) / dp;
            dCp_dbeta_mat(i,j) = (Cp_mat(i,j) - Cp_mat(i-1,j)) / dp;
        end
        % nodes where a neighbour is NaN (pitch boundary for this J):
        % leave as NaN — the lookup will clamp pitch to valid range anyway
    end
end

% ---- d/dJ: differentiate along J rows for each pitch level ---------------
for i = 1:n_pitch
    valid = find(~isnan(J_mat(i,:)));
    if length(valid) < 2; continue; end
    J_i  = J_mat(i,  valid);
    Ct_i = Ct_mat(i, valid);
    Cp_i = Cp_mat(i, valid);
    n_v  = length(valid);

    dCt_dJ_i = zeros(1, n_v);
    dCp_dJ_i = zeros(1, n_v);

    for jj = 1:n_v
        if jj > 1 && jj < n_v
            dJ = J_i(jj+1) - J_i(jj-1);
            dCt_dJ_i(jj) = (Ct_i(jj+1) - Ct_i(jj-1)) / dJ;
            dCp_dJ_i(jj) = (Cp_i(jj+1) - Cp_i(jj-1)) / dJ;
        elseif jj == 1
            dJ = J_i(2) - J_i(1);
            dCt_dJ_i(jj) = (Ct_i(2) - Ct_i(1)) / dJ;
            dCp_dJ_i(jj) = (Cp_i(2) - Cp_i(1)) / dJ;
        else
            dJ = J_i(end) - J_i(end-1);
            dCt_dJ_i(jj) = (Ct_i(end) - Ct_i(end-1)) / dJ;
            dCp_dJ_i(jj) = (Cp_i(end) - Cp_i(end-1)) / dJ;
        end
    end

    dCt_dJ_mat(i, valid) = dCt_dJ_i;
    dCp_dJ_mat(i, valid) = dCp_dJ_i;
end

%% ---- J axis vector (all unique J values across the full table) ----------
% J_vec is the union of all sample points from every pitch row, sorted.
% It serves as the explicit second-axis label for the NaN-padded matrices:
%   row index i  →  pitch_vec(i)   [deg]
%   col index j  →  J_vec(j)       [-]   (NaN where no data for that pitch)
J_vec = unique(raw(:,6));   % sorted ascending, no NaN

%% ---- Rebuild NaN-padded matrices aligned to J_vec ----------------------
% The matrices built earlier are padded to n_J_max per pitch row but columns
% are NOT aligned across rows (each row starts at its own minimum J).
% Rebuild so that column j always corresponds to J_vec(j) across all rows.
n_J       = length(J_vec);
J_mat_aligned  = NaN(n_pitch, n_J);
Ct_mat_aligned = NaN(n_pitch, n_J);
Cp_mat_aligned = NaN(n_pitch, n_J);
dCt_dbeta_aligned = NaN(n_pitch, n_J);
dCp_dbeta_aligned = NaN(n_pitch, n_J);
dCt_dJ_aligned    = NaN(n_pitch, n_J);
dCp_dJ_aligned    = NaN(n_pitch, n_J);

for i = 1:n_pitch
    valid  = ~isnan(J_mat(i,:));
    J_i    = J_mat(i, valid);
    [~, col_idx] = ismember(round(J_i,6), round(J_vec,6));
    J_mat_aligned(i,  col_idx) = J_i;
    Ct_mat_aligned(i, col_idx) = Ct_mat(i, valid);
    Cp_mat_aligned(i, col_idx) = Cp_mat(i, valid);
    dCt_dbeta_aligned(i, col_idx) = dCt_dbeta_mat(i, valid);
    dCp_dbeta_aligned(i, col_idx) = dCp_dbeta_mat(i, valid);
    dCt_dJ_aligned(i,    col_idx) = dCt_dJ_mat(i,    valid);
    dCp_dJ_aligned(i,    col_idx) = dCp_dJ_mat(i,    valid);
end

% Replace working matrices with aligned versions
J_mat         = J_mat_aligned;
Ct_mat        = Ct_mat_aligned;
Cp_mat        = Cp_mat_aligned;
dCt_dbeta_mat = dCt_dbeta_aligned;
dCp_dbeta_mat = dCp_dbeta_aligned;
dCt_dJ_mat    = dCt_dJ_aligned;
dCp_dJ_mat    = dCp_dJ_aligned;

%% ---- Valid mask ---------------------------------------------------------
% Capture the NaN pattern now, before NaNs are stripped for Simulink.
% valid_mat(i,j) = true  → real XROTOR data exists at (pitch_vec(i), J_vec(j))
% valid_mat(i,j) = false → padded cell, no data for this combination
% Same dimensions as all _mat grids: [n_pitch x n_J]
valid_mat = ~isnan(Ct_mat);   % true = real XROTOR data, false = will be extrapolated

%% ---- Fill ragged row edges by linear extrapolation ---------------------
% Rows with shorter J extent cause slope discontinuities when blended with
% a longer neighbour row during pitch interpolation — one row's value freezes
% at its boundary while the other keeps moving. Fix: extrapolate each short
% row linearly to cover the full J_vec so every row is complete.
% valid_mat records which cells are original data vs extrapolated.

mats_to_fill = {Ct_mat, Cp_mat, dCt_dbeta_mat, dCp_dbeta_mat, dCt_dJ_mat, dCp_dJ_mat};
n_mats = length(mats_to_fill);

for i = 1:n_pitch
    % Find last valid column for this row
    last_valid = 0;
    for j = n_J:-1:1
        if valid_mat(i,j); last_valid = j; break; end
    end
    if last_valid == 0 || last_valid == n_J; continue; end

    % Fill J_mat columns (just copy from J_vec)
    for j = last_valid+1 : n_J
        J_mat(i,j) = J_vec(j);
    end

    % Linearly extrapolate each data matrix using slope of last two valid points
    for m = 1:n_mats
        mat = mats_to_fill{m};
        if last_valid >= 2
            slope = mat(i, last_valid) - mat(i, last_valid-1);
        else
            slope = 0;
        end
        for j = last_valid+1 : n_J
            mat(i,j) = mat(i, j-1) + slope;
        end
        mats_to_fill{m} = mat;
    end
end

Ct_mat        = mats_to_fill{1};
Cp_mat        = mats_to_fill{2};
dCt_dbeta_mat = mats_to_fill{3};
dCp_dbeta_mat = mats_to_fill{4};
dCt_dJ_mat    = mats_to_fill{5};
dCp_dJ_mat    = mats_to_fill{6};

fprintf('  Filled : %d extrapolated cells across %d rows\n', ...
        sum(~valid_mat(:)), n_pitch);

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
meta.dim2.desc   = 'Advance ratio J = V/(n*D). Column index of all _mat grids. NaN where no XROTOR data exists for that (pitch, J) combination.';
meta.dim2.values = J_vec;              % [n_J x 1]
meta.dim2.n      = n_J;

% --- Grid descriptions ----------------------------------------------------
meta.grids.J_mat.desc      = 'Advance ratio at each grid node [−]. NaN where no data.';
meta.grids.J_mat.dim1      = 'pitch_delta [deg] — see meta.dim1.values';
meta.grids.J_mat.dim2      = 'J [−] — see meta.dim2.values';
meta.grids.J_mat.size      = sprintf('%d x %d', n_pitch, n_J);

meta.grids.Ct_mat.desc     = 'Thrust coefficient Ct = T/(rho*n^2*D^4) [−]';
meta.grids.Cp_mat.desc     = 'Power coefficient  Cp = P/(rho*n^3*D^5) [−]';

meta.grids.dCt_dbeta.desc  = 'dCt/d(pitch_delta) [1/deg] — precomputed central FD, offline';
meta.grids.dCp_dbeta.desc  = 'dCp/d(pitch_delta) [1/deg] — precomputed central FD, offline';
meta.grids.dCt_dJ.desc     = 'dCt/dJ [−] — precomputed central FD, offline';
meta.grids.dCp_dJ.desc     = 'dCp/dJ [−] — precomputed central FD, offline';

meta.grids.valid_mat.desc = 'logical [n_pitch x n_J] — true where real XROTOR data exists. false = padded cell. Use to mask any post-processing on the grids.';

meta.grids.note = ['All grids are [n_pitch x n_J]. ' ...
    'Row i = pitch_vec(i), column j = J_vec(j). ' ...
    'valid_mat(i,j) flags which cells contain real data. ' ...
    'In the runtime tbl struct all grids are zero-padded (NaNs replaced) ' ...
    'so Simulink bus Constant blocks accept them; valid_mat is the ' ...
    'authoritative record of which cells are meaningful.'];

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
     'valid_mat', 'meta', '-v7.3');

fprintf('  Saved  : %s\n', mat_file);
fprintf('  Pitch  : [%g .. %g] deg  (%d levels): %s\n', ...
        pitch_vec(1), pitch_vec(end), n_pitch, mat2str(pitch_vec'));
fprintf('  J      : [%.4f .. %.4f]  (%d values)\n', J_vec(1), J_vec(end), n_J);
fprintf('  Grid   : %d x %d  (pitch x J),  %d valid / %d padded cells\n', ...
        n_pitch, n_J, sum(valid_mat(:)), sum(~valid_mat(:)));
fprintf('  Grids  : Ct, Cp, dCt/dbeta, dCp/dbeta, dCt/dJ, dCp/dJ, valid_mat\n');
fprintf('Done.\n');

end

