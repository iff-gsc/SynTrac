% XPROP_REVIEW_DATA  Rebuild both propeller tables and review one of them.
%
% Run from configuration_2a/. Builds
%   data/xprop_data.mat      from data/xprop_data.txt      (PropData 'Old')
%   data/xprop_data_new.mat  from data/xprop_data_new.txt  (PropData 'New')
% which aircraftSyntracCreate loads via its 'PropData' option.

dataset    = 'Old';   % 'Old' or 'New'
hide_stall = true;    % true: surf plots show only valid AND non-stalled nodes
show_full  = false;   % true: show the complete grid as the solver sees it,
                      %       incl. smoothed fill / extrapolation (NOT data)
use_units  = true;    % true: axes in J [-] and pitch_delta [deg], false: indices

%% Build both tables
xprop_build_matfile('data/xprop_data.txt',     'data/xprop_data.mat');
xprop_build_matfile('data/xprop_data_new.txt', 'data/xprop_data_new.mat');

%% Load the selected table
switch dataset
    case 'Old'; mat_file = 'data/xprop_data.mat';
    case 'New'; mat_file = 'data/xprop_data_new.mat';
end
tbl = xprop_load_table(mat_file);
s   = load(mat_file, 'data_mat', 'filled_mat', 'trusted_mat');

%% Axes: physical units or grid indices
if use_units
    [X, Y] = meshgrid(tbl.J_vec, tbl.pitch_vec);
    x_lbl  = 'J [-]';
    y_lbl  = '\beta_\Delta [deg]';
else
    [X, Y] = meshgrid(1:length(tbl.J_vec), 1:length(tbl.pitch_vec));
    x_lbl  = 'J index';
    y_lbl  = 'pitch index';
end

%% Plot all grids, masked with valid_mat (and optionally stall_mat)
show_mat = logical(tbl.valid_mat);
if show_full
    show_mat = true(size(show_mat));
    mask_txt = 'full grid';
elseif hide_stall
    show_mat = show_mat & ~logical(tbl.stall_mat);
    mask_txt = 'valid, no stall';
else
    mask_txt = 'valid';
end
fields = {'J_mat', 'Ct_mat', 'Cp_mat', 'dCt_dbeta_mat', 'dCp_dbeta_mat', ...
          'dCt_dJ_mat', 'dCp_dJ_mat'};
titles = {'J', 'C_t', 'C_p', 'dC_t/d\beta [1/deg]', 'dC_p/d\beta [1/deg]', ...
          'dC_t/dJ', 'dC_p/dJ'};

figure(2)
for k = 1:length(fields)
    subplot(3,3,k)
    M = tbl.(fields{k});
    M(~show_mat) = NaN;
    surf(X, Y, M)
    title([titles{k} ' (' dataset ', ' mask_txt ')'])
    xlabel(x_lbl); ylabel(y_lbl)
    axis tight
    view(-37.5+150, 60);
end

% Node classes: 0 extrapolated, 1 untrusted filled hole, 2 trusted filled
% hole, 3 original data; +4 if valid
cls = 3*double(s.data_mat) + 1*double(s.filled_mat) + double(s.trusted_mat & s.filled_mat) ...
      + 4*double(tbl.valid_mat);
s_peak = load(mat_file, 'J_peak');
if use_units
    x_peak = s_peak.J_peak;
else
    [~, x_peak] = ismember(round(s_peak.J_peak, 6), round(tbl.J_vec, 6));
    x_peak(x_peak == 0) = NaN;
end

% Physical regions for the annotation
stall_reg = logical(tbl.stall_mat) & logical(s.trusted_mat);
wind_reg  = tbl.Ct_mat < 0;                       % beyond zero thrust
band_reg  = logical(tbl.valid_mat) & ~logical(tbl.stall_mat) & tbl.Ct_mat > 0;

for sp = 8:9
    subplot(3,3,sp)
    imagesc(X(1,:), Y(:,1), cls); axis xy; caxis([0 7]); hold on
    % Stall boundary (C_t maximum per row) and zero-thrust line (C_t = 0)
    contour(X, Y, double(tbl.stall_mat), [0.5 0.5], 'm-', 'LineWidth', 2)
    plot(x_peak, Y(:,1), 'm^', 'MarkerFaceColor', 'm', 'MarkerSize', 4)
    contour(X, Y, tbl.Ct_mat, [0 0], 'r-', 'LineWidth', 2)
    if sp == 9
        region_label(X, Y, stall_reg, {'blade stall', '(high power)'}, 'm')
        region_label(X, Y, band_reg,  {'valid operating', 'band'},    'k')
        region_label(X, Y, wind_reg,  {'C_t < 0: brake /', 'windmill (no data)'}, 'r')
    end
    hold off
    xlabel(x_lbl); ylabel(y_lbl)
    if sp == 8
        colorbar
        title({'0 extrap, 1 hole, 2 trusted hole, 3 data, +4 valid', ...
               'magenta: stall boundary (C_t max), red: C_t = 0'})
    else
        title({'operating regions', ...
               'magenta: stall boundary (C_t max), red: zero thrust C_t = 0'})
    end
end

function region_label(X, Y, reg, txt, col)
% REGION_LABEL  Put a text label on the node of reg closest to its centroid.
if ~any(reg(:)); return; end
xs = X(reg); ys = Y(reg);
% normalise axes so the centroid search is not dominated by one axis
sx = max(max(X(:)) - min(X(:)), eps); sy = max(max(Y(:)) - min(Y(:)), eps);
[~, k] = min(((xs - mean(xs))/sx).^2 + ((ys - mean(ys))/sy).^2);
text(xs(k), ys(k), txt, 'Color', col, 'FontWeight', 'bold', 'FontSize', 8, ...
     'HorizontalAlignment', 'center', 'BackgroundColor', [1 1 1 0.7], 'Margin', 1)
end
