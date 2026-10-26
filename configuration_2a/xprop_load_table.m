function [tbl, tbl_bus, tbl_meta] = xprop_load_table(mat_file)
% XPROP_LOAD_TABLE  Load propeller data table and create Simulink Bus object.
%
% Call ONCE in your init script or Simulink model InitFcn.
%
% USAGE:
%   [tbl, tbl_bus] = xprop_load_table('xprop_data.mat');
%
% Simulink InitFcn:
%   [tbl, tbl_bus] = xprop_load_table('xprop_data.mat');
%   % tbl      → assign as bus signal source (Constant block with bus type)
%   % tbl_bus  → automatically registered in base workspace as 'xprop_tbl_bus'
%   %            and in Simulink.data.dictionary if one is attached
%
% Then in the MATLAB Function block:
%   - Add one input port, name it 'tbl', set data type to Bus: xprop_tbl_bus
%   - Access fields as tbl.pitch_vec, tbl.J_mat, tbl.Ct_mat etc.
%
% OUTPUTS:
%   tbl      - plain MATLAB struct (for MATLAB-side use and as bus signal data)
%   tbl_bus  - Simulink.Bus object describing the bus type
%              Also written to base workspace as 'xprop_tbl_bus' so Simulink
%              can resolve the bus type by name in the block dialog.
%
% STRUCT FIELDS (= bus elements):
%   .pitch_vec      [n_p x 1]    pitch_delta axis [deg]
%   .J_vec          [n_J x 1]    advance ratio axis [-]
%   .J_mat          [n_p x n_J]  advance ratio grid
%   .Ct_mat         [n_p x n_J]  thrust coefficient [-]
%   .Cp_mat         [n_p x n_J]  power coefficient [-]
%   .dCt_dbeta_mat  [n_p x n_J]  dCt/d(pitch_delta) [1/deg]
%   .dCp_dbeta_mat  [n_p x n_J]  dCp/d(pitch_delta) [1/deg]
%   .dCt_dJ_mat     [n_p x n_J]  dCt/dJ [-]
%   .dCp_dJ_mat     [n_p x n_J]  dCp/dJ [-]
%   .valid_mat      [n_p x n_J]  uint8, 1 = trustworthy node
%   .stall_mat      [n_p x n_J]  uint8, 1 = stalled side of the pitch row

%% ---- Default path -------------------------------------------------------
if nargin < 1 || isempty(mat_file)
    here     = fileparts(mfilename('fullpath'));
    mat_file = fullfile(here, 'xprop_data.mat');
end

if ~isfile(mat_file)
    error('xprop_load_table:notFound', ...
        ['Mat file not found: %s\n' ...
         'Generate with: xprop_build_matfile(''xprop_data.txt'')'], mat_file);
end

%% ---- Load and validate --------------------------------------------------
s = load(mat_file);

required = {'pitch_vec','J_vec','J_mat','Ct_mat','Cp_mat', ...
            'dCt_dbeta_mat','dCp_dbeta_mat','dCt_dJ_mat','dCp_dJ_mat', ...
            'valid_mat', 'stall_mat'};
for k = 1:length(required)
    if ~isfield(s, required{k})
        error('xprop_load_table:missingVar', ...
            'Variable ''%s'' missing. Regenerate with xprop_build_matfile().', ...
            required{k});
    end
end

%% ---- Build tbl struct ---------------------------------------------------
% xprop_build_matfile delivers complete grids (holes filled, envelope
% extrapolated). Simulink bus Constant blocks require all values to be
% finite, so any remaining NaN (e.g. from an old .mat file) is replaced
% with 0 as a safeguard. valid_mat marks the trustworthy nodes.
nan0 = @(x) subsasgn(x, substruct('()', {isnan(x)}), 0);

tbl.pitch_vec     = s.pitch_vec;
tbl.J_vec         = s.J_vec;
tbl.J_mat         = nan0(s.J_mat);
tbl.Ct_mat        = nan0(s.Ct_mat);
tbl.Cp_mat        = nan0(s.Cp_mat);
tbl.dCt_dbeta_mat = nan0(s.dCt_dbeta_mat);
tbl.dCp_dbeta_mat = nan0(s.dCp_dbeta_mat);
tbl.dCt_dJ_mat    = nan0(s.dCt_dJ_mat);
tbl.dCp_dJ_mat    = nan0(s.dCp_dJ_mat);
tbl.valid_mat     = s.valid_mat;       
tbl.stall_mat     = s.stall_mat;

tbl_meta = s.meta;

%% ---- Build Simulink Bus object ------------------------------------------
% Each numeric field becomes one BusElement with exact dimensions and type.
% meta is excluded — it contains strings and nested structs which buses
% cannot represent. All data fields are double.

bus_fields = {'pitch_vec', 'J_vec', 'J_mat', 'Ct_mat', 'Cp_mat', ...
              'dCt_dbeta_mat', 'dCp_dbeta_mat', 'dCt_dJ_mat', 'dCp_dJ_mat', ...
              'valid_mat', 'stall_mat'};

bus_descriptions = { ...
    'pitch_delta axis [deg], row index of all _mat grids', ...
    'Advance ratio axis [-], column index of all _mat grids', ...
    'Advance ratio at each grid node [-], 0 where no data', ...
    'Thrust coefficient Ct = T/(rho*n^2*D^4) [-], 0 where no data', ...
    'Power coefficient Cp = P/(rho*n^3*D^5) [-], 0 where no data', ...
    'dCt/d(pitch_delta) [1/deg], precomputed offline, 0 where no data', ...
    'dCp/d(pitch_delta) [1/deg], precomputed offline, 0 where no data', ...
    'dCt/dJ [-], precomputed offline, 0 where no data', ...
    'dCp/dJ [-], precomputed offline, 0 where no data', ...
    'Valid cell mask [uint8]: 1 = value and derivatives from trusted data, 0 = not', ...
    'Stall mask [uint8]: 1 = stalled side of the pitch row (conservative), 0 = not'};

% valid_mat is logical in MATLAB; cast to uint8 for bus compatibility
tbl.valid_mat = uint8(tbl.valid_mat);
tbl.stall_mat = uint8(tbl.stall_mat);

n_fields = length(bus_fields);
elems(n_fields) = Simulink.BusElement;   % pre-allocate

for k = 1:n_fields
    e                = Simulink.BusElement;
    e.Name           = bus_fields{k};
    e.DataType       = 'double';
    if any(strcmp(bus_fields{k}, {'valid_mat', 'stall_mat'}))
        e.DataType   = 'uint8';
    end
    e.Dimensions     = size(tbl.(bus_fields{k}));
    e.DimensionsMode = 'Fixed';
    e.SampleTime     = -1;
    e.Complexity     = 'real';
    e.Description    = bus_descriptions{k};
    elems(k)         = e;
end

tbl_bus             = Simulink.Bus;
tbl_bus.Description = sprintf('XPROP propeller data table. Source: %s', mat_file);
tbl_bus.Elements    = elems;

%% ---- Register bus in base workspace so Simulink can resolve by name -----
% The bus type name 'xprop_tbl_bus' is what you enter in the block dialog
% under Signal Attributes → Data type → Bus: xprop_tbl_bus
assignin('base', 'xprop_tbl_bus', tbl_bus);

%% ---- Print summary ------------------------------------------------------
fprintf('xprop_load_table: loaded %s\n', mat_file);
fprintf('  Pitch : [%g .. %g] deg  (%d levels)\n', ...
        tbl.pitch_vec(1), tbl.pitch_vec(end), length(tbl.pitch_vec));
fprintf('  J     : [%.4f .. %.4f]  (%d values)\n', ...
        tbl.J_vec(1), tbl.J_vec(end), length(tbl.J_vec));
fprintf('  Grid  : %d x %d  (pitch x J)\n', size(tbl.J_mat));
fprintf('  Bus   : ''xprop_tbl_bus'' registered in base workspace\n');

end