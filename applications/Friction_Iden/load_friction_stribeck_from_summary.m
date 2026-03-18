function [friction_params, I_a] = load_friction_stribeck_from_summary(n_joints, opts)
% load_friction_stribeck_from_summary  从「全部电机参数汇总」表读取 stribeck_viscous 参数与惯量
%
% 在 data/ 与 build/ 下查找 全部电机参数汇总.xlsx，筛选摩擦模型为 stribeck_viscous 的行，
% 按表顺序取前 n_joints 行，读取 τ_s、τ_c_pos/τ_c_neg、v_s、b、τ_0、I_a；缺失或 NaN 填 0。
%
% 输入：
%   n_joints - 关节数（如 6 表示左腿 6 关节）
%   opts     - 可选，结构体：
%     .data_dir     - 优先查找的目录（默认先 data 再 build）
%     .xlsx_name    - 文件名（默认 '全部电机参数汇总.xlsx'）
%     .sheet        - Sheet 名（默认 '全部电机参数汇总'）
%     .row_for_joint - 1×n_joints：关节 j 对应表中第几行（stribeck 筛选后的行号，从 1 起）。
%                       row_for_joint(j)=k 表示关节 j 用第 k 行；0 或 NaN 表示该关节参数全为 0。
%                       ［］或不传则默认：关节 1→第1行、2→第2行…，表行不足的关节填 0。
%
% 输出：
%   friction_params - 结构体，供 subtract_friction_rotor_torque 使用：
%       .model = 'stribeck_viscous'
%       .tau_s(1×n), .tau_c_pos(1×n), .tau_c_neg(1×n), .v_s(1×n), .b(1×n), .tau0(1×n)
%   I_a - 1×n 电机转子惯量 (kg·m²)，缺省处为 0
%
% 公式：τ_f = [ τ_c + (τ_s − τ_c)·exp(−(|q̇|/v_s)²) ]·sign(q̇) + b·q̇ + τ_0，
%       其中 τ_c 按 q̇ 正负取 tau_c_pos / tau_c_neg；v_s 为 0 时按 1e-6 避免除零。

if nargin < 2
    opts = struct();
end
script_dir = fileparts(mfilename('fullpath'));
if ~isfield(opts, 'data_dir'), opts.data_dir = {}; end
if ischar(opts.data_dir), opts.data_dir = {opts.data_dir}; end
if ~isfield(opts, 'xlsx_name'), opts.xlsx_name = '全部电机参数汇总.xlsx'; end
if ~isfield(opts, 'sheet'), opts.sheet = '全部电机参数汇总'; end
if ~isfield(opts, 'row_for_joint'), opts.row_for_joint = []; end

% 查找顺序：data、build
search_dirs = [opts.data_dir(:)', {fullfile(script_dir, 'data')}, {fullfile(script_dir, 'build')}];
xlsx_path = '';
for i = 1:numel(search_dirs)
    p = fullfile(search_dirs{i}, opts.xlsx_name);
    if exist(p, 'file')
        xlsx_path = p;
        break;
    end
end
if isempty(xlsx_path)
    error('load_friction_stribeck_from_summary: 未找到 %s（已查找: %s）', ...
        opts.xlsx_name, strjoin(search_dirs, ', '));
end

T = readtable(xlsx_path, 'Sheet', opts.sheet, 'VariableNamingRule', 'preserve');
vars = T.Properties.VariableNames;

% 列名匹配（兼容首列 BOM 或空格）
col = @(key) find_col(vars, key);

% 筛选 stribeck_viscous
idx_model = col('摩擦模型');
if isempty(idx_model)
    error('load_friction_stribeck_from_summary: 表中无「摩擦模型」列。');
end
model_col = T.(vars{idx_model});
if iscell(model_col)
    stribeck = cellfun(@(c) contains(c, 'stribeck', 'IgnoreCase', true), model_col);
else
    stribeck = contains(string(model_col), 'stribeck', 'IgnoreCase', true);
end
T = T(stribeck, :);
if height(T) < 1
    error('load_friction_stribeck_from_summary: 表中无 stribeck_viscous 行。');
end
N_rows = height(T);

% 关节→表行映射：row_for_joint(j)=k 表示关节 j 用第 k 行；0/NaN 表示该关节全 0
rfj = opts.row_for_joint;
if isempty(rfj)
    rfj = (1:n_joints)';
    rfj = rfj(:)';
    % 超出表行数的关节视为 0（不取行）
    rfj(rfj > N_rows) = 0;
else
    rfj = rfj(:)';
    rfj = rfj(1:min(numel(rfj), n_joints));
    if numel(rfj) < n_joints
        rfj = [rfj, zeros(1, n_joints - numel(rfj))];
    end
    rfj(rfj < 1 | rfj > N_rows | ~isfinite(rfj)) = 0;
end

% 读整列（每列 1×N_rows），再按 row_for_joint 填到 1×n_joints
vec_all = @(col_idx) col_to_vec(T, vars, col_idx, N_rows);
tau_s_all     = vec_all(col('tau_s_Nm'));
tau_c_pos_all = vec_all(col('tau_c_pos_Nm'));
tau_c_neg_all = vec_all(col('tau_c_neg_Nm'));
v_s_all       = vec_all(col('v_act_rads'));
b_all         = vec_all(col('b_Nms_rad'));
tau0_all      = vec_all(col('tau0_Nm'));
I_a_all       = vec_all(col('I_a_kgm2'));

tau_s     = zeros(1, n_joints);
tau_c_pos = zeros(1, n_joints);
tau_c_neg = zeros(1, n_joints);
v_s       = zeros(1, n_joints);
b         = zeros(1, n_joints);
tau0      = zeros(1, n_joints);
I_a       = zeros(1, n_joints);
for j = 1:n_joints
    r = rfj(j);
    if r >= 1 && r <= N_rows
        tau_s(j)     = safe_val(tau_s_all(r));
        tau_c_pos(j) = safe_val(tau_c_pos_all(r));
        tau_c_neg(j) = safe_val(tau_c_neg_all(r));
        v_s(j)       = safe_val(v_s_all(r));
        b(j)         = safe_val(b_all(r));
        tau0(j)      = safe_val(tau0_all(r));
        I_a(j)       = safe_val(I_a_all(r));
    end
end

% v_s 为 0 时避免除零（后续在 subtract 里再保护一次）
v_s(v_s <= 0) = 1e-6;

friction_params = struct();
friction_params.model      = 'stribeck_viscous';
friction_params.tau_s      = tau_s;
friction_params.tau_c_pos  = tau_c_pos;
friction_params.tau_c_neg  = tau_c_neg;
friction_params.v_s        = v_s;
friction_params.b          = b;
friction_params.tau0       = tau0;
end

function i = find_col(vars, key)
    for i = 1:numel(vars)
        if contains(vars{i}, key, 'IgnoreCase', true)
            return;
        end
    end
    i = [];
end

function v = get_col_vec(T, vars, col_idx)
    if isempty(col_idx)
        v = [];
        return;
    end
    v = T.(vars{col_idx});
    if iscell(v)
        v = cellfun(@(x) num_or_nan(x), v);
    else
        v = double(v);
    end
    v = v(:)';
end

function v = col_to_vec(T, vars, col_idx, N_rows)
    if isempty(col_idx)
        v = zeros(1, N_rows);
        return;
    end
    v = get_col_vec(T, vars, col_idx);
    v = v(:)';
    if numel(v) < N_rows
        v = [v, zeros(1, N_rows - numel(v))];
    else
        v = v(1:N_rows);
    end
    v(isnan(v)) = 0;
end

function x = safe_val(x)
    if isempty(x) || ~isfinite(x), x = 0; else, x = double(x); end
end

function x = num_or_nan(c)
    if isempty(c), x = NaN; return; end
    if isnumeric(c), x = double(c); return; end
    if ischar(c) || isstring(c)
        c = str2double(char(c));
        x = double(c);
        if isnan(c), x = NaN; end
        return;
    end
    x = NaN;
end

function out = safe_col(vec, n)
    vec = vec(:)';
    if isempty(vec), out = zeros(1, n); return; end
    vec = vec(1:min(n, numel(vec)));
    vec(isnan(vec)) = 0;
    if numel(vec) < n
        out = [vec, zeros(1, n - numel(vec))];
    else
        out = vec(1:n);
    end
end
