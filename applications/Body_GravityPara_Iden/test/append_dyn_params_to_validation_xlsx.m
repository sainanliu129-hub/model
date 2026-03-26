%% append_dyn_params_to_validation_xlsx
% 在不重新跑完整验证的前提下：
%   1) 从 build/step5_full_params.mat 读取 pi_cad/pi_rec/pi_phys/pi_fd
%   2) 读取现有 full_dynamics_validation_summary.xlsx
%   3) 仅写入/覆盖 sheet: 'DYN_PARAMS'
%
% 用法：
%   - 修改顶部配置后直接运行本脚本
%
clc; clear; close all;

app_root = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(app_root, 'principle'));
addpath(app_root);
addpath(fullfile(app_root, 'test'));

%% 1) 配置
build_dir = fullfile(app_root, 'build');

step5_mat = fullfile(build_dir, 'step5_full_params.mat');
xlsx_file = fullfile(build_dir, 'full_dynamics_validation_summary.xlsx');

limb = 'left_leg';     % 'left_leg'|'right_leg'|'left_arm'|'right_arm'
para_order = 1;        % 1 or 2，必须与 step5 的约定一致

% 若你只关心某几个模型，也可以在脚本里把空的 pi_* 留着
force_psd = false;    % 本脚本仅写参数到表，不会投影，保留原值

if ~isfile(step5_mat)
    error('缺少 step5 参数文件: %s', step5_mat);
end
if ~isfile(xlsx_file)
    error('缺少验证 xlsx 文件: %s', xlsx_file);
end

%% 2) 读取 pi_*（辨识结果）
S = load(step5_mat);
pi_cad  = get_field_or_empty(S, 'pi_cad');
pi_rec  = get_field_or_empty(S, 'pi_rec');
pi_phys = get_field_or_empty(S, 'pi_phys');
pi_fd   = get_field_or_empty(S, 'pi_fd');

%% 3) 构造参数行名（与 pi_vec 的 Bodies 顺序一致）
[robot_limb, n] = get_e1_limb_robot(limb);
L = 10 * n;

bodyNames = cell(1, n);
for i = 1:n
    bodyNames{i} = robot_limb.Bodies{i}.Name;
end

if para_order == 1
    comp_names = {'m','mx','my','mz','Ixx','Ixy','Ixz','Iyy','Iyz','Izz'};
else
    comp_names = {'Ixx','Ixy','Ixz','Iyy','Iyz','Izz','mx','my','mz','m'};
end

param_name = cell(L, 1);
for i = 1:n
    for j = 1:10
        idx = (i - 1) * 10 + j;
        param_name{idx} = sprintf('%s_%s', bodyNames{i}, comp_names{j});
    end
end

%% 4) 对齐 pi_* 到同一长度 L（不足补 NaN，超长截断）
cad_vals  = align_pi_to_L(pi_cad,  L);
rec_vals  = align_pi_to_L(pi_rec,  L);
phys_vals = align_pi_to_L(pi_phys, L);
fd_vals   = align_pi_to_L(pi_fd,   L);

dyn_params_table = table(param_name, cad_vals, rec_vals, phys_vals, fd_vals, ...
    'VariableNames', {'param','pi_cad','pi_rec','pi_phys','pi_fd'});

%% 5) 写入 xlsx（只覆盖 DYN_PARAMS sheet）
writetable(dyn_params_table, xlsx_file, 'Sheet', 'DYN_PARAMS');
fprintf('已更新 %s：写入 sheet = DYN_PARAMS（rows=%d）\n', xlsx_file, height(dyn_params_table));

%% ===== local helpers =====
function v = get_field_or_empty(S, name)
if isfield(S, name) && ~isempty(S.(name))
    v = S.(name)(:);
else
    v = [];
end
end

function vals = align_pi_to_L(pi_vec, L)
vals = nan(L, 1);
if isempty(pi_vec)
    return;
end
v = pi_vec(:);
m = min(numel(v), L);
vals(1:m) = v(1:m);
end

