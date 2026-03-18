function data = read_leg_joint_csv(csv_file)
% read_leg_joint_csv  读取腿关节实机/仿真 CSV，按左腿 n×6、右腿 n×6 组织
%
% 支持两种格式（自动检测）：
%
% 【老格式】表头：time, cmd_leg_l1_joint, pos_leg_l1_joint, vel_leg_l1_joint, torque_leg_l1_joint, ...
%           (l2..l6 同理), cmd_leg_r1_joint, ... (r2..r6 同理)
%           可选列：acc_leg_l1_joint ... acc_leg_r6_joint；cmd_torque_leg_l1_joint ... cmd_torque_leg_r6_joint（指令力矩）
%
% 【新格式】表头：time, q_1_data, q_2_data, ..., qd_1_data, ..., tau_1_data, ...
%           以及可选 _sim 列：q_1_sim, qd_1_sim, tau_1_sim, ...
%           可选：qdd_1_data, qdd_1_sim；基座四元数等列会被忽略。
%           关节数由 q_i_data 列数推断（如 7 关节则左腿 1:6、右腿第 7 列+补零）。
%
% 输出 struct data（两种格式统一）：
%   data.time        n×1
%   data.cmd_leg_l   n×6   data.pos_leg_l n×6  data.vel_leg_l n×6  data.torque_leg_l n×6
%   data.cmd_leg_r   n×6   data.pos_leg_r n×6  data.vel_leg_r n×6  data.torque_leg_r n×6
%   data.acc_leg_l   n×6   data.acc_leg_r n×6  （仅当 CSV 含 acc_* 列时存在）
%   data.cmd_torque_leg_l n×6  data.cmd_torque_leg_r n×6  （仅当 CSV 含 cmd_torque_leg_* 列时存在）
%   data.format      'leg_joint' | 'q_data_sim'  供调用方区分来源
%
% 用法：data = read_leg_joint_csv('path/to/file.csv');

if ~exist(csv_file, 'file')
    error('文件不存在: %s', csv_file);
end

T = readtable(csv_file);
vars = T.Properties.VariableNames;

% 检测格式：是否存在老格式列名（pos_leg_l1_joint 或 cmd_leg_l1_joint）
[~, ~, joint_suffix] = get_e1_leg_joint_names();
old_col = ['pos_', joint_suffix{1}];
has_old = ismember(old_col, vars);
if ~has_old
    old_col_cmd = ['cmd_', joint_suffix{1}];
    has_old = ismember(old_col_cmd, vars);
end

if has_old
    data = read_leg_joint_csv_old(T, joint_suffix);
    data.format = 'leg_joint';
    return;
end

% 新格式：q_1_data, q_2_data, ... 及 qd_*, tau_*
if ismember('q_1_data', vars)
    data = read_leg_joint_csv_new(T, vars);
    data.format = 'q_data_sim';
    return;
end

error('read_leg_joint_csv: 未识别的 CSV 格式。需要老格式（pos_leg_l1_joint 等）或新格式（q_1_data, qd_1_data, tau_1_data 等）。');
end

%% 老格式：pos_leg_l1_joint, vel_leg_l1_joint, torque_leg_l1_joint, ...
function data = read_leg_joint_csv_old(T, joint_suffix)
data.time = T.(T.Properties.VariableNames{1});
n = length(data.time);

data.cmd_leg_l    = zeros(n, 6);
data.pos_leg_l   = zeros(n, 6);
data.vel_leg_l   = zeros(n, 6);
data.torque_leg_l = zeros(n, 6);
for j = 1:6
    data.cmd_leg_l(:, j)    = T.(['cmd_', joint_suffix{j}]);
    data.pos_leg_l(:, j)    = T.(['pos_', joint_suffix{j}]);
    data.vel_leg_l(:, j)    = T.(['vel_', joint_suffix{j}]);
    data.torque_leg_l(:, j) = T.(['torque_', joint_suffix{j}]);
end

data.cmd_leg_r    = zeros(n, 6);
data.pos_leg_r   = zeros(n, 6);
data.vel_leg_r   = zeros(n, 6);
data.torque_leg_r = zeros(n, 6);
for j = 1:6
    data.cmd_leg_r(:, j)    = T.(['cmd_', joint_suffix{6+j}]);
    data.pos_leg_r(:, j)    = T.(['pos_', joint_suffix{6+j}]);
    data.vel_leg_r(:, j)    = T.(['vel_', joint_suffix{6+j}]);
    data.torque_leg_r(:, j) = T.(['torque_', joint_suffix{6+j}]);
end

vars = T.Properties.VariableNames;
if ismember(['acc_', joint_suffix{1}], vars)
    data.acc_leg_l = zeros(n, 6);
    data.acc_leg_r = zeros(n, 6);
    for j = 1:6
        data.acc_leg_l(:, j) = T.(['acc_', joint_suffix{j}]);
        data.acc_leg_r(:, j) = T.(['acc_', joint_suffix{6+j}]);
    end
end
% 可选：指令力矩 cmd_torque_leg_l1_joint ... cmd_torque_leg_r6_joint
if ismember(['cmd_torque_', joint_suffix{1}], vars)
    data.cmd_torque_leg_l = zeros(n, 6);
    data.cmd_torque_leg_r = zeros(n, 6);
    for j = 1:6
        data.cmd_torque_leg_l(:, j) = T.(['cmd_torque_', joint_suffix{j}]);
        data.cmd_torque_leg_r(:, j) = T.(['cmd_torque_', joint_suffix{6+j}]);
    end
end
end

%% 新格式：q_1_data, q_2_data, ..., qd_1_data, ..., tau_1_data, ...（关节数可变，映射到 12 维腿）
function data = read_leg_joint_csv_new(T, vars)
% 推断关节数
n_joints = 0;
for i = 1:24
    if ismember(sprintf('q_%d_data', i), vars)
        n_joints = i;
    else
        break;
    end
end
if n_joints == 0
    error('read_leg_joint_csv_new: 未找到 q_1_data, q_2_data, ... 列');
end

data.time = T.(vars{1});
n = length(data.time);

% 统一输出 12 维腿：左 6 + 右 6
data.pos_leg_l   = zeros(n, 6);
data.vel_leg_l   = zeros(n, 6);
data.torque_leg_l = zeros(n, 6);
data.cmd_leg_l   = zeros(n, 6);
data.pos_leg_r   = zeros(n, 6);
data.vel_leg_r   = zeros(n, 6);
data.torque_leg_r = zeros(n, 6);
data.cmd_leg_r   = zeros(n, 6);

for j = 1:min(6, n_joints)
    data.pos_leg_l(:, j)    = T.(sprintf('q_%d_data', j));
    data.vel_leg_l(:, j)    = T.(sprintf('qd_%d_data', j));
    data.torque_leg_l(:, j) = T.(sprintf('tau_%d_data', j));
    if ismember(sprintf('q_%d_sim', j), vars)
        data.cmd_leg_l(:, j) = T.(sprintf('q_%d_sim', j));
    end
end
for j = 1:min(6, max(0, n_joints - 6))
    ji = 6 + j;
    data.pos_leg_r(:, j)    = T.(sprintf('q_%d_data', ji));
    data.vel_leg_r(:, j)    = T.(sprintf('qd_%d_data', ji));
    data.torque_leg_r(:, j) = T.(sprintf('tau_%d_data', ji));
    if ismember(sprintf('q_%d_sim', ji), vars)
        data.cmd_leg_r(:, j) = T.(sprintf('q_%d_sim', ji));
    end
end

% 可选：加速度 qdd_i_data
if ismember('qdd_1_data', vars)
    data.acc_leg_l = zeros(n, 6);
    data.acc_leg_r = zeros(n, 6);
    for j = 1:min(6, n_joints)
        data.acc_leg_l(:, j) = T.(sprintf('qdd_%d_data', j));
    end
    for j = 1:min(6, max(0, n_joints - 6))
        data.acc_leg_r(:, j) = T.(sprintf('qdd_%d_data', 6+j));
    end
end
end
